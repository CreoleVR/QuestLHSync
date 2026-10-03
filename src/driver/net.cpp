#include "net.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>

#include "sync.h"

static const int kUdpPort = 47281, kTcpPort = 47280;

double QpcNow() {
  static LARGE_INTEGER f = [] { LARGE_INTEGER v; QueryPerformanceFrequency(&v); return v; }();
  LARGE_INTEGER c;
  QueryPerformanceCounter(&c);
  return (double)(c.QuadPart / f.QuadPart) + (double)(c.QuadPart % f.QuadPart) / (double)f.QuadPart;
}

void HeadsetLink::Start() {
  if (run_) return;
  run_ = true;
  th_ = std::thread(&HeadsetLink::Loop, this);
}

void HeadsetLink::Stop() {
  run_ = false;
  if (th_.joinable()) th_.join();
}

void HeadsetLink::SetHosts(const std::vector<std::string> &h) {
  std::lock_guard<std::mutex> g(m_);
  hosts_ = h;
}

void HeadsetLink::SetPreferred(const std::string &serial) {
  std::lock_guard<std::mutex> g(m_);
  preferred_ = serial;
}

void HeadsetLink::SetMemory(const std::string &path) {
  std::lock_guard<std::mutex> g(m_);
  mem_path_ = path;
  last_ip_.clear();
  if (FILE *f = fopen(path.c_str(), "r")) {
    char ip[64];
    if (fscanf(f, "%63s", ip) == 1) last_ip_ = ip;
    fclose(f);
  }
}

static void Sleep_(std::atomic<bool> &run, int ms) {
  for (int i = 0; i < ms / 50 && run; i++) Sleep(50);
}

void HeadsetLink::Loop() {
  WSADATA wd;
  WSAStartup(MAKEWORD(2, 2), &wd);
  int quiet = 0;
  while (run_) {
    if (!wanted_) { state_ = kIdle; Sleep_(run_, 500); continue; }
    state_ = kSearching;
    std::vector<Found> found;
    Discover(found);
    std::string pref;
    {
      std::lock_guard<std::mutex> g(m_);
      pref = preferred_;
    }
    const Found *pick = nullptr;
    for (auto &f : found)
      if (!pref.empty() && f.serial == pref) pick = &f;
    if (!pick && !found.empty()) pick = &found[0];
    if (!pick) {
      if (quiet++ % 30 == 0) log_("no QuestLHSync headset answers on the network (is it awake, with lhsyncd installed?)");
      // straight over TCP: manual hosts (a firewall eating UDP), then the last headset (a network dropping broadcasts)
      std::vector<std::string> direct;
      {
        std::lock_guard<std::mutex> g(m_);
        direct = hosts_;
        if (!last_ip_.empty() && std::find(direct.begin(), direct.end(), last_ip_) == direct.end())
          direct.push_back(last_ip_);
      }
      bool talked = false;
      for (auto &h : direct)
        if (run_ && wanted_ && !talked) talked = Session({h, "", "", kTcpPort});
      if (talked) quiet = 0;
      Sleep_(run_, 2000);
      continue;
    }
    quiet = 0;
    Session(*pick);
    state_ = kSearching;
    Sleep_(run_, 2000);
  }
  WSACleanup();
}

// directed broadcast address of every IPv4 interface that's up
static std::vector<sockaddr_in> Broadcasts() {
  std::vector<sockaddr_in> out;
  ULONG sz = 16384;
  std::vector<unsigned char> buf(sz);
  if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER, nullptr,
                           (IP_ADAPTER_ADDRESSES *)buf.data(), &sz) == ERROR_BUFFER_OVERFLOW) {
    buf.resize(sz);
    if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER, nullptr,
                             (IP_ADAPTER_ADDRESSES *)buf.data(), &sz) != NO_ERROR)
      return out;
  }
  for (auto *a = (IP_ADAPTER_ADDRESSES *)buf.data(); a; a = a->Next) {
    if (a->OperStatus != IfOperStatusUp || a->IfType == IF_TYPE_SOFTWARE_LOOPBACK) continue;
    for (auto *u = a->FirstUnicastAddress; u; u = u->Next) {
      auto *sin = (sockaddr_in *)u->Address.lpSockaddr;
      if (sin->sin_family != AF_INET || u->OnLinkPrefixLength == 0 || u->OnLinkPrefixLength >= 31) continue;
      uint32_t ip = ntohl(sin->sin_addr.s_addr), mask = 0xFFFFFFFFu << (32 - u->OnLinkPrefixLength);
      sockaddr_in b{};
      b.sin_family = AF_INET;
      b.sin_port = htons(kUdpPort);
      b.sin_addr.s_addr = htonl(ip | ~mask);
      out.push_back(b);
    }
  }
  return out;
}

bool HeadsetLink::Discover(std::vector<Found> &out) {
  SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s == INVALID_SOCKET) return false;
  BOOL on = TRUE;
  setsockopt(s, SOL_SOCKET, SO_BROADCAST, (const char *)&on, sizeof on);
  sockaddr_in any{};
  any.sin_family = AF_INET;
  bind(s, (sockaddr *)&any, sizeof any);
  std::vector<sockaddr_in> dst = Broadcasts();
  sockaddr_in all{};
  all.sin_family = AF_INET;
  all.sin_port = htons(kUdpPort);
  all.sin_addr.s_addr = INADDR_BROADCAST;
  dst.push_back(all);
  std::vector<std::string> hosts;
  {
    std::lock_guard<std::mutex> g(m_);
    hosts = hosts_;
  }
  for (auto &h : hosts) {
    addrinfo hint{}, *res = nullptr;
    hint.ai_family = AF_INET;
    hint.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(h.c_str(), nullptr, &hint, &res) == 0 && res) {
      sockaddr_in a = *(sockaddr_in *)res->ai_addr;
      a.sin_port = htons(kUdpPort);
      dst.push_back(a);
      freeaddrinfo(res);
    }
  }
  const char q[] = "QLHS?";
  for (int round = 0; round < 2; round++)
    for (auto &d : dst) sendto(s, q, (int)strlen(q), 0, (sockaddr *)&d, sizeof d);
  std::set<std::string> seen;
  auto until = QpcNow() + 0.6;
  for (;;) {
    double left = until - QpcNow();
    if (left <= 0) break;
    fd_set rf;
    FD_ZERO(&rf);
    FD_SET(s, &rf);
    timeval tv{0, (long)(left * 1e6)};
    if (select(0, &rf, nullptr, nullptr, &tv) <= 0) break;
    char buf[512];
    sockaddr_in from{};
    int fl = sizeof from;
    int n = recvfrom(s, buf, sizeof buf - 1, 0, (sockaddr *)&from, &fl);
    if (n <= 0) continue;
    buf[n] = 0;
    char serial[64] = "", model[128] = "";
    int ver = 0, port = 0;
    if (sscanf(buf, "QLHS %d %63s %d %127s", &ver, serial, &port, model) < 3 || ver != 1) continue;
    char ip[64];
    inet_ntop(AF_INET, &from.sin_addr, ip, sizeof ip);
    if (!seen.insert(serial).second) continue;
    for (char *c = model; *c; c++) if (*c == '_') *c = ' ';
    out.push_back({ip, serial, model, port ? port : kTcpPort});
  }
  closesocket(s);
  return !out.empty();
}

static bool SendAll(SOCKET s, const std::string &d) {
  size_t o = 0;
  while (o < d.size()) {
    int n = send(s, d.data() + o, (int)(d.size() - o), 0);
    if (n <= 0) return false;
    o += n;
  }
  return true;
}

bool HeadsetLink::Session(const Found &f) {
  state_ = kConnecting;
  addrinfo hint{}, *res = nullptr;
  hint.ai_family = AF_INET;
  hint.ai_socktype = SOCK_STREAM;
  char port[16];
  snprintf(port, sizeof port, "%d", f.port);
  if (getaddrinfo(f.ip.c_str(), port, &hint, &res) != 0 || !res) return false;
  SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  u_long nb = 1;
  ioctlsocket(s, FIONBIO, &nb);
  connect(s, res->ai_addr, (int)res->ai_addrlen);
  freeaddrinfo(res);
  fd_set wf, ef;
  FD_ZERO(&wf); FD_SET(s, &wf);
  FD_ZERO(&ef); FD_SET(s, &ef);
  timeval tv{3, 0};
  if (select(0, nullptr, &wf, &ef, &tv) <= 0 || FD_ISSET(s, &ef)) {
    std::lock_guard<std::mutex> g(m_);
    if (failed_ip_ != f.ip) log_("headset " + f.ip + ": TCP connect failed");  // once per address, not every retry
    failed_ip_ = f.ip;
    closesocket(s);
    return false;
  }
  nb = 0;
  ioctlsocket(s, FIONBIO, &nb);
  BOOL on = TRUE;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&on, sizeof on);
  setsockopt(s, SOL_SOCKET, SO_KEEPALIVE, (const char *)&on, sizeof on);
  {
    std::lock_guard<std::mutex> g(m_);
    addr_ = f.ip + ":" + port;
    serial_ = f.serial;
    model_ = f.model;
    fw_.clear();
    failed_ip_.clear();
  }
  sync_->HeadsetReset();
  std::string in;
  in.reserve(1 << 20);
  size_t calib_need = 0;
  std::string calib_name;
  bool have_calib = false;
  uint32_t seq = 0;
  double next_ping = 0, last_rx = QpcNow(), start = QpcNow();
  connected_at_ = start;
  state_ = kConnected;
  log_((f.model.empty() ? std::string("headset") : f.model) + " at " + f.ip + ": connected");
  bool ok = true;
  char buf[65536];
  while (run_ && wanted_ && ok) {
    double now = QpcNow();
    if (now >= next_ping) {  // clock round trips: 4 Hz
      next_ping = now + 0.25;
      char p[64];
      snprintf(p, sizeof p, "P %u %lld\n", ++seq, (long long)(QpcNow() * 1e9));
      if (!SendAll(s, p)) break;
    }
    if (now - last_rx > 10) { log_("headset: nothing received for 10 s, reconnecting"); break; }
    fd_set rf;
    FD_ZERO(&rf);
    FD_SET(s, &rf);
    timeval t2{0, 50000};
    int r = select(0, &rf, nullptr, nullptr, &t2);
    if (r < 0) break;
    if (r == 0) continue;
    int n = recv(s, buf, sizeof buf, 0);
    if (n <= 0) { log_("headset: connection closed"); break; }
    double rx = QpcNow();
    last_rx = rx;
    in.append(buf, n);
    size_t pos = 0;
    for (;;) {
      if (calib_need) {  // raw calibration bytes after a "C" line
        if (in.size() - pos < calib_need) break;
        std::string err;
        std::string json = in.substr(pos, calib_need);
        pos += calib_need;
        calib_need = 0;
        if (sync_->SetCalibration(json, &err)) {
          have_calib = true;
          log_("camera calibration: " + calib_name.substr(0, calib_name.find('/')));  // the kind, not the device's id
        } else {
          log_("camera calibration " + calib_name + " unusable: " + err);
          ok = false;
          break;
        }
        continue;
      }
      size_t e = in.find('\n', pos);
      if (e == std::string::npos) break;
      std::string line = in.substr(pos, e - pos);
      pos = e + 1;
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.size() < 2 || line[1] != ' ') continue;
      switch (line[0]) {
        case 'F':
          last_frame_ = rx;
          if (have_calib) sync_->OnLine(rx, line.c_str());
          break;
        case 'Q': {
          unsigned sq;
          long long send_ns, mono_ns;
          if (sscanf(line.c_str(), "Q %u %lld %lld", &sq, &send_ns, &mono_ns) == 3) {
            sync_->OnPing(send_ns / 1e9, rx, mono_ns / 1e9);
            if (sync_->recording()) rec_(rx, "Q " + std::to_string(send_ns / 1e9) + " " + std::to_string(rx) + " " + std::to_string(mono_ns / 1e9));
          }
          break;
        }
        case 'H': {
          std::string h = line;
          auto field = [&](const char *k) {
            size_t i = h.find(std::string(" ") + k + "=");
            if (i == std::string::npos) return std::string();
            i += strlen(k) + 2;
            size_t j = h.find(' ', i);
            std::string v = h.substr(i, j == std::string::npos ? std::string::npos : j - i);
            for (auto &c : v) if (c == '_') c = ' ';
            return v;
          };
          {
            std::lock_guard<std::mutex> g(m_);
            serial_ = field("serial");
            if (!field("model").empty()) model_ = field("model");
            fw_ = field("fw");
            if (!mem_path_.empty() && last_ip_ != f.ip && h.rfind("H QuestLHSync ", 0) == 0) {
              last_ip_ = f.ip;
              if (FILE *mf = fopen(mem_path_.c_str(), "w")) {
                fprintf(mf, "%s %s\n", f.ip.c_str(), serial_.c_str());
                fclose(mf);
              }
            }
          }
          log_(model() + ": firmware " + fw() + ", module " + field("module"));
          break;
        }
        case 'C': {
          long long len = 0;
          char name[256] = "";
          if (sscanf(line.c_str(), "C %lld %255s", &len, name) >= 1 && len > 0 && len < (64 << 20)) {
            calib_need = (size_t)len;
            calib_name = name;
          }
          break;
        }
        case 'T':
          rec_(rx, line);
          break;
        case 'I':
        case 'W':
        case 'E':
          rec_(rx, line);
          log_("headset: " + line.substr(2));
          break;
      }
    }
    in.erase(0, pos);
  }
  closesocket(s);
  state_ = kSearching;
  return true;
}
