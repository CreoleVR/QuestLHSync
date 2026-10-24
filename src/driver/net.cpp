#include "net.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
typedef int SOCKET;
#define INVALID_SOCKET (-1)
#define closesocket close
#endif

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>

#include "platform.h"
#include "sync.h"

static const int kUdpPort = 47281, kTcpPort = 47280;
// a headset that sends no camera frames this long, when SteamVR's could be another one (the kind can't tell, and
// another headset answered, or a manual or remembered address is another one's), gives way to the next: one left on a
// desk, awake
static const double kQuietSwitch = 20.0;  // s
static const double kQuietAgain = 120.0;  // s, once it gave way and came round again: nothing else answered then
static const size_t kKnownMax = 8;        // headsets kept in the memory

#ifdef _WIN32
double QpcNow() {
  static LARGE_INTEGER f = [] { LARGE_INTEGER v; QueryPerformanceFrequency(&v); return v; }();
  LARGE_INTEGER c;
  QueryPerformanceCounter(&c);
  return (double)(c.QuadPart / f.QuadPart) + (double)(c.QuadPart % f.QuadPart) / (double)f.QuadPart;
}
#else
// CLOCK_MONOTONIC: the same clock as the pose times (now + poseTimeOffset) and the headset round trips
double QpcNow() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + ts.tv_nsec * 1e-9;
}
#endif

// waits until s is readable (or writable): 1 ready, 0 timed out, -1 error. poll on Linux: vrserver may hold more than
// FD_SETSIZE descriptors, which select can't take
static int WaitFd(SOCKET s, bool write, double seconds) {
#ifdef _WIN32
  fd_set f, e;
  FD_ZERO(&f); FD_SET(s, &f);
  FD_ZERO(&e); FD_SET(s, &e);
  timeval tv{(long)seconds, (long)((seconds - (long)seconds) * 1e6)};
  int r = select(0, write ? nullptr : &f, write ? &f : nullptr, write ? &e : nullptr, &tv);
  if (r > 0 && write && FD_ISSET(s, &e)) return -1;
  return r < 0 ? -1 : r > 0 ? 1 : 0;
#else
  pollfd p{s, (short)(write ? POLLOUT : POLLIN), 0};
  int ms = (int)(seconds * 1000 + 0.5);
  for (;;) {
    int r = poll(&p, 1, ms);
    if (r < 0 && errno == EINTR) continue;
    if (r < 0) return -1;
    if (r == 0) return 0;
    if (write && (p.revents & (POLLERR | POLLHUP))) return -1;
    return 1;
  }
#endif
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

void HeadsetLink::SetFamily(const std::string &family) {
  std::lock_guard<std::mutex> g(m_);
  if (family != family_) wrong_ips_.clear();
  family_ = family;
}

void HeadsetLink::SetStreamer(const std::string &driver) {
  std::lock_guard<std::mutex> g(m_);
  streamer_ = driver;
}

std::string QuestFamily(const std::string &text) {
  std::string s;
  for (char c : text)
    if (isalnum((unsigned char)c)) s += (char)tolower((unsigned char)c);
  auto has = [&](const char *k) { return s.find(k) != std::string::npos; };
  if (has("questpro") || has("seacliff")) return "Quest Pro";
  if (has("quest3") || has("eureka") || has("panther")) return "Quest 3";
  if (has("steamframe") || has("deckard")) return "Steam Frame";
  return "";
}

// a headset of this model can be SteamVR's HMD (family "" or an unnamed model: can't tell, so yes)
static bool Fits(const std::string &family, const std::string &model) {
  std::string f = QuestFamily(model);
  return family.empty() || f.empty() || f == family;
}

// the memory: a line per headset, the latest first, "<ip> <id> <model> <streamer's driver>" ("-" for unknown, "_" for
// a space; the id is HeadsetId's, the serial isn't kept). Up to 1.25 it held the last headset's address alone (and up
// to 1.3 its serial after it, now turned into its id): read alike.
static std::string MemField(const std::string &s) {
  if (s.empty()) return "-";
  std::string o = s;
  for (auto &c : o)
    if (isspace((unsigned char)c)) c = '_';
  return o;
}

static std::string MemValue(const char *s, bool spaces) {
  std::string o = strcmp(s, "-") ? s : "";
  if (spaces)
    for (auto &c : o)
      if (c == '_') c = ' ';
  return o;
}

static void WriteMemory(const std::string &path, const std::vector<HeadsetLink::Known> &known) {
  if (FILE *f = fopen(path.c_str(), "w")) {
    for (auto &k : known)
      fprintf(f, "%s %s %s %s\n", k.ip.c_str(), MemField(k.id).c_str(), MemField(k.model).c_str(),
              MemField(k.streamer).c_str());
    fclose(f);
  }
}

void HeadsetLink::SetMemory(const std::string &path) {
  std::lock_guard<std::mutex> g(m_);
  mem_path_ = path;
  known_.clear();
  bool serials = false;
  if (FILE *f = fopen(path.c_str(), "r")) {
    char line[512];
    while (fgets(line, sizeof line, f) && known_.size() < kKnownMax) {
      char ip[64] = "", id[64] = "-", model[128] = "-", streamer[64] = "-";
      if (sscanf(line, "%63s %63s %127s %63s", ip, id, model, streamer) < 1) continue;
      std::string i = MemValue(id, false);
      if (!i.empty() && (i.size() != 8 || i.find_first_not_of("0123456789abcdef") != std::string::npos)) {
        i = HeadsetId(i);  // a serial, as kept up to 1.3
        serials = true;
      }
      known_.push_back({ip, i, MemValue(model, true), MemValue(streamer, false)});
    }
    fclose(f);
  }
  if (serials) WriteMemory(path, known_);
}

static bool SameHeadset(const HeadsetLink::Known &k, const std::string &ip, const std::string &id) {
  if (!id.empty() && !k.id.empty()) return k.id == id;
  return k.ip == ip;
}

// k.streamer "" keeps the one it had: a headset counts as used with a streamer once its cameras send frames
void HeadsetLink::Remember(const Known &k) {
  std::vector<Known> v{k};
  if (k.streamer.empty())
    for (auto &o : known_)
      if (SameHeadset(o, k.ip, k.id)) { v[0].streamer = o.streamer; break; }
  for (auto &o : known_)  // the same headset, or another one that had its address before
    if (!SameHeadset(o, k.ip, k.id) && o.ip != k.ip && v.size() < kKnownMax) v.push_back(o);
  bool same = v.size() == known_.size();
  for (size_t i = 0; same && i < v.size(); i++)
    same = v[i].ip == known_[i].ip && v[i].id == known_[i].id && v[i].model == known_[i].model &&
           v[i].streamer == known_[i].streamer;
  known_.swap(v);
  if (!same && !mem_path_.empty()) WriteMemory(mem_path_, known_);
}

// which headset to try first when several could be SteamVR's (all of them its kind, or the kind unknown): the one asked
// for (settings), then those last used with this streamer, then the others remembered, the latest first, then new ones;
// a headset that sent no camera frames goes after all of them
int HeadsetLink::Rank(const std::string &ip, const std::string &id) const {
  int r = quiet_ips_.count(ip) ? 1000 : 0;
  if (!id.empty() && id == HeadsetId(preferred_)) return r;
  for (size_t i = 0; i < known_.size(); i++)
    if (SameHeadset(known_[i], ip, id))
      return r + (!streamer_.empty() && known_[i].streamer == streamer_ ? 100 : 200) + (int)i;
  return r + 300;
}

bool HeadsetLink::Others(const std::string &ip) const {
  for (auto &a : answered_)
    if (a != ip && !wrong_ips_.count(a)) return true;
  for (auto &h : hosts_)
    if (h != ip && !wrong_ips_.count(h)) return true;
  for (auto &k : known_)
    if (k.ip != ip && !wrong_ips_.count(k.ip) && Fits(family_, k.model)) return true;
  return false;
}

static void Sleep_(std::atomic<bool> &run, int ms) {
  for (int i = 0; i < ms / 50 && run; i++) qlhs::SleepMs(50);
}

void HeadsetLink::Loop() {
#ifdef _WIN32
  WSADATA wd;
  WSAStartup(MAKEWORD(2, 2), &wd);
#endif
  int quiet = 0;
  while (run_) {
    if (!wanted_) { state_ = kIdle; Sleep_(run_, 500); continue; }
    state_ = kSearching;
    std::vector<Found> found;
    Discover(found);
    // what to try, in turn until one talks, by Rank: the headsets that answered and can be SteamVR's, and straight
    // over TCP the manual hosts (a firewall eating UDP; ahead of headsets new to the memory) and the headsets remembered
    // (a network dropping broadcasts), each after one that answered at the same rank
    std::vector<std::pair<int, Found>> cand;
    std::string fam;
    {
      std::lock_guard<std::mutex> g(m_);
      fam = family_;
      auto listed = [&](const std::string &ip) {
        for (auto &c : cand)
          if (c.second.ip == ip) return true;
        return false;
      };
      answered_.clear();
      for (auto &f : found)
        if (Fits(fam, f.model)) {
          cand.push_back({2 * Rank(f.ip, HeadsetId(f.serial)), f});
          answered_.insert(f.ip);
        }
      for (auto &h : hosts_)
        if (!wrong_ips_.count(h) && !listed(h)) {
          int r = Rank(h, "");
          if (r % 1000 == 300) r -= 50;  // not remembered: still ahead of new headsets that answered
          cand.push_back({2 * r + 1, {h, "", "", kTcpPort}});
        }
      for (auto &k : known_)
        if (Fits(fam, k.model) && !wrong_ips_.count(k.ip) && !listed(k.ip))
          cand.push_back({2 * Rank(k.ip, k.id) + 1, {k.ip, "", "", kTcpPort}});
    }
    std::stable_sort(cand.begin(), cand.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    bool fitting = false;
    for (auto &f : found) fitting = fitting || Fits(fam, f.model);
    if (!fitting && !found.empty() && skipped_ != found[0].serial) {  // once per headset, not every round
      skipped_ = found[0].serial;
      log_(found[0].model + " at " + found[0].ip + " answers, but SteamVR's headset is a " + fam + ": not used");
    }
    if (found.empty() && quiet++ % 30 == 0)
      log_("no QuestLHSync headset answers on the network (is it awake, with lhsyncd installed?)");
    for (auto &c : cand)
      if (run_ && wanted_ && Session(c.second)) {
        quiet = 0;
        break;
      }
    state_ = kSearching;
    Sleep_(run_, 2000);
  }
#ifdef _WIN32
  WSACleanup();
#endif
}

// directed broadcast address of every IPv4 interface that's up
static std::vector<sockaddr_in> Broadcasts() {
  std::vector<sockaddr_in> out;
#ifdef _WIN32
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
#else
  ifaddrs *ifs = nullptr;
  if (getifaddrs(&ifs) != 0) return out;
  for (ifaddrs *a = ifs; a; a = a->ifa_next) {
    if (!a->ifa_addr || a->ifa_addr->sa_family != AF_INET || !a->ifa_netmask) continue;
    if (!(a->ifa_flags & IFF_UP) || (a->ifa_flags & IFF_LOOPBACK)) continue;
    uint32_t ip = ntohl(((sockaddr_in *)a->ifa_addr)->sin_addr.s_addr);
    uint32_t mask = ntohl(((sockaddr_in *)a->ifa_netmask)->sin_addr.s_addr);
    if (mask == 0 || mask == 0xFFFFFFFFu || mask == 0xFFFFFFFEu) continue;  // no prefix, or /31, /32
    sockaddr_in b{};
    b.sin_family = AF_INET;
    b.sin_port = htons(kUdpPort);
    b.sin_addr.s_addr = htonl(ip | ~mask);
    out.push_back(b);
  }
  freeifaddrs(ifs);
#endif
  return out;
}

bool HeadsetLink::Discover(std::vector<Found> &out) {
  SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s == INVALID_SOCKET) return false;
#ifndef _WIN32
  fcntl(s, F_SETFD, fcntl(s, F_GETFD) | FD_CLOEXEC);
#endif
  int on = 1;
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
    if (WaitFd(s, false, left) <= 0) break;
    char buf[512];
    sockaddr_in from{};
    socklen_t fl = sizeof from;
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
#ifdef _WIN32
    int n = send(s, d.data() + o, (int)(d.size() - o), 0);
#else
    ssize_t n = send(s, d.data() + o, d.size() - o, MSG_NOSIGNAL);  // a dropped headset mustn't SIGPIPE vrserver
#endif
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
  if (s == INVALID_SOCKET) { freeaddrinfo(res); return false; }
#ifdef _WIN32
  u_long nb = 1;
  ioctlsocket(s, FIONBIO, &nb);
#else
  fcntl(s, F_SETFD, fcntl(s, F_GETFD) | FD_CLOEXEC);  // not into the dashboard app the driver starts
  fcntl(s, F_SETFL, fcntl(s, F_GETFL) | O_NONBLOCK);
#endif
  connect(s, res->ai_addr, (int)res->ai_addrlen);
  freeaddrinfo(res);
  bool up = WaitFd(s, true, 3) > 0;
#ifndef _WIN32
  if (up) {  // a refused connect shows as writable here, with the error in SO_ERROR
    int err = 0;
    socklen_t el = sizeof err;
    up = getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &el) == 0 && err == 0;
  }
#endif
  if (!up) {
    std::lock_guard<std::mutex> g(m_);
    if (failed_ip_ != f.ip) log_("headset " + f.ip + ": TCP connect failed");  // once per address, not every retry
    failed_ip_ = f.ip;
    closesocket(s);
    return false;
  }
#ifdef _WIN32
  nb = 0;
  ioctlsocket(s, FIONBIO, &nb);
  BOOL on = TRUE;
#else
  fcntl(s, F_SETFL, fcntl(s, F_GETFL) & ~O_NONBLOCK);
  int on = 1;
#endif
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
  bool ok = true, wrong = false, framed = false, quiet_checked = false, remembered = false;
  double quiet_after;
  {
    std::lock_guard<std::mutex> g(m_);
    quiet_after = quiet_ips_.count(f.ip) ? kQuietAgain : kQuietSwitch;
  }
  char buf[65536];
  while (run_ && wanted_ && ok) {
    double now = QpcNow();
    if (!framed && !quiet_checked && now - start > quiet_after) {  // once: memory and hosts don't change meanwhile
      quiet_checked = true;
      bool others, said;
      {
        std::lock_guard<std::mutex> g(m_);
        others = family_.empty() && Others(f.ip);
        said = quiet_ips_.count(f.ip) > 0;
        if (others) quiet_ips_.insert(f.ip);
      }
      if (others) {
        if (!said)
          log_(model() + " at " + f.ip + ": no camera frames in " + std::to_string((int)kQuietSwitch) +
               " s, SteamVR's headset may be another: trying it");
        break;
      }
    }
    if (now >= next_ping) {  // clock round trips: 4 Hz
      next_ping = now + 0.25;
      char p[64];
      snprintf(p, sizeof p, "P %u %lld\n", ++seq, (long long)(QpcNow() * 1e9));
      if (!SendAll(s, p)) break;
    }
    if (now - last_rx > 10) { log_("headset: nothing received for 10 s, reconnecting"); break; }
    int r = WaitFd(s, false, 0.05);
    if (r < 0) break;
    if (r == 0) continue;
    int n = (int)recv(s, buf, sizeof buf, 0);
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
          if (!framed) {
            framed = true;
            std::lock_guard<std::mutex> g(m_);
            quiet_ips_.erase(f.ip);
            if (remembered) Remember({f.ip, HeadsetId(serial_), model_, streamer_});
          }
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
          std::string fam;
          bool fits;
          {
            std::lock_guard<std::mutex> g(m_);
            serial_ = field("serial");
            if (!field("model").empty()) model_ = field("model");
            fw_ = field("fw");
            fam = family_;
            fits = Fits(fam, model_);
            remembered = fits && h.rfind("H QuestLHSync ", 0) == 0;
            if (remembered) Remember({f.ip, HeadsetId(serial_), model_, ""});
          }
          log_(model() + ": firmware " + fw() + ", module " + field("module"));
          if (!fits) {  // reached straight over TCP (no discovery): the address of another headset
            if (skipped_ != serial()) log_(model() + " at " + f.ip + " isn't SteamVR's headset (a " + fam + "): not used");
            skipped_ = serial();
            {
              std::lock_guard<std::mutex> g(m_);
              wrong_ips_.insert(f.ip);
            }
            wrong = true;
            ok = false;
          } else {
            sync_->SetHeadset(serial(), model());  // its own state.json from here on
          }
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
  return !wrong;
}
