#include <winsock2.h>
#include <windows.h>
#include <setupapi.h>
#include <shlobj.h>

#include "gravity.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

#include "json.h"
#include "net.h"

extern "C" {
void __stdcall HidD_GetHidGuid(GUID *guid);
BOOLEAN __stdcall HidD_GetSerialNumberString(HANDLE device, PVOID buffer, ULONG length);
}

namespace {
constexpr double kHist = 20;              // s of poses kept per device
constexpr double kRestFor = 3;            // s still before its accelerometer is read
constexpr double kListen = 3;             // s of IMU samples a read takes (some 330 at the Index's rate)
constexpr double kRestPos = 0.003;        // m and deg: still
constexpr double kRestDeg = 0.5;
constexpr double kSameDeg = 10;           // a rest this close to one read this session isn't read again
constexpr size_t kMaxRests = 40;          // per device, the newest
constexpr size_t kMaxSessions = 20;       // the newest sessions' rests
constexpr double kSigMeas = 0.1 / kDeg;   // rad: a rest's up beyond its noise (the pose, the config's IMU axes)
constexpr double kSigOff = 0.05;          // m/s^2: the accelerometer's offset left after its factory calibration
constexpr double kSigPrior = 5 / kDeg;    // rad: a session's tilt, before its rests
constexpr double kApplySd = 0.25;         // deg: the tilt is applied once known this well
constexpr double kMaxTilt = 3;            // deg: more is something else
constexpr double kSlew = 0.1 / kDeg;      // rad a step (1 s): a change eases in
constexpr double kG = 9.80665;

std::string Fmt(const char *fmt, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  return buf;
}

std::string Now() {
  time_t t = time(nullptr);
  struct tm tm;
  localtime_s(&tm, &t);
  char b[32];
  strftime(b, sizeof b, "%Y-%m-%d %H:%M:%S", &tm);
  return b;
}

bool ReadText(const std::string &path, std::string &out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::stringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

V3 Unit(V3 v) { return v * (1 / norm(v)); }

// a frame in a lighthouse device config (its axes in the device's tracking frame): columns X Y Z
bool Axes(const JVal *j, M3 &R) {
  if (!j) return false;
  const JVal *px = j->get("plus_x"), *pz = j->get("plus_z");
  if (!px || !pz) return false;
  auto x = px->nums(), z = pz->nums();
  if (x.size() != 3 || z.size() != 3) return false;
  V3 X = Unit({x[0], x[1], x[2]}), Z{z[0], z[1], z[2]};
  V3 Y = Unit(cross(Z, X));
  Z = cross(X, Y);
  for (int r = 0; r < 3; r++) { R.m[r][0] = X[r]; R.m[r][1] = Y[r]; R.m[r][2] = Z[r]; }
  return true;
}

bool Vec(const JVal *j, V3 &v) {
  if (!j) return false;
  auto n = j->nums();
  if (n.size() != 3) return false;
  v = {n[0], n[1], n[2]};
  return true;
}

// the mean of rotations close together
M3 MeanRot(const std::vector<M3> &Rs) {
  Quat q0 = ToQuat(Rs.front());
  double w = 0, x = 0, y = 0, z = 0;
  for (const M3 &R : Rs) {
    Quat q = ToQuat(R);
    double s = q.w * q0.w + q.x * q0.x + q.y * q0.y + q.z * q0.z < 0 ? -1 : 1;
    w += s * q.w; x += s * q.x; y += s * q.y; z += s * q.z;
  }
  return ToM3({w, x, y, z});
}

// n x n (row-major) inverse by Gauss-Jordan; false if singular
bool Invert(std::vector<double> &A, int n) {
  std::vector<double> I(n * n, 0);
  for (int i = 0; i < n; i++) I[i * n + i] = 1;
  for (int c = 0; c < n; c++) {
    int p = c;
    for (int r = c + 1; r < n; r++)
      if (std::fabs(A[r * n + c]) > std::fabs(A[p * n + c])) p = r;
    if (std::fabs(A[p * n + c]) < 1e-300) return false;
    for (int j = 0; j < n; j++) { std::swap(A[p * n + j], A[c * n + j]); std::swap(I[p * n + j], I[c * n + j]); }
    double d = A[c * n + c];
    for (int j = 0; j < n; j++) { A[c * n + j] /= d; I[c * n + j] /= d; }
    for (int r = 0; r < n; r++) {
      if (r == c) continue;
      double f = A[r * n + c];
      if (f == 0) continue;
      for (int j = 0; j < n; j++) { A[r * n + j] -= f * A[c * n + j]; I[r * n + j] -= f * I[c * n + j]; }
    }
  }
  A.swap(I);
  return true;
}

// The receiver's reports: 0x23 one Watchman packet, 0x24 two (the second 29 bytes on). A packet: time MSB, size, time
// LSB, then size - 1 bytes: Watchman v2 flags, and with 0x80 the IMU sample first: a time byte, accel xyz, gyro xyz
// (int16 LE). libsurvive's driver_vive.c (survive_handle_watchman, handle_watchman_v2, read_imu_data).
void Decode(const unsigned char *r, size_t n, std::vector<V3> &acc) {
  if (n < 2 || (r[0] != 0x23 && r[0] != 0x24)) return;
  for (size_t off : {size_t(1), size_t(30)}) {
    if (off == 30 && r[0] != 0x24) break;
    if (off + 3 > n) break;
    const unsigned char *p = r + off;
    size_t size = p[1];
    if (size < 15 || off + 3 + size - 1 > n) continue;
    const unsigned char *pay = p + 3;
    if (!(pay[0] & 0x80)) continue;
    auto s16 = [&](int i) { return (double)(int16_t)(pay[2 + 2 * i] | (pay[3 + 2 * i] << 8)); };
    acc.push_back({s16(0), s16(1), s16(2)});
  }
}
}  // namespace

Gravity::Gravity(std::string dir, LogFn log) : dir_(std::move(dir)), session_(Now()), log_(std::move(log)) {
  // SteamVR's config folder: each lighthouse device's config, with its IMU calibration
  PWSTR p = nullptr;
  std::filesystem::path paths;
  if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, nullptr, &p)))
    paths = std::filesystem::path(p) / "openvr" / "openvrpaths.vrpath";
  CoTaskMemFree(p);
  std::string text;
  JVal root;
  if (ReadText(paths.string(), text) && JParse(text, root))
    if (const JVal *a = root.get("config"))
      if (a->t == JVal::Arr && !a->a.empty() && a->a[0].t == JVal::Str) config_dir_ = a->a[0].s;
  if (config_dir_.empty()) log_("gravity: no SteamVR config folder: levelling by gravity off");
  Load();
  worker_ = std::thread(&Gravity::Worker, this);
}

void Gravity::Stop() {
  run_ = false;
  if (worker_.joinable()) worker_.join();
}

void Gravity::SetDevice(int id, const std::string &serial, const std::string &receiver) {
  std::lock_guard<std::mutex> g(m_);
  Device &d = dev_[id];
  if (d.serial != serial) d = Device();
  d.serial = serial;
  d.receiver = receiver;
}

void Gravity::OnPose(int id, double t, V3 p, const M3 &R) {
  if (!enabled_) return;
  std::lock_guard<std::mutex> g(m_);
  auto it = dev_.find(id);
  if (it == dev_.end()) return;
  Device &d = it->second;
  if (t - d.last < 0.1) return;  // 10 Hz
  d.last = t;
  d.hist.push_back({t, p, R});
  while (!d.hist.empty() && d.hist.front().t < t - kHist) d.hist.pop_front();
  if (d.rest_since < 0 || norm(p - d.rest_p) > kRestPos || RotDeg(T(d.rest_R) * R) > kRestDeg) {
    d.rest_since = t;  // moved: a new rest may begin here
    d.rest_p = p;
    d.rest_R = R;
    d.read = false;
  }
}

// one rest from before t0 until t1, with its poses still coming
bool Gravity::Still(const Device &d, double from, double to) {
  return d.rest_since >= 0 && d.rest_since <= from && !d.hist.empty() && d.hist.back().t >= to - 0.5;
}

bool Gravity::LoadConfig(Device &d) {
  std::string lower = d.serial, text;
  for (char &c : lower) c = (char)tolower((unsigned char)c);
  JVal root;
  M3 Rti, Rth;
  const JVal *imu = nullptr;
  if (config_dir_.empty() ||
      !ReadText((std::filesystem::path(config_dir_) / "lighthouse" / lower / "config.json").string(), text) ||
      !JParse(text, root) || !(imu = root.get("imu")) || !Axes(imu, Rti) || !Axes(root.get("head"), Rth) ||
      !Vec(imu->get("acc_bias"), d.bias) || !Vec(imu->get("acc_scale"), d.scale)) {
    d.cfg = -1;
    log_(Fmt("gravity: no IMU calibration for %s in SteamVR's config folder: not used", d.serial.c_str()));
    return false;
  }
  d.head_imu = T(Rth) * Rti;  // head <- tracking <- IMU
  d.cfg = 1;
  return true;
}

// the receiver's HID interface: Valve's (vid_28de) whose serial is the receiver's
bool Gravity::FindPath(const std::string &receiver, std::wstring &path) {
  auto c = paths_.find(receiver);
  if (c != paths_.end()) { path = c->second; return true; }
  GUID guid;
  HidD_GetHidGuid(&guid);
  HDEVINFO set = SetupDiGetClassDevsW(&guid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
  if (set == INVALID_HANDLE_VALUE) return false;
  std::wstring want(receiver.begin(), receiver.end());
  SP_DEVICE_INTERFACE_DATA ifd{sizeof ifd};
  for (DWORD i = 0; SetupDiEnumDeviceInterfaces(set, nullptr, &guid, i, &ifd); i++) {
    DWORD need = 0;
    SetupDiGetDeviceInterfaceDetailW(set, &ifd, nullptr, 0, &need, nullptr);
    if (!need) continue;
    std::vector<char> buf(need);
    auto *det = (SP_DEVICE_INTERFACE_DETAIL_DATA_W *)buf.data();
    det->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
    if (!SetupDiGetDeviceInterfaceDetailW(set, &ifd, det, need, nullptr, nullptr)) continue;
    std::wstring p = det->DevicePath, low = p;
    for (auto &ch : low) ch = (wchar_t)towlower(ch);
    if (low.find(L"vid_28de") == std::wstring::npos) continue;
    HANDLE h = CreateFileW(p.c_str(), 0, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) continue;
    wchar_t sn[128] = L"";
    bool match = HidD_GetSerialNumberString(h, sn, sizeof sn) && want == sn;
    CloseHandle(h);
    if (match) {
      paths_[receiver] = path = p;
      SetupDiDestroyDeviceInfoList(set);
      return true;
    }
  }
  SetupDiDestroyDeviceInfoList(set);
  return false;
}

// secs of the receiver's input reports, read only (another reader of the same interface: nothing is sent)
bool Gravity::Listen(const std::string &receiver, double secs, std::vector<V3> &acc) {
  std::wstring path;
  if (!FindPath(receiver, path)) return false;
  HANDLE h = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
                         FILE_FLAG_OVERLAPPED, nullptr);
  if (h == INVALID_HANDLE_VALUE) { paths_.erase(receiver); return false; }
  OVERLAPPED ov{};
  ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
  unsigned char buf[512];
  double end = QpcNow() + secs;
  while (run_ && QpcNow() < end) {
    ResetEvent(ov.hEvent);
    DWORD n = 0;
    if (!ReadFile(h, buf, sizeof buf, &n, &ov)) {
      if (GetLastError() != ERROR_IO_PENDING) break;
      if (WaitForSingleObject(ov.hEvent, 200) != WAIT_OBJECT_0) {
        CancelIoEx(h, &ov);
        GetOverlappedResult(h, &ov, &n, TRUE);
        continue;
      }
      if (!GetOverlappedResult(h, &ov, &n, FALSE)) break;
    }
    Decode(buf, n, acc);
  }
  CancelIoEx(h, nullptr);
  CloseHandle(ov.hEvent);
  CloseHandle(h);
  return true;
}

void Gravity::Worker() {
  while (run_) {
    Sleep(500);
    if (!enabled_ || config_dir_.empty()) continue;
    double now = QpcNow();
    int id = -1;
    std::string recv, serial;
    {
      std::lock_guard<std::mutex> g(m_);
      for (auto &kv : dev_) {
        Device &d = kv.second;
        if (d.receiver.empty() || d.read || d.rest_since < 0 || now - d.rest_since < kRestFor || d.hist.empty())
          continue;
        if (d.cfg == 0) LoadConfig(d);
        if (d.cfg != 1) { d.read = true; continue; }
        M3 R = d.hist.back().R * d.head_imu;
        bool seen = false;
        for (const M3 &t : d.taken) seen = seen || RotDeg(T(t) * R) < kSameDeg;
        if (seen) { d.read = true; continue; }
        id = kv.first;
        recv = d.receiver;
        serial = d.serial;
        break;
      }
    }
    if (id < 0) continue;
    double t0 = QpcNow();
    std::vector<V3> raw;
    bool ok = Listen(recv, kListen, raw);
    double t1 = QpcNow();
    std::lock_guard<std::mutex> g(m_);
    auto it = dev_.find(id);
    if (it == dev_.end() || it->second.serial != serial) continue;
    Device &d = it->second;
    d.read = true;
    if (!ok || raw.size() < 100) {
      if (run_ && !fails_[recv]++)
        log_(Fmt("gravity: no IMU samples from %s's receiver %s (%s)", serial.c_str(), recv.c_str(),
                 ok ? "not a format it knows" : "not found"));
      continue;
    }
    if (!Still(d, t0, t1)) continue;  // moved while it was read
    V3 m;
    for (const V3 &a : raw) m += a;
    m = m * (1.0 / raw.size());
    double worst = 0;
    for (int k = 0; k < 3; k++) {
      double s = 0;
      for (const V3 &a : raw) s += (a[k] - m[k]) * (a[k] - m[k]);
      worst = std::max(worst, std::sqrt(s / raw.size()));
    }
    // counts per g: the range the device runs at (+-8 g on an Index controller and a 3.0 tracker: 4096)
    double cpg = 0;
    for (double c : {2048.0, 4096.0, 8192.0, 16384.0})
      if (std::fabs(norm(m) / c - 1) < 0.1) cpg = c;
    if (!cpg || worst > 0.012 * cpg) {
      log_(Fmt("gravity: %s's IMU read wasn't a clean rest (%zu samples, |a| %.0f counts, sd %.0f): not used",
               serial.c_str(), raw.size(), norm(m), worst));
      continue;
    }
    V3 a = m * (kG / cpg) - d.bias;
    for (int k = 0; k < 3; k++) a[k] *= d.scale[k];
    std::vector<M3> Rs;
    for (const Sample &s : d.hist)
      if (s.t >= t0 && s.t <= t1) Rs.push_back(s.R);
    if (Rs.empty()) continue;
    M3 R = MeanRot(Rs) * d.head_imu;  // raw <- IMU
    d.taken.push_back(R);
    pending_.push_back({serial, a, R, (int)raw.size()});
  }
}

void Gravity::Load() {
  std::string text;
  JVal root;
  if (!ReadText(dir_ + "\\gravity.json", text) || !JParse(text, root)) return;
  const JVal *k = root.get("key"), *rs = root.get("rests");
  if (!k || k->t != JVal::Str || !rs || rs->t != JVal::Arr) return;
  key_ = k->s;
  for (const JVal &r : rs->a) {
    const JVal *dv = r.get("dev"), *se = r.get("session"), *wh = r.get("when"), *q = r.get("q");
    V3 a;
    if (!dv || dv->t != JVal::Str || !se || se->t != JVal::Str || !Vec(r.get("a"), a) || !q) continue;
    auto qv = q->nums();
    if (qv.size() != 4) continue;
    rests_.push_back({dv->s, se->s, wh && wh->t == JVal::Str ? wh->s : "", a, ToM3({qv[0], qv[1], qv[2], qv[3]})});
  }
}

void Gravity::Save() const {
  std::string s = "{\n \"note\": \"QuestLHSync's gravity levelling: lighthouse devices at rest, per SteamVR session: "
                  "the accelerometer (calibrated, m/s^2, its axes) and the turn from its axes into the reference "
                  "frame as the anchor places it (q). Delete to start over.\",\n \"key\": \"" + key_ +
                  "\",\n \"rests\": [";
  for (size_t i = 0; i < rests_.size(); i++) {
    const Rest &r = rests_[i];
    Quat q = ToQuat(r.Q);
    s += Fmt("%s\n  {\"dev\": \"%s\", \"session\": \"%s\", \"when\": \"%s\", \"a\": [%.5f, %.5f, %.5f], "
             "\"q\": [%.8f, %.8f, %.8f, %.8f]}",
             i ? "," : "", r.dev.c_str(), r.session.c_str(), r.when.c_str(), r.a.x, r.a.y, r.a.z, q.w, q.x, q.y, q.z);
  }
  s += "\n ]\n}\n";
  std::string path = dir_ + "\\gravity.json", tmp = path + ".tmp";
  FILE *f = fopen(tmp.c_str(), "wb");
  if (!f) return;
  fwrite(s.data(), 1, s.size(), f);
  fclose(f);
  MoveFileExA(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING);
}

// Each rest's up, less its device's offset o and tilted by its session's Tilt(x, z), is the headset's up: least
// squares over every rest kept, the offsets held near 0 (kSigOff). This session's tilt, and how well it is known.
void Gravity::Fit() {
  const V3 up{0, 1, 0};
  std::map<std::string, int> di, si;
  for (const Rest &r : rests_) {
    di.emplace(r.dev, (int)di.size());
    si.emplace(r.session, (int)si.size());
  }
  auto cur = si.find(session_);
  if (cur == si.end()) return;
  const int ns = (int)si.size(), n = 2 * ns + 3 * (int)di.size();
  auto res = [&](const std::vector<double> &y, std::vector<double> &rv) {
    rv.clear();
    for (const Rest &r : rests_) {
      int s = 2 * si[r.session], k = 2 * ns + 3 * di[r.dev];
      V3 u = Tilt(y[s], y[s + 1]) * Unit(r.Q * (r.a - V3{y[k], y[k + 1], y[k + 2]})) - up;
      for (int c = 0; c < 3; c++) rv.push_back(u[c] / kSigMeas);
    }
    for (int j = 0; j < n; j++) rv.push_back(y[j] / (j < 2 * ns ? kSigPrior : kSigOff));
  };
  std::vector<double> y(n, 0), r0, r1, J, A;
  for (int it = 0; it < 15; it++) {
    res(y, r0);
    const size_t m = r0.size();
    J.assign(m * n, 0);
    for (int j = 0; j < n; j++) {
      std::vector<double> yp = y;
      double e = j < 2 * ns ? 1e-7 : 1e-6;
      yp[j] += e;
      res(yp, r1);
      for (size_t i = 0; i < m; i++) J[i * n + j] = (r1[i] - r0[i]) / e;
    }
    A.assign(n * n, 0);
    std::vector<double> g(n, 0);
    for (size_t i = 0; i < m; i++)
      for (int a = 0; a < n; a++) {
        if (J[i * n + a] == 0) continue;
        g[a] += J[i * n + a] * r0[i];
        for (int b = 0; b < n; b++) A[a * n + b] += J[i * n + a] * J[i * n + b];
      }
    if (!Invert(A, n)) return;
    double step = 0;
    for (int a = 0; a < n; a++) {
      double da = 0;
      for (int b = 0; b < n; b++) da -= A[a * n + b] * g[b];
      y[a] += da;
      step = std::max(step, std::fabs(da));
    }
    if (step < 1e-10) break;
  }
  int c = 2 * cur->second;
  tx_ = y[c];
  tz_ = y[c + 1];
  double sd = std::sqrt(std::max(A[c * n + c], A[(c + 1) * n + c + 1])) * kDeg, deg = std::hypot(tx_, tz_) * kDeg;
  size_t mine = 0;
  std::set<std::string> devs;
  for (const Rest &r : rests_)
    if (r.session == session_) { mine++; devs.insert(r.dev); }
  good_ = sd <= kApplySd && deg <= kMaxTilt;
  log_(Fmt("gravity: %zu rest%s of %zu device%s this session (%zu kept in all): SteamVR's room is %.2f deg off "
           "level (within %.2f); %s", mine, mine > 1 ? "s" : "", devs.size(), devs.size() > 1 ? "s" : "",
           rests_.size(), deg, sd,
           good_ ? "levelling by it"
                 : deg > kMaxTilt ? "too far to be a level: not applied"
                                  : "not applied yet: rests in other orientations tell the accelerometers' offsets"));
}

bool Gravity::Step(const Ref &ref, bool &on, M3 &tilt) {
  std::vector<Raw> got;
  {
    std::lock_guard<std::mutex> g(m_);
    if (ref.ok) got.swap(pending_);  // else reads wait for the frame
  }
  bool want = enabled_ && ref.ok;
  if (want && key_ != ref.key) {
    if (!rests_.empty()) log_(Fmt("gravity: another reference frame: %zu rests dropped", rests_.size()));
    rests_.clear();
    key_ = ref.key;
    fitted_ = good_ = false;
    Save();
  }
  for (const Raw &r : got) {
    Rest e{r.dev, session_, Now(), r.a, ref.C * r.R};
    V3 u = Unit(e.Q * e.a);
    log_(Fmt("gravity: %s at rest, its accelerometer read (%d samples): up %.2f deg off SteamVR's", r.dev.c_str(), r.n,
             std::acos(std::min(1.0, u.y)) * kDeg));
    rests_.push_back(e);
    size_t of = 0;
    for (const Rest &x : rests_) of += x.dev == r.dev;
    if (of > kMaxRests)
      rests_.erase(std::find_if(rests_.begin(), rests_.end(), [&](const Rest &x) { return x.dev == r.dev; }));
    std::set<std::string> ss;
    for (auto it = rests_.rbegin(); it != rests_.rend(); ++it) ss.insert(it->session);
    while (ss.size() > kMaxSessions) {  // the oldest session goes
      std::string oldest = *ss.begin();
      for (const Rest &x : rests_) oldest = std::min(oldest, x.session);
      rests_.erase(std::remove_if(rests_.begin(), rests_.end(), [&](const Rest &x) { return x.session == oldest; }),
                   rests_.end());
      ss.erase(oldest);
    }
    fitted_ = false;
  }
  if (!got.empty()) Save();
  if (want && !fitted_) {
    Fit();
    fitted_ = true;
  }
  bool now_on = want && good_;
  if (!now_on) {
    if (!on_) return false;
    on_ = false;
    on = false;
    tilt = M3();
    log_("gravity: levelling off");
    return true;
  }
  if (!on_) applied_ = ToQuat(ref.turn);  // from the turn the frame has now (the other station's): nothing jumps
  Quat target = ToQuat(Tilt(tx_, tz_));
  double d = QuatDeg(applied_, target) / kDeg;
  if (on_ && d < 1e-7) return false;
  applied_ = d > kSlew ? Slerp(applied_, target, kSlew / d) : target;  // eases in
  on_ = on = true;
  tilt = ToM3(applied_);
  return true;
}
