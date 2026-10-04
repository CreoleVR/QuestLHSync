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
BOOLEAN __stdcall HidD_SetNumInputBuffers(HANDLE device, ULONG n);
}

namespace {
constexpr double kKeepS = 20;                      // s of IMU samples and poses kept
constexpr double kPoseStep = 0.002;               // s: poses kept at most this often
constexpr double kWin = 1.0;                      // s: a window
constexpr double kMaxImuGap = 0.03;               // s: a gap in the IMU samples or the poses breaks a window
constexpr double kMaxPoseGap = 0.05;
constexpr double kVelHalf = 0.03;                 // s: the IMU's velocity from the poses this close
constexpr double kLagSpan = 15, kLagEvery = 30;   // s: the IMU clock against the poses', over the last kLagSpan
constexpr double kMaxLag = 0.06, kLagStep = 0.0005, kMinCorr = 0.9, kMinTurn = 0.3;  // s, s, -, rad/s
constexpr double kStill = 0.02;                   // rad/s: a window this still needs no clock (its turn holds)
constexpr double kBin = 2.0;                      // s: the device clock's bins
constexpr size_t kBins = 30;
constexpr double kFitEvery = 10;                  // s
constexpr double kNoise0 = 2.5e-4, kNoiseSpin = 0.022 * 0.022;  // a window's noise per axis, (m/s)^2: + per (rad/s)^2
constexpr double kSigOffNew = 0.15;               // m/s^2: an offset not known yet (factories leave up to 0.2)
constexpr double kSigOffKept = 0.05;              // m/s^2: a kept offset, at best, in the next session (cold)
// how fast offsets wander, (m/s^2)^2 per s: while a device warms up (its first kWarm s) the accelerometer's offset
// moves some 0.05 m/s^2 over minutes (0.3 deg; real recordings), much less after
constexpr double kWarm = 600, kQWarm = 7e-6, kQ = 4e-7;
constexpr double kQG = 3e-7;                      // (m/s^2)^2 per s: gravity in the frame moves a little
// SteamVR's orientation of each device is off by a few tenths of a degree, and moves as SteamVR recalibrates the
// device's IMU (often, in its first minutes): a sideways error on that device's R a, wandering about 0. Gravity
// is what the devices agree on; one device's drift stays in its own (recorded sessions: their windows' likelihood)
constexpr double kSigTilt = 0.05, kTauTilt = 300;  // m/s^2 (0.3 deg), s
constexpr double kSigGx = 0.5;                    // m/s^2: gravity's sideways part before any data (3 deg)
constexpr double kKeepSd = 0.05;                  // m/s^2: offsets known this well are kept
constexpr double kApplySd = 0.25;                 // deg: the tilt is applied once known this well
constexpr double kMaxTilt = 3;                    // deg: more is something else
constexpr double kMaxGErr = 0.02;                 // |G| within 2% of g
constexpr double kSigG = 0.05;                    // m/s^2: |G| held near g this loosely (rests alone can't tell it
                                                  // from the offsets along it)
constexpr double kSlew = 0.1 / kDeg;              // rad a step (1 s): a change eases in
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

// a frame in a lighthouse device config (its axes in the device's tracking frame): columns X Y Z, and its place
bool Axes(const JVal *j, M3 &R, V3 &p) {
  if (!j) return false;
  const JVal *px = j->get("plus_x"), *pz = j->get("plus_z"), *pp = j->get("position");
  if (!px || !pz) return false;
  auto x = px->nums(), z = pz->nums();
  if (x.size() != 3 || z.size() != 3) return false;
  V3 X = Unit({x[0], x[1], x[2]}), Z{z[0], z[1], z[2]};
  V3 Y = Unit(cross(Z, X));
  Z = cross(X, Y);
  for (int r = 0; r < 3; r++) { R.m[r][0] = X[r]; R.m[r][1] = Y[r]; R.m[r][2] = Z[r]; }
  p = {};
  if (pp) {
    auto v = pp->nums();
    if (v.size() == 3) p = {v[0], v[1], v[2]};
  }
  return true;
}

bool Vec(const JVal *j, V3 &v) {
  if (!j) return false;
  auto n = j->nums();
  if (n.size() != 3) return false;
  v = {n[0], n[1], n[2]};
  return true;
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

// the tilt (Tilt(x, z)) that turns direction u onto +y
void TiltOf(V3 u, double &tx, double &tz) {
  u = Unit(u);
  double th = std::acos(std::max(-1.0, std::min(1.0, u.y))), h = std::hypot(u.x, u.z);
  tx = tz = 0;
  if (h > 1e-15) { tx = -th * u.z / h; tz = th * u.x / h; }
}

// linear interpolation in a sorted series, the index carried along (t rising)
double Interp(const std::vector<double> &t, const std::vector<double> &v, double x, size_t &i) {
  while (i + 1 < t.size() && t[i + 1] <= x) i++;
  if (i + 1 >= t.size()) return v.back();
  if (x <= t[i]) return v[i];
  return v[i] + (v[i + 1] - v[i]) * (x - t[i]) / (t[i + 1] - t[i]);
}
}  // namespace

Gravity::Gravity(std::string dir, LogFn log, bool live) : dir_(std::move(dir)), log_(std::move(log)), live_(live) {
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
  if (live_) worker_ = std::thread(&Gravity::Worker, this);
}

void Gravity::Stop() {
  run_ = false;
  if (worker_.joinable()) worker_.join();
  for (auto &kv : readers_) {
    if (kv.second->th.joinable()) kv.second->th.join();
  }
  readers_.clear();
  if (dirty_) Save();
}

void Gravity::SetDevice(int id, const std::string &serial, const std::string &receiver) {
  std::lock_guard<std::mutex> g(m_);
  dev_in_[id] = {serial, receiver};
}

void Gravity::OnPose(int id, double t, V3 p, const Quat &q) {
  if (!enabled_ || config_dir_.empty()) return;
  std::lock_guard<std::mutex> g(m_);
  if (!dev_in_.count(id)) return;
  double &last = pose_last_[id];
  if (t - last < kPoseStep) return;
  last = t;
  pose_in_[id].push_back({t, p, q});
}

void Gravity::OnImu(const std::string &receiver, double arrival, uint32_t tick, const int16_t a[3],
                    const int16_t g[3]) {
  ImuS s{arrival, (int64_t)tick, {(float)a[0], (float)a[1], (float)a[2]}, {(float)g[0], (float)g[1], (float)g[2]}};
  std::lock_guard<std::mutex> l(m_);
  imu_in_[receiver].push_back(s);
}

bool Gravity::LoadConfig(Dev &d) {
  std::string lower = d.serial, text;
  for (char &c : lower) c = (char)tolower((unsigned char)c);
  JVal root;
  M3 Rti, Rth;
  V3 pti, pth;
  const JVal *imu = nullptr;
  if (config_dir_.empty() ||
      !ReadText((std::filesystem::path(config_dir_) / "lighthouse" / lower / "config.json").string(), text) ||
      !JParse(text, root) || !(imu = root.get("imu")) || !Axes(imu, Rti, pti) || !Axes(root.get("head"), Rth, pth) ||
      !Vec(imu->get("acc_bias"), d.bias) || !Vec(imu->get("acc_scale"), d.scale)) {
    d.cfg = -1;
    log_(Fmt("gravity: no IMU calibration for %s in SteamVR's config folder: not used", d.serial.c_str()));
    return false;
  }
  d.head_imu = T(Rth) * Rti;  // head <- tracking <- IMU
  d.r_head = T(Rth) * (pti - pth);
  d.cfg = 1;
  return true;
}

// the receiver's HID interface: Valve's (vid_28de) whose serial is the receiver's
bool Gravity::FindPath(const std::string &receiver, std::wstring &path) {
  GUID guid;
  HidD_GetHidGuid(&guid);
  HDEVINFO set = SetupDiGetClassDevsW(&guid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
  if (set == INVALID_HANDLE_VALUE) return false;
  std::wstring want(receiver.begin(), receiver.end());
  SP_DEVICE_INTERFACE_DATA ifd{sizeof ifd};
  bool found = false;
  for (DWORD i = 0; !found && SetupDiEnumDeviceInterfaces(set, nullptr, &guid, i, &ifd); i++) {
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
    if (HidD_GetSerialNumberString(h, sn, sizeof sn) && want == sn) { path = p; found = true; }
    CloseHandle(h);
  }
  SetupDiDestroyDeviceInfoList(set);
  return found;
}

// A receiver's input reports, read only (another reader of the same interface, with its own queue: nothing is sent).
// Reports: 0x23 one Watchman packet, 0x24 two (the second 29 bytes on). A packet: time MSB, size, time LSB, then
// size - 1 bytes: Watchman v2 flags, and with 0x80 the IMU sample first: a time byte, accel xyz, gyro xyz (int16 LE).
// The sample's time: the packet's two bytes and its own, the top 24 bits of the device's 48 MHz clock. libsurvive's
// driver_vive.c (survive_handle_watchman, handle_watchman_v2, read_imu_data).
void Gravity::ReadLoop(std::string receiver, Reader *rd) {
  std::wstring path;
  HANDLE h = INVALID_HANDLE_VALUE;
  if (FindPath(receiver, path))
    h = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING,
                    FILE_FLAG_OVERLAPPED, nullptr);
  if (h == INVALID_HANDLE_VALUE) { rd->done = true; return; }
  HidD_SetNumInputBuffers(h, 512);
  OVERLAPPED ov{};
  ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
  unsigned char r[512];
  while (run_ && enabled_ && !rd->stop) {
    ResetEvent(ov.hEvent);
    DWORD n = 0;
    if (!ReadFile(h, r, sizeof r, &n, &ov)) {
      if (GetLastError() != ERROR_IO_PENDING) break;
      if (WaitForSingleObject(ov.hEvent, 200) != WAIT_OBJECT_0) {
        CancelIoEx(h, &ov);
        GetOverlappedResult(h, &ov, &n, TRUE);
        continue;
      }
      if (!GetOverlappedResult(h, &ov, &n, FALSE)) break;
    }
    double t = QpcNow();
    if (n < 2 || (r[0] != 0x23 && r[0] != 0x24)) continue;
    for (size_t off : {size_t(1), size_t(30)}) {
      if (off == 30 && r[0] != 0x24) break;
      if (off + 3 > n) break;
      const unsigned char *p = r + off;
      size_t size = p[1];
      if (size < 15 || off + 3 + size - 1 > n) continue;
      const unsigned char *pay = p + 3;
      if (!(pay[0] & 0x80)) continue;
      int16_t v[6];
      for (int i = 0; i < 6; i++) v[i] = (int16_t)(pay[2 + 2 * i] | (pay[3 + 2 * i] << 8));
      uint32_t tick = (uint32_t)p[0] << 24 | (uint32_t)p[2] << 16 | (uint32_t)pay[1] << 8;
      OnImu(receiver, t, tick, v, v + 3);
      rd->n++;
    }
  }
  CancelIoEx(h, nullptr);
  CloseHandle(ov.hEvent);
  CloseHandle(h);
  rd->done = true;
}

void Gravity::Worker() {
  while (run_) {
    Sleep(250);
    if (config_dir_.empty()) continue;
    Process(QpcNow());
  }
}

// a sample in order: its time unwrapped, and the clock's bins (arrival - device time, least per bin: the radio and
// USB only ever add delay)
void Gravity::Ingest(Rx &r, const ImuS &s0) {
  ImuS s = s0;
  uint32_t u = (uint32_t)s.tick;
  if (r.last < 0) s.tick = u;
  else {
    int64_t d = (int32_t)(u - (uint32_t)r.last);
    if (d <= 0 && d > -480000) return;  // repeated or out of order
    if (d < 0 || d > 480000000) {        // the clock jumped (reconnected), or 10 s went by unseen: start over
      int ep = r.epoch + 1;
      r = Rx();
      r.epoch = ep;
      s.tick = u;
    } else {
      s.tick = r.last + d;
    }
  }
  r.last = s.tick;
  r.s.push_back(s);
  double dv = s.tick / 48e6, off = s.arr - dv;
  if (r.bins.empty() || std::floor(r.bins.back().dev / kBin) != std::floor(dv / kBin)) r.bins.push_back({dv, off, 1});
  else {
    Bin &b = r.bins.back();
    b.n++;
    if (off < b.off) { b.off = off; b.dev = dv; }
  }
  while (r.bins.size() > kBins) r.bins.pop_front();
}

// device time -> QPC: a line through the bins' least delays
void Gravity::MapClock(Rx &r) {
  std::vector<const Bin *> b;
  for (const Bin &x : r.bins)
    if (x.n >= 20) b.push_back(&x);
  r.mapped = b.size() >= 2;
  if (!r.mapped) return;
  r.ref = b.front()->dev;
  if (b.size() < 3) {
    r.a = std::min(b[0]->off, b[1]->off);
    r.b = 0;
  } else {
    double sx = 0, sy = 0, sxx = 0, sxy = 0, n = (double)b.size();
    for (const Bin *x : b) {
      double X = x->dev - r.ref;
      sx += X; sy += x->off; sxx += X * X; sxy += X * x->off;
    }
    double den = n * sxx - sx * sx;
    r.b = den > 0 ? (n * sxy - sx * sy) / den : 0;
    r.a = (sy - r.b * sx) / n;
  }
  if (r.s.size() > 100) r.rate = (r.s.size() - 1) / ((r.s.back().tick - r.s.front().tick) / 48e6);
}

bool Gravity::PoseAt(const Dev &d, double t, Quat &q, V3 &p) const {
  auto it = std::upper_bound(d.poses.begin(), d.poses.end(), t, [](double x, const PoseS &s) { return x < s.t; });
  if (it == d.poses.begin() || it == d.poses.end()) return false;
  const PoseS &b = *it, &a = *(it - 1);
  if (b.t - a.t > kMaxPoseGap) return false;
  double s = (t - a.t) / (b.t - a.t);
  q = Slerp(a.q, b.q, s);
  p = a.p + (b.p - a.p) * s;
  return true;
}

// the IMU's velocity at t: a quadratic through its place (from the poses) within kVelHalf
bool Gravity::VelAt(const Dev &d, double t, V3 &v) const {
  auto lo = std::lower_bound(d.poses.begin(), d.poses.end(), t - kVelHalf,
                             [](const PoseS &s, double x) { return s.t < x; });
  double S[9] = {0}, B[3][3] = {{0}};
  int n = 0;
  double umin = 1, umax = -1;
  for (auto it = lo; it != d.poses.end() && it->t <= t + kVelHalf; ++it) {
    double u = it->t - t, f[3] = {1, u, u * u};
    V3 p = it->p + ToM3(it->q) * d.r_head;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) S[i * 3 + j] += f[i] * f[j];
      for (int k = 0; k < 3; k++) B[k][i] += f[i] * p[k];
    }
    umin = std::min(umin, u);
    umax = std::max(umax, u);
    n++;
  }
  if (n < 5 || umin > -0.4 * kVelHalf || umax < 0.4 * kVelHalf) return false;
  std::vector<double> A(S, S + 9);
  if (!Invert(A, 3)) return false;
  for (int k = 0; k < 3; k++) v[k] = A[3] * B[k][0] + A[4] * B[k][1] + A[5] * B[k][2];
  return true;
}

// The IMU's clock against the poses': the gyro's speed against the orientations' turning over the last kLagSpan s
// (SteamVR's own angular velocity runs some 8 ms behind its orientations), the best of +-kMaxLag
void Gravity::FindLag(Dev &d, const Rx &r, double now) {
  d.next_lag = now + (d.has_lag ? kLagEvery : 5);
  if (d.poses.size() < 100 || r.s.size() < 100) return;
  double end = std::min(ImuTime(r, r.s.back()), d.poses.back().t) - kMaxLag - 0.05;
  double start = std::max({end - kLagSpan, ImuTime(r, r.s.front()), d.poses.front().t}) + kMaxLag + 0.05;
  if (end - start < 8) return;
  std::vector<double> tw, w, ti, gi;
  for (size_t k = 0; k + 3 < d.poses.size(); k++) {
    const PoseS &a = d.poses[k], &b = d.poses[k + 3];
    if (b.t < start - 0.2 || a.t > end + 0.2 || b.t - a.t > kMaxPoseGap) continue;
    tw.push_back(0.5 * (a.t + b.t));
    w.push_back(QuatDeg(a.q, b.q) / kDeg / (b.t - a.t));
  }
  for (const ImuS &s : r.s) {
    ti.push_back(ImuTime(r, s));
    gi.push_back(std::sqrt(s.g[0] * s.g[0] + s.g[1] * s.g[1] + s.g[2] * s.g[2]));
  }
  if (tw.size() < 100) return;
  const double step = 0.002;
  size_t n = (size_t)((end - start) / step), i = 0;
  std::vector<double> x(n);
  double mx = 0;
  for (size_t k = 0; k < n; k++) mx += x[k] = Interp(tw, w, start + k * step, i);
  mx /= n;
  double sx = 0;
  for (double &v : x) { v -= mx; sx += v * v; }
  if (std::sqrt(sx / n) < kMinTurn) return;  // not turning enough to tell
  std::vector<double> cs;
  for (double L = -kMaxLag; L <= kMaxLag + 1e-9; L += kLagStep) {
    std::vector<double> y(n);
    double my = 0;
    i = 0;
    for (size_t k = 0; k < n; k++) my += y[k] = Interp(ti, gi, start + k * step - L, i);
    my /= n;
    double sxy = 0, syy = 0;
    for (size_t k = 0; k < n; k++) { double e = y[k] - my; sxy += x[k] * e; syy += e * e; }
    cs.push_back(sxy / std::sqrt(sx * syy + 1e-30));
  }
  size_t k = std::max_element(cs.begin(), cs.end()) - cs.begin();
  double L = -kMaxLag + k * kLagStep;
  if (k > 0 && k + 1 < cs.size()) {
    double den = cs[k - 1] - 2 * cs[k] + cs[k + 1];
    if (den < 0) L += 0.5 * (cs[k - 1] - cs[k + 1]) / den * kLagStep;
  }
  if (cs[k] < kMinCorr) return;
  if (!d.has_lag || std::fabs(L - d.lag) > 0.005) d.lag = L;
  else d.lag += 0.3 * (L - d.lag);
  if (!d.has_lag) {  // the seconds it moved in, still buffered, count now too
    d.redo = d.next;
    d.next = -1;
  }
  if (!d.has_lag && !d.said) {
    d.said = true;
    log_(Fmt("gravity: %s's IMU: %.0f samples/s, %.0f counts per g, its clock %+.1f ms from its poses",
             d.serial.c_str(), r.rate, d.cpg, d.lag * 1000));
  }
  d.has_lag = true;
}

// whole seconds of samples and poses, in the reference frame: y = sum R a dt - (v(t1) - v(t0)), T, M = sum R dt.
// Before its clock is known (it hasn't turned enough), a device's still seconds only.
void Gravity::Windows(Dev &d, const Rx &r, const Ref &ref) {
  if (r.s.size() < 3 || d.poses.size() < 10) return;
  std::vector<double> ti(r.s.size());
  for (size_t k = 0; k < r.s.size(); k++) ti[k] = ImuTime(r, r.s[k]) + d.lag;
  double first = std::max(ti.front(), d.poses.front().t) + 0.1, avail = std::min(ti.back(), d.poses.back().t) - 0.1;
  if (d.next < first) d.next = first;
  size_t want = (size_t)std::max(20.0, 0.6 * r.rate * kWin);
  while (d.next + kWin <= avail) {
    double w0 = d.next, w1 = w0 + kWin;
    d.next = w1;
    size_t i0 = std::lower_bound(ti.begin(), ti.end(), w0) - ti.begin();
    size_t i1 = std::lower_bound(ti.begin(), ti.end(), w1) - ti.begin();
    if (i0 < 1 || i1 >= ti.size() || i1 - i0 < want) continue;
    bool ok = true;
    for (size_t i = i0; i <= i1 && ok; i++) ok = ti[i] - ti[i - 1] <= kMaxImuGap;
    if (!ok) continue;
    auto pa = std::lower_bound(d.poses.begin(), d.poses.end(), w0 - 0.04,
                               [](const PoseS &s, double x) { return s.t < x; });
    if (pa == d.poses.begin()) continue;
    double spin = 0, last_t = (pa - 1)->t;
    Quat last_q = (pa - 1)->q;
    for (auto it = pa; ok && it != d.poses.end() && last_t <= w1 + 0.04; ++it) {
      ok = it->t - last_t <= kMaxPoseGap;
      if (it->t > w0 && it->t < w1) spin += QuatDeg(last_q, it->q) / kDeg;
      last_t = it->t;
      last_q = it->q;
    }
    if (!ok || last_t <= w1 + 0.04) continue;
    bool still = spin / kWin <= kStill;
    if (!d.has_lag && !still) continue;
    if (w0 < d.redo && still) continue;  // taken before its clock was known
    V3 S, y, va, vb;
    M3 M;
    for (int a = 0; a < 3; a++) M.m[a][a] = 0;
    double Tw = 0;
    for (size_t i = i0; i < i1 && ok; i++) {
      double dt = 0.5 * (ti[i + 1] - ti[i - 1]);
      Quat q;
      V3 p;
      ok = PoseAt(d, ti[i], q, p);
      M3 R = ToM3(q) * d.head_imu;  // raw <- IMU
      V3 a;
      for (int k = 0; k < 3; k++) a[k] = (r.s[i].a[k] * (kG / d.cpg) - d.bias[k]) * d.scale[k];
      S += R * a * dt;
      for (int u = 0; u < 3; u++)
        for (int v = 0; v < 3; v++) M.m[u][v] += R.m[u][v] * dt;
      Tw += dt;
    }
    double ta = ti[i0] - 0.5 * (ti[i0] - ti[i0 - 1]), tb = ti[i1 - 1] + 0.5 * (ti[i1] - ti[i1 - 1]);
    if (!ok || !VelAt(d, ta, va) || !VelAt(d, tb, vb)) continue;
    y = S - (vb - va);
    Update(Win{d.serial, ref.C * y, Tw, ref.C * M, spin / Tw, w0});
  }
}

void Gravity::Process(double now) {
  std::map<int, std::pair<std::string, std::string>> din;
  std::map<int, std::vector<PoseS>> pin;
  std::map<std::string, std::vector<ImuS>> iin;
  Ref ref;
  {
    std::lock_guard<std::mutex> g(m_);
    din = dev_in_;
    pin.swap(pose_in_);
    iin.swap(imu_in_);
    ref = ref_;
  }
  if (ref.ok && ref.key != key_) {
    if (nwin_) log_("gravity: another reference frame: starting over");
    kx_.clear();
    kP_.clear();
    slot_.clear();
    born_.clear();
    mv_.clear();
    kt_ = -1;
    noise0_ = noise1_ = 1;
    nwin_ = 0;
    t_first_ = 1e300;
    t_last_ = -1e300;
    key_ = ref.key;
    std::lock_guard<std::mutex> g(m_);
    fit_ = Fit();
    applying_ = false;
    fit_seq_++;
  }
  // the devices, their configs, and receivers shared by two (their samples can't be told apart)
  std::map<std::string, int> per;
  for (auto &kv : din) {
    Dev &d = dev_[kv.first];
    if (d.serial != kv.second.first) d = Dev();
    d.serial = kv.second.first;
    if (d.receiver != kv.second.second) { d.receiver = kv.second.second; d.has_lag = false; d.next = -1; }
    if (!d.receiver.empty()) per[d.receiver]++;
  }
  std::set<std::string> want;
  for (auto &kv : dev_) {
    Dev &d = kv.second;
    bool shared = !d.receiver.empty() && per[d.receiver] > 1;
    if (shared && !d.shared)
      log_(Fmt("gravity: %s shares its receiver %s with another device: its IMU not used", d.serial.c_str(),
               d.receiver.c_str()));
    d.shared = shared;
    if (d.receiver.empty() || d.shared) continue;
    if (d.cfg == 0) LoadConfig(d);
    if (d.cfg == 1) want.insert(d.receiver);
  }
  // the receivers' readers (live)
  if (live_) {
    for (auto it = readers_.begin(); it != readers_.end();) {
      Reader &rd = *it->second;
      if (!want.count(it->first) || !enabled_) rd.stop = true;
      if (!rd.done) { ++it; continue; }
      rd.th.join();
      if (rd.n == 0 && run_ && enabled_ && !fails_[it->first]++)
        log_(Fmt("gravity: no IMU samples from receiver %s", it->first.c_str()));
      retry_[it->first] = now + 10;
      it = readers_.erase(it);
    }
    if (enabled_)
      for (const std::string &rc : want)
        if (!readers_.count(rc) && now >= retry_[rc]) {
          auto rd = std::make_unique<Reader>();
          Reader *p = rd.get();
          rd->th = std::thread(&Gravity::ReadLoop, this, rc, p);
          readers_[rc] = std::move(rd);
        }
  }
  if (!enabled_) { dev_.clear(); rx_.clear(); return; }
  for (auto &kv : iin) {
    Rx &r = rx_[kv.first];
    for (const ImuS &s : kv.second) Ingest(r, s);
    while (!r.s.empty() && r.s.front().arr < r.s.back().arr - kKeepS) r.s.pop_front();
    MapClock(r);
  }
  for (auto &kv : pin) {
    auto it = dev_.find(kv.first);
    if (it == dev_.end()) continue;
    Dev &d = it->second;
    for (const PoseS &p : kv.second)
      if (d.poses.empty() || p.t > d.poses.back().t) d.poses.push_back(p);
    while (!d.poses.empty() && d.poses.front().t < d.poses.back().t - kKeepS) d.poses.pop_front();
  }
  for (auto &kv : dev_) {
    Dev &d = kv.second;
    if (d.cfg != 1 || d.shared || d.receiver.empty()) continue;
    auto rit = rx_.find(d.receiver);
    if (rit == rx_.end() || !rit->second.mapped) continue;
    const Rx &r = rit->second;
    if (d.epoch != r.epoch) { d.epoch = r.epoch; d.has_lag = false; d.next = -1; }
    if (d.cpg == 0 && r.s.size() >= 2000) {  // counts per g: |a| on its calmest samples (least turning) is 1 g
      std::vector<std::pair<double, double>> ga;
      for (const ImuS &s : r.s)
        ga.push_back({std::hypot(s.g[0], s.g[1], s.g[2]), std::hypot(s.a[0], s.a[1], s.a[2])});
      std::sort(ga.begin(), ga.end());
      std::vector<double> am;
      for (size_t k = 0; k < ga.size() / 10; k++) am.push_back(ga[k].second);
      std::nth_element(am.begin(), am.begin() + am.size() / 2, am.end());
      double m = am[am.size() / 2];
      for (double c : {2048.0, 4096.0, 8192.0, 16384.0})
        if (std::fabs(m / c - 1) < 0.1) d.cpg = c;
    }
    if (!d.cpg) continue;
    if (now >= d.next_lag) FindLag(d, r, now);
    if (ref.ok && ref.key == key_) Windows(d, r, ref);
  }
  if (new_wins_ && now >= next_fit_) {
    next_fit_ = now + kFitEvery;
    new_wins_ = false;
    Report(now);
  }
  if (dirty_ && now - last_save_ >= 300) Save();
}

// A device's offset joins the filter: from the last session if kept (loosely: warming up moves it), else near 0;
// its tilt error from 0.
// Gravity first: about +y (the reference frame is SteamVR's, near level), |G| about g.
int Gravity::Slot(const std::string &dev, double t) {
  auto it = slot_.find(dev);
  if (it != slot_.end()) return it->second;
  if (kx_.empty()) {
    kx_ = {0, kG, 0};
    kP_.assign(9, 0);
    kP_[0] = kP_[8] = kSigGx * kSigGx;
    kP_[4] = kSigG * kSigG;
  }
  int n = (int)kx_.size(), m = n + 5;
  std::vector<double> P(m * m, 0);
  for (int r = 0; r < n; r++)
    for (int c = 0; c < n; c++) P[r * m + c] = kP_[r * n + c];
  auto s = prior_.find(dev);
  double sd = s != prior_.end() ? std::max(s->second.sd, kSigOffKept) : kSigOffNew;
  for (int a = 0; a < 3; a++) {
    kx_.push_back(s != prior_.end() ? s->second.o[a] : 0);
    P[(n + a) * m + n + a] = sd * sd;
  }
  for (int a = 0; a < 2; a++) {  // its tilt error (x, z), from 0
    kx_.push_back(0);
    P[(n + 3 + a) * m + n + 3 + a] = kSigTilt * kSigTilt;
  }
  kP_.swap(P);
  slot_[dev] = n;
  born_[dev] = t;
  return n;
}

// One second into the filter (Kalman): y = G T + M o + T b + noise, the noise growing with the turning and scaled by
// what the windows show. Gravity barely moves, each offset wanders (fast while its device warms up) and each tilt
// error wanders about 0. A window far out counts less, a wild one (a tracking glitch) not at all.
void Gravity::Update(const Win &w) {
  int o = Slot(w.dev, w.t), n = (int)kx_.size();
  double *P = kP_.data();
  if (kt_ >= 0 && w.t > kt_) {
    double dt = w.t - kt_;
    for (int a = 0; a < 3; a++) P[a * n + a] += kQG * dt;
    double fade = std::exp(-dt / kTauTilt);
    for (auto &kv : slot_) {
      double q = w.t - born_[kv.first] < kWarm ? kQWarm : kQ;
      int s = kv.second;
      for (int a = 0; a < 3; a++) P[(s + a) * n + s + a] += q * dt;
      for (int i = s + 3; i < s + 5; i++) {  // the tilt error wanders about 0
        kx_[i] *= fade;
        for (int c = 0; c < n; c++) { P[i * n + c] *= fade; P[c * n + i] *= fade; }
        P[i * n + i] += kSigTilt * kSigTilt * (1 - fade * fade);
      }
    }
  }
  kt_ = std::max(kt_, w.t);
  V3 G{kx_[0], kx_[1], kx_[2]}, od{kx_[o], kx_[o + 1], kx_[o + 2]};
  V3 e = w.y - G * w.T - w.M * od - V3{kx_[o + 3], 0, kx_[o + 4]} * w.T;
  // P H^T with H = [T I | M at o | T on x and z at o + 3]; H P H^T
  std::vector<double> PH(n * 3), S(9), Si;
  for (int r = 0; r < n; r++)
    for (int c = 0; c < 3; c++) {
      double v = P[r * n + c] * w.T;
      for (int j = 0; j < 3; j++) v += P[r * n + o + j] * w.M.m[c][j];
      if (c != 1) v += P[r * n + o + 3 + c / 2] * w.T;
      PH[r * 3 + c] = v;
    }
  for (int c = 0; c < 3; c++)
    for (int d = 0; d < 3; d++) {
      double v = w.T * PH[c * 3 + d];
      for (int j = 0; j < 3; j++) v += w.M.m[c][j] * PH[(o + j) * 3 + d];
      if (c != 1) v += w.T * PH[(o + 3 + c / 2) * 3 + d];
      S[c * 3 + d] = v;
    }
  auto nis = [&](double R) {  // e^T (S + R I)^-1 e, the inverse left in Si
    Si = S;
    for (int c = 0; c < 3; c++) Si[c * 4] += R;
    if (!Invert(Si, 3)) return -1.0;
    double s = 0;
    for (int c = 0; c < 3; c++)
      for (int d = 0; d < 3; d++) s += e[c] * Si[c * 3 + d] * e[d];
    return s;
  };
  // the two terms' scales, each from the windows it rules (still ones, turning ones)
  double r0 = kNoise0 * noise0_, r1 = kNoiseSpin * w.spin * w.spin * noise1_, R = r0 + r1, q = nis(R);
  if (q < 0) return;
  double &sc = r0 >= r1 ? noise0_ : noise1_;
  sc = std::max(0.01, std::min(20.0, sc * std::exp(0.02 * (std::min(q / 3, 5.0) - 1))));
  if (q > 1000) return;
  if (q > 16 && (q = nis(R * q / 16)) < 0) return;
  std::vector<double> K(n * 3);
  for (int r = 0; r < n; r++)
    for (int c = 0; c < 3; c++) {
      double v = 0;
      for (int d = 0; d < 3; d++) v += PH[r * 3 + d] * Si[d * 3 + c];
      K[r * 3 + c] = v;
    }
  for (int r = 0; r < n; r++) kx_[r] += K[r * 3] * e[0] + K[r * 3 + 1] * e[1] + K[r * 3 + 2] * e[2];
  for (int r = 0; r < n; r++)
    for (int c = 0; c < n; c++)
      P[r * n + c] -= K[r * 3] * PH[c * 3] + K[r * 3 + 1] * PH[c * 3 + 1] + K[r * 3 + 2] * PH[c * 3 + 2];
  for (int r = 0; r < n; r++)
    for (int c = r + 1; c < n; c++) P[r * n + c] = P[c * n + r] = 0.5 * (P[r * n + c] + P[c * n + r]);
  if (rec_)
    rec_(w.t, Fmt("W %s %.5f %.4f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f", w.dev.c_str(), w.T,
                  w.spin, w.y.x, w.y.y, w.y.z, w.M.m[0][0], w.M.m[0][1], w.M.m[0][2], w.M.m[1][0], w.M.m[1][1],
                  w.M.m[1][2], w.M.m[2][0], w.M.m[2][1], w.M.m[2][2]));
  nwin_++;
  t_first_ = std::min(t_first_, w.t);
  t_last_ = std::max(t_last_, w.t + kWin);
  if (w.spin > 0.3) mv_.insert(std::llround(w.t / kWin));
  new_wins_ = true;
}

void Gravity::Report(double now) {
  int n = (int)kx_.size();
  if (n < 8) return;
  Fit f;
  f.G = {kx_[0], kx_[1], kx_[2]};
  double g = norm(f.G);
  TiltOf(f.G, f.tx, f.tz);
  f.sd = std::sqrt(std::max(kP_[0], kP_[2 * n + 2])) / g * kDeg;  // recordings: subsets of the devices agree within it
  f.devs = (int)slot_.size();
  f.moving = mv_.size() * kWin;
  for (auto &kv : slot_) {
    int o = kv.second;
    double v = std::max({kP_[o * n + o], kP_[(o + 1) * n + o + 1], kP_[(o + 2) * n + o + 2]});
    f.off[kv.first] = {{kx_[o], kx_[o + 1], kx_[o + 2]}, std::sqrt(v), Now()};
  }
  f.ok = true;
  double deg = std::hypot(f.tx, f.tz) * kDeg;
  int state = f.sd > kApplySd * (applying_ ? 1.5 : 1) ? 1  // once on, it takes more to stop
              : deg > kMaxTilt                         ? 2
              : std::fabs(g / kG - 1) > kMaxGErr       ? 3
                                                       : 0;
  if (rec_) rec_(now, Fmt("I gravity %.5f %.5f %.4f %.4f %d %d", f.tx * kDeg, f.tz * kDeg, f.sd, g, f.devs, state));
  {
    std::lock_guard<std::mutex> l(m_);
    fit_ = f;
    fit_.ok = state == 0;
    fit_.key = key_;
    applying_ = fit_.ok;
    fit_seq_++;
  }
  // offsets known well enough are kept for the next session
  for (auto &kv : f.off)
    if (kv.second.sd <= kKeepSd) {
      auto s = saved_.find(kv.first);
      if (s == saved_.end() || kv.second.sd < 0.8 * s->second.sd || norm(kv.second.o - s->second.o) > 0.01) {
        saved_[kv.first] = kv.second;
        dirty_ = true;
      }
    }
  bool moved = std::hypot(f.tx - said_tx_, f.tz - said_tz_) * kDeg >= 0.05;
  if (state == said_state_ && !(state == 0 && moved && now - last_log_ >= 60)) return;
  said_state_ = state;
  said_tx_ = f.tx;
  said_tz_ = f.tz;
  last_log_ = now;
  std::string why = state == 0   ? "levelling by it"
                    : state == 1 ? "not applied yet: the devices moving and turning about tell their "
                                   "accelerometers' offsets"
                    : state == 2 ? "too far to be a level: not applied"
                                 : Fmt("gravity reads %.2f m/s^2: not applied", g);
  double span = t_last_ - t_first_;
  log_(Fmt("gravity: %d device%s over %s, %.0f s of it moving: SteamVR's room is %.2f deg off level (within %.2f); %s",
           f.devs, f.devs > 1 ? "s" : "", (span < 120 ? Fmt("%.0f s", span) : Fmt("%.0f min", span / 60)).c_str(),
           f.moving, deg, f.sd, why.c_str()));
}

void Gravity::Load() {
  std::string text;
  JVal root;
  if (!ReadText(dir_ + "\\gravity.json", text) || !JParse(text, root)) return;
  const JVal *offs = root.get("offsets");
  if (!offs || offs->t != JVal::Obj) return;
  for (auto &kv : offs->o) {
    V3 o;
    const JVal *sd = kv.second.get("sd"), *wh = kv.second.get("when");
    if (!Vec(kv.second.get("o"), o) || !sd || sd->t != JVal::Num) continue;
    prior_[kv.first] = {o, sd->n, wh && wh->t == JVal::Str ? wh->s : ""};
  }
  saved_ = prior_;
}

void Gravity::Save() {
  last_save_ = QpcNow();
  dirty_ = false;
  std::string s = "{\n \"note\": \"QuestLHSync's gravity levelling: each lighthouse device's accelerometer offset left "
                  "after its factory calibration (m/s^2, its IMU's axes) as its motion told it, and how well (sd). "
                  "Delete to start over.\",\n \"offsets\": {";
  bool first = true;
  for (auto &kv : saved_) {
    const Off &o = kv.second;
    s += Fmt("%s\n  \"%s\": {\"o\": [%.5f, %.5f, %.5f], \"sd\": %.5f, \"when\": \"%s\"}", first ? "" : ",",
             kv.first.c_str(), o.o.x, o.o.y, o.o.z, o.sd, o.when.c_str());
    first = false;
  }
  s += "\n }\n}\n";
  std::string path = dir_ + "\\gravity.json", tmp = path + ".tmp";
  FILE *f = fopen(tmp.c_str(), "wb");
  if (!f) return;
  fwrite(s.data(), 1, s.size(), f);
  fclose(f);
  MoveFileExA(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING);
}

bool Gravity::Step(const Ref &ref, bool &on, M3 &tilt) {
  {
    std::lock_guard<std::mutex> g(m_);
    ref_ = ref;
    if (fit_seq_ != seen_seq_) {
      seen_seq_ = fit_seq_;
      good_ = fit_.ok;
      tx_ = fit_.tx;
      tz_ = fit_.tz;
      fit_key_ = fit_.key;
    }
  }
  bool want = enabled_ && ref.ok && ref.key == fit_key_;
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
