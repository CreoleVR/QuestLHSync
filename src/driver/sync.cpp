#include "sync.h"

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstring>
#include <ctime>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

#include "json.h"

static const double kInf = std::numeric_limits<double>::infinity();

static std::string Fmt(const char *fmt, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  return buf;
}

static double Median(std::vector<double> v) {
  if (v.empty()) return 0;
  size_t n = v.size(), h = n / 2;
  std::nth_element(v.begin(), v.begin() + h, v.end());
  double hi = v[h];
  if (n & 1) return hi;
  double lo = *std::max_element(v.begin(), v.begin() + h);
  return (lo + hi) / 2;
}

static bool ReadFile(const std::string &path, std::string &out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::stringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

static bool WriteFileAtomic(const std::string &path, const std::string &text) {
  std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
    if (!f) return false;
    f << text;
    if (!f) return false;
  }
  std::remove(path.c_str());
  return std::rename(tmp.c_str(), path.c_str()) == 0;
}

static std::string Now() {
  time_t t = time(nullptr);
  struct tm tm;
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  char b[32];
  strftime(b, sizeof b, "%Y-%m-%d %H:%M:%S", &tm);
  return b;
}

// ================================================================ optics
void CamCal::Dist(double a, double b, double &u, double &v) const {
  double r = std::hypot(a, b), th = std::atan(r), th2 = th * th;
  double thd = th * (1 + th2 * (k[0] + th2 * (k[1] + th2 * (k[2] + th2 * (k[3] + th2 * (k[4] + th2 * k[5]))))));
  double s = r > 1e-12 ? thd / r : 1.0;
  double uu = a * s, vv = b * s, r2 = uu * uu + vv * vv, d = 2 * (uu * p[0] + vv * p[1]);
  u = uu + d * uu + r2 * p[0];
  v = vv + d * vv + r2 * p[1];
}

// Newton on the forward model
bool CamCal::Unproject(double px, double py, V3 &d) const {
  double tu = (px - cx) / f, tv = (py - cy) / fy;
  double rd = std::hypot(tu, tv), th = std::min(std::max(rd, 0.0), 1.5);
  double s = rd > 1e-12 ? std::tan(th) / rd : 1.0, a = tu * s, b = tv * s;
  const double h = 1e-6;
  for (int i = 0; i < 30; i++) {
    double u, v, ua, va, ub, vb;
    Dist(a, b, u, v);
    Dist(a + h, b, ua, va);
    Dist(a, b + h, ub, vb);
    double eu = u - tu, ev = v - tv;
    double J11 = (ua - u) / h, J21 = (va - v) / h, J12 = (ub - u) / h, J22 = (vb - v) / h;
    double det = J11 * J22 - J12 * J21;
    a -= (J22 * eu - J12 * ev) / det;
    b -= (-J21 * eu + J11 * ev) / det;
  }
  double n = std::sqrt(a * a + b * b + 1);
  if (!std::isfinite(n) || n <= 0) return false;
  d = {a / n, b / n, 1 / n};
  return true;
}

bool Optics::Load(const std::string &json, std::string *err) {
  JVal root;
  if (!JParse(json, root)) { if (err) *err = "calibration isn't JSON"; return false; }
  const JVal *kind = root.get("kind");
  if (kind && kind->t == JVal::Str && kind->s == "steam_frame") return LoadFrame(root, err);
  const JVal *cams = root.get("CameraCalibration");
  if (!cams || cams->t != JVal::Arr) { if (err) *err = "no CameraCalibration"; return false; }
  CamCal out[kMaxCam];
  int n = 0;
  for (auto &c : cams->a) {
    const JVal *id = c.get("Id"), *size = c.get("ImageSize"), *dfc = c.get("DeviceFromCamera");
    const JVal *proj = c.get("Projection"), *dist = c.get("Distortion"), *shutter = c.get("Shutter");
    const JVal *sensor = c.get("SensorType");
    if (!id || !size || !dfc || !proj || !dist) continue;
    int i = id->t == JVal::Str ? atoi(id->s.c_str()) : (int)id->num(-1);  // "Id": "2" in Meta's files
    if (i < 0 || i >= kMaxCam) continue;
    const JVal *type = shutter ? shutter->get("Type") : nullptr;
    if (type && type->t == JVal::Str && type->s == "Rolling") continue;  // the colour cameras: not tracking ones
    auto sz = size->nums(), T = dfc->nums();
    auto pc = proj->get("Coefficients") ? proj->get("Coefficients")->nums() : std::vector<double>();
    auto dc = dist->get("Coefficients") ? dist->get("Coefficients")->nums() : std::vector<double>();
    if (sz.size() < 2 || T.size() < 16 || pc.size() < 3 || dc.size() < 8) continue;
    CamCal &k = out[i];
    k.w = (int)sz[0]; k.h = (int)sz[1];
    for (int r = 0; r < 3; r++)
      for (int q = 0; q < 3; q++) k.R.m[r][q] = T[r * 4 + q];
    k.t = {T[3], T[7], T[11]};
    k.f = k.fy = pc[0]; k.cx = pc[1]; k.cy = pc[2];
    for (int j = 0; j < 6; j++) k.k[j] = dc[j];
    k.p[0] = dc[6]; k.p[1] = dc[7];
    k.ir = sensor && sensor->t == JVal::Str && sensor->s == "OV7251" ? kIR : 1.0;
    k.valid = true;
    n++;
  }
  if (!n) { if (err) *err = "no tracking camera calibration"; return false; }
  for (int i = 0; i < kMaxCam; i++) cams_[i] = out[i];
  const JVal *dev = root.get("Device"), *type = dev ? dev->get("DeviceType") : nullptr;
  device_ = type && type->t == JVal::Str ? type->s : "";
  exact_time_ = false;
  return true;
}

// a pose in the Frame's calibration files: the child frame's axes and origin, in its parent
static bool FramePose(const JVal *j, double scale, M3 &R, V3 &t) {
  if (!j) return false;
  const JVal *px = j->get("plus_x"), *pz = j->get("plus_z"), *pos = j->get("position");
  if (!px || !pz || !pos) return false;
  auto x = px->nums(), z = pz->nums(), p = pos->nums();
  if (x.size() != 3 || z.size() != 3 || p.size() != 3) return false;
  V3 X{x[0], x[1], x[2]}, Z{z[0], z[1], z[2]}, Y = cross(Z, X);
  for (int r = 0; r < 3; r++) { R.m[r][0] = X[r]; R.m[r][1] = Y[r]; R.m[r][2] = Z[r]; }
  t = V3{p[0], p[1], p[2]} * scale;
  return true;
}

// The Steam Frame's lhsyncd sends {"kind": "steam_frame", "xrservice": /persist/xrservice.json, "device_config":
// /persist/device_config.json}. xrservice.json: each tracking camera's KB4 intrinsics (Kannala-Brandt: Fisheye62 with
// only the 4 radial terms) and its pose against camera 0 (mm). device_config.json: camera 0's pose in the CAD frame
// (cv.cad_from_cal) and the head's (head), the frame SteamVR's HMD pose is of (m). Camera i's ray:
// head <- cad <- camera 0 <- camera i.
bool Optics::LoadFrame(const JVal &root, std::string *err) {
  const JVal *xr = root.get("xrservice"), *dc = root.get("device_config");
  const JVal *cams = xr ? xr->get("cameras") : nullptr, *cv = dc ? dc->get("cv") : nullptr;
  M3 Rcc, Rch;
  V3 tcc, tch;
  if (!cams || cams->t != JVal::Arr || !cv || !FramePose(cv->get("cad_from_cal"), 1.0, Rcc, tcc) ||
      !FramePose(dc->get("head"), 1.0, Rch, tch)) {
    if (err) *err = "not a Steam Frame calibration (xrservice.json cameras, device_config.json cv.cad_from_cal, head)";
    return false;
  }
  M3 Rhc = T(Rch) * Rcc;  // head <- camera 0
  V3 thc = T(Rch) * (tcc - tch);
  CamCal out[kMaxCam];
  int n = 0;
  for (size_t i = 0; i < cams->a.size() && i < (size_t)kMaxCam; i++) {
    const JVal &c = cams->a[i];
    const JVal *ex = c.get("extrinsics"), *in = c.get("intrinsics");
    const JVal *units = ex ? ex->get("__units") : nullptr;
    double scale = units && units->t == JVal::Str && units->s == "mm" ? 1e-3 : 1.0;
    M3 Rk;
    V3 tk;
    if (!FramePose(ex, scale, Rk, tk) || !in || in->t != JVal::Arr) continue;
    const JVal *kb = nullptr;
    for (auto &m : in->a)
      if (m.get("cameraModel") && m.get("cameraModel")->s == "kb") kb = &m;
    if (!kb || !c.get("width") || !c.get("height")) continue;
    auto num = [&](const char *k) { return kb->get(k) ? kb->get(k)->num(NAN) : NAN; };
    CamCal &k = out[i];
    k.w = (int)c.get("width")->num(); k.h = (int)c.get("height")->num();
    k.f = num("fx"); k.fy = num("fy"); k.cx = num("cx"); k.cy = num("cy");
    k.k[0] = num("k1"); k.k[1] = num("k2"); k.k[2] = num("k3"); k.k[3] = num("k4");
    bool finite = std::isfinite(k.f + k.fy + k.cx + k.cy + k.k[0] + k.k[1] + k.k[2] + k.k[3]) && k.f > 0 && k.fy > 0;
    if (!finite || k.w <= 0 || k.h <= 0) { k = CamCal(); continue; }
    k.R = Rhc * Rk;
    k.t = Rhc * tk + thc;
    k.valid = true;
    n++;
  }
  if (!n) {
    if (err) *err = "no usable camera in xrservice.json (KB4 intrinsics, extrinsics)";
    return false;
  }
  for (int i = 0; i < kMaxCam; i++) cams_[i] = out[i];
  const JVal *model = dc->get("model_number");
  device_ = model && model->t == JVal::Str ? model->s : "Steam Frame";
  exact_time_ = true;
  return true;
}

bool Optics::Ray(int cam, double x, double y, V3 &o, V3 &d) const {
  if (cam < 0 || cam >= kMaxCam || !cams_[cam].valid) return false;
  const CamCal &c = cams_[cam];
  double k = c.ir;
  V3 dc;
  if (!c.Unproject(c.cx + (x - c.cx) * k, c.cy + (y - c.cy) * k, dc)) return false;
  d = c.R * dc;
  double n = norm(d);
  if (!std::isfinite(n) || n <= 0) return false;
  d = d * (1 / n);
  o = c.t;
  return true;
}

bool Optics::Pose(int cam, M3 &R, V3 &t) const {
  if (cam < 0 || cam >= kMaxCam || !cams_[cam].valid) return false;
  R = cams_[cam].R;
  t = cams_[cam].t;
  return true;
}

bool Optics::InImage(int cam, V3 dc, double margin) const {
  if (cam < 0 || cam >= kMaxCam || !cams_[cam].valid || !(dc.z > 0.05)) return false;
  const CamCal &c = cams_[cam];
  double u, v;
  c.Dist(dc.x / dc.z, dc.y / dc.z, u, v);
  double x = c.cx + c.f * u / c.ir, y = c.cy + c.fy * v / c.ir;
  return x >= margin && y >= margin && x <= c.w - margin && y <= c.h - margin;
}

// ================================================================ frame grid, clock
static double PyMod(double a, double m) {
  double r = std::fmod(a, m);
  return r < 0 ? r + m : r;
}

// the widest empty stretch of the detection times folded at P: the poll lags fill the rest, so it ends at the grid
static double FoldGap(const std::deque<double> &h, double P, double *grid) {
  std::vector<double> ph;
  ph.reserve(h.size());
  for (double v : h) ph.push_back(PyMod(v, P));
  std::sort(ph.begin(), ph.end());
  size_t n = ph.size(), best = 0;
  double bg = -1;
  for (size_t i = 0; i < n; i++) {
    double g = (i + 1 < n ? ph[i + 1] : ph[0] + P) - ph[i];
    if (g > bg) { bg = g; best = i; }
  }
  if (grid) *grid = ph[(best + 1) % n];
  return bg;
}

// how tightly the detection times fold at P: the narrowest phase band holding 90% of them (stragglers, a log flush
// tens of ms late, don't count)
static double FoldSpread(const std::deque<double> &h, double P) {
  std::vector<double> ph;
  ph.reserve(h.size() * 2);
  for (double v : h) ph.push_back(PyMod(v, P));
  std::sort(ph.begin(), ph.end());
  size_t n = ph.size(), m = (n * 9 + 9) / 10;
  for (size_t i = 0; i < n; i++) ph.push_back(ph[i] + P);
  double w = P;
  for (size_t i = 0; i < n; i++) w = std::min(w, ph[i + m - 1] - ph[i]);
  return w;
}

// the short frames' period from their detection times (each a few ms after its frame): near the median gap (or the
// period found before), the one they fold at most tightly. 0 if none folds cleanly (frame times too irregular to use)
double FrameGrid::Learn(const std::deque<double> &h, double prev) {
  std::vector<double> d;
  for (size_t i = 1; i < h.size(); i++)
    if (h[i] - h[i - 1] > 0.005 && h[i] - h[i - 1] < 0.5) d.push_back(h[i] - h[i - 1]);
  if (d.size() < 20) return 0;
  double p0 = Median(d), span = h.back() - h.front();
  if (span < 20 * p0) return 0;
  // a step of the period moves the window's far end span/P times as much: steps of 2 ms there, then of 0.05 ms
  double best = p0, bw = kInf, coarse = 0.002 * p0 / span, fine = 0.00005 * p0 / span;
  auto check = [&](double p) {
    double w = FoldSpread(h, p);
    if (w < bw) { bw = w; best = p; }
  };
  double lo = prev > 0 ? 0.995 * prev : 0.9 * p0, hi = prev > 0 ? 1.005 * prev : 1.1 * p0;
  for (double p = lo; p <= hi; p += coarse) check(p);
  double c = best;
  for (double p = c - coarse; p <= c + coarse; p += fine) check(p);
  return bw <= 0.5 * best ? best : 0;
}

double FrameGrid::period(int cam) const {
  auto it = c_.find(cam);
  return it == c_.end() ? 0 : it->second.p;
}

bool FrameGrid::Lag(int cam, double t, double &lag) {
  auto &c = c_[cam];
  auto &h = c.h;
  h.push_back(t);
  while (!h.empty() && h.front() < t - WIN) h.pop_front();
  auto &lh = c.longer;
  lh.push_back(t);
  while (!lh.empty() && lh.front() < t - LEARN_WIN) lh.pop_front();
  if (h.size() < 40) return false;
  // a Quest: whichever of its two periods the frames fold at, if either folds cleanly (otherwise it's learned)
  if (quest_ && !c.p && !c.learn) {
    double a = FoldSpread(h, kQuest50), b = FoldSpread(h, kQuest60), p = b < a ? kQuest60 : kQuest50;
    if (std::min(a, b) <= 0.5 * p) c.p = p;
    else c.learn = true;
  }
  // learned: from the frames so far once the grid's window is full, then refined every WIN over up to LEARN_WIN (a
  // longer stretch pins the period closer). A camera's rate doesn't change, so a stretch too messy to fold (frames
  // misread as short ones) keeps the period found before
  if (!quest_ || c.learn) {
    if (t >= c.next && h.back() - h.front() >= 0.9 * WIN) {
      double p = Learn(lh, c.p);
      if (p > 0) c.p = p;
      c.next = t + WIN;
    }
  }
  double P = c.p;
  if (P <= 0) return false;
  double grid;
  FoldGap(h, P, &grid);
  double l = PyMod(t - grid, P);
  lag = l > P - std::min(0.01, P / 8) ? l - P : l;
  return true;
}

void Clock::AddArrival(double pc, double hs) {
  double d = pc - hs;
  long long k = (long long)std::floor(hs / 10);
  auto it = env_.find(k);
  if (it == env_.end() || d < it->second.d) { env_[k] = {hs, d, 0}; dirty_ = true; }
}

void Clock::AddPing(double pc_send, double pc_recv, double hs) {
  double rtt = pc_recv - pc_send, d = (pc_send + pc_recv) / 2 - hs;
  if (rtt < 0 || rtt > 1.0) return;
  long long k = (long long)std::floor(hs / 10);
  auto it = env_.find(k);
  if (it == env_.end() || rtt < it->second.rtt) { env_[k] = {hs, d, rtt}; dirty_ = true; }
  rtt_ = rtt_ <= 0 ? rtt : 0.9 * rtt_ + 0.1 * rtt;
}

void Clock::Fit() {
  dirty_ = false;
  while (env_.size() > 30) env_.erase(env_.begin());
  if (env_.empty()) return;
  std::vector<double> x, y;
  for (auto &kv : env_) { x.push_back(kv.second.hs); y.push_back(kv.second.d); }
  x0_ = x.back();
  if (x.size() >= 3) {
    double n = (double)x.size(), sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t i = 0; i < x.size(); i++) {
      double u = x[i] - x0_;
      sx += u; sy += y[i]; sxx += u * u; sxy += u * y[i];
    }
    double den = n * sxx - sx * sx;
    a_ = den != 0 ? (n * sxy - sx * sy) / den : 0;
    b_ = (sy - a_ * sx) / n;
  } else {
    a_ = 0;
    b_ = *std::min_element(y.begin(), y.end());
  }
  has_ = true;
}

bool Clock::Map(double hs, double &pc) {
  if (dirty_) Fit();
  if (!has_) return false;
  pc = hs + a_ * (hs - x0_) + b_;
  return true;
}

// ================================================================ poses
// Faster than this from a to b, the head didn't get there: the pose stream jumped
static bool PoseJump(const PoseS &a, const PoseS &b, bool *rot = nullptr, bool *pos = nullptr) {
  double dt = b.t - a.t;
  bool r = QuatDeg(a.q, b.q) > 5.0 + 500.0 * dt, p = norm(b.p - a.p) > 0.05 + 3.0 * dt;
  if (rot) *rot = r;
  if (pos) *pos = p;
  return r || p;
}

// A jump waits kBrkSettle s for the poses after it, since streamed poses come late, early and stale. One that comes
// back near the poses just before it made it a glitch; the head's own motion either side (with kBrkSlack s of
// timestamp slack) or a stall just before it (the jump catches up with the head) explain it too. Only the rest is a
// break: the Quest may have moved its space under the older sightings.
static constexpr double kBrkSettle = 0.2, kBrkSpan = 0.12, kBrkSlack = 0.05, kBrkStall = 0.05, kBrkBefore = 0.15;

void PoseHist::Add(double t, const Quat &q, V3 p) {
  std::lock_guard<std::mutex> g(m_);
  PoseS s{t, q, p};
  if (i_) {
    const PoseS &l = ring_[(i_ - 1) % N];
    if (t - l.t <= 0) return;  // poses must move forward in time
    if (t - l.t > 1.0) {  // a stall this long is a break, whatever the poses
      pend_ = 0;
      Break(l, s);
    } else if (pend_) {
      if (Back(s)) { i_ = pend_; pend_ = 0; }  // a glitch: its poses go
    } else if (PoseJump(l, s)) {
      pend_ = i_;
    }
  }
  ring_[i_ % N] = s;
  i_++;
  if (pend_ && t - ring_[pend_ % N].t >= kBrkSettle) Settle();
}

void PoseHist::Break(const PoseS &a, const PoseS &b) {
  brk_t_ = b.t;
  brk_what_ = Fmt("%.2f s, %.0f cm, %.1f deg", b.t - a.t, norm(b.p - a.p) * 100, QuatDeg(a.q, b.q));
  brk_id_++;
}

// s back near the poses just before the pending jump, and well off where the jump went?
bool PoseHist::Back(const PoseS &s) const {
  const PoseS &a = ring_[(pend_ - 1) % N], &c = ring_[pend_ % N];
  bool jr, jp;
  PoseJump(a, c, &jr, &jp);
  double da = QuatDeg(a.q, c.q), dp = norm(c.p - a.p);
  uint64_t lo = i_ > (uint64_t)N ? i_ - N : 0;
  for (uint64_t k = pend_; k-- > lo;) {
    const PoseS &b = ring_[k % N];
    if (b.t < a.t - kBrkSlack) break;
    if (!PoseJump(b, s) && (!jr || QuatDeg(b.q, s.q) < 0.5 * da) && (!jp || norm(s.p - b.p) < 0.5 * dp)) return true;
  }
  return false;
}

void PoseHist::Settle() {
  uint64_t lo = i_ > (uint64_t)N ? i_ - N : 0, ia = pend_ - 1, ic = pend_;
  pend_ = 0;
  const PoseS &a = ring_[ia % N], &c = ring_[ic % N];
  double dt = c.t - a.t, da = QuatDeg(a.q, c.q), dp = norm(c.p - a.p);
  // the head's speed over kBrkSpan before the jump and after it
  double w = 0, v = 0;
  auto speed = [&](uint64_t i, uint64_t j) {
    const PoseS &x = ring_[i % N], &y = ring_[j % N];
    double d = std::max(1e-3, y.t - x.t);
    w = std::max(w, QuatDeg(x.q, y.q) / d);
    v = std::max(v, norm(y.p - x.p) / d);
  };
  uint64_t ib, id;
  ib = Find(a.t - kBrkSpan, ib) ? ib + 1 : lo;
  if (ib < ia) speed(ib, ia);
  else if (ia > lo) speed(ia - 1, ia);
  id = Find(c.t + kBrkSpan, id) ? std::min(id + 1, i_ - 1) : i_ - 1;
  speed(ic, std::max(id, ic + 1));
  bool moving = da <= 5.0 + w * (dt + kBrkSlack) && dp <= 0.05 + v * (dt + kBrkSlack);
  double span = 0;  // from the pose before the oldest stall within kBrkBefore
  for (uint64_t k = ic; k > lo && ring_[k % N].t >= c.t - kBrkBefore; k--) {
    const PoseS &y = ring_[k % N], &x = ring_[(k - 1) % N];
    if (y.t - x.t > kBrkStall) span = c.t - x.t;
  }
  bool stale = span > 0 && da <= 5.0 + 800.0 * span && dp <= 0.05 + 2.0 * span;
  if (!moving && !stale) Break(a, c);
}

bool PoseHist::Find(double t, uint64_t &idx) const {
  if (!i_) return false;
  uint64_t lo = i_ > (uint64_t)N ? i_ - N : 0, hi = i_;  // valid [lo, hi)
  if (ring_[lo % N].t > t) return false;
  while (hi - lo > 1) {  // newest index with time <= t
    uint64_t mid = lo + (hi - lo) / 2;
    if (ring_[mid % N].t <= t) lo = mid; else hi = mid;
  }
  idx = lo;
  return true;
}

bool PoseHist::AtQ(double t, Quat &q, V3 &p) const {
  std::lock_guard<std::mutex> g(m_);
  uint64_t i;
  if (!Find(t, i) || i == i_ - 1) return false;  // no extrapolation past the newest pose
  const PoseS &a = ring_[i % N], &b = ring_[(i + 1) % N];
  if (b.t - a.t > 0.05) return false;
  double s = (t - a.t) / (b.t - a.t);
  q = Slerp(a.q, b.q, s);
  p = a.p + (b.p - a.p) * s;
  return true;
}

bool PoseHist::At(double t, M3 &R, V3 &p) const {
  Quat q;
  if (!AtQ(t, q, p)) return false;
  R = ToM3(q);
  return true;
}

bool PoseHist::Still(double t, double span, double rot, double pos) const {
  std::lock_guard<std::mutex> g(m_);
  uint64_t hi;
  if (!Find(t, hi)) return true;
  uint64_t lo;
  if (!Find(t - span, lo)) lo = i_ > (uint64_t)N ? i_ - N : 0;
  else if (ring_[lo % N].t < t - span) lo++;
  if (hi < lo || hi - lo + 1 < 10 || ring_[lo % N].t > t - 0.9 * span) return true;  // too little history to tell
  const PoseS &l = ring_[hi % N];
  uint64_t stride = std::max<uint64_t>(1, (hi - lo + 1) / 512);
  for (uint64_t i = lo; i <= hi; i += stride) {
    const PoseS &s = ring_[i % N];
    if (QuatDeg(s.q, l.q) >= rot || norm(s.p - l.p) >= pos) return false;
  }
  return true;
}

bool PoseHist::Speed(double t, double &w, double span) const {
  Quat a, b;
  V3 pa, pb;
  if (!AtQ(t - span, a, pa) || !AtQ(t, b, pb)) return false;
  w = QuatDeg(a, b) / span;
  return true;
}

bool PoseHist::Latest(V3 &p, double *t) const {
  std::lock_guard<std::mutex> g(m_);
  if (!i_) return false;
  p = ring_[(i_ - 1) % N].p;
  if (t) *t = ring_[(i_ - 1) % N].t;
  return true;
}

// ================================================================ reference frame
static Quat QuatFromJ(const JVal *v) {
  auto q = v ? v->nums() : std::vector<double>();
  if (q.size() < 4) return {};
  return {q[0], q[1], q[2], q[3]};  // w x y z
}

bool StationsFile::Load(const std::string &path) {
  *this = StationsFile();
  std::string text;
  JVal d;
  if (!ReadFile(path, text) || !JParse(text, d)) return false;
  const JVal *a = d.get("anchor"), *ap = d.get("anchor_p"), *aq = d.get("anchor_q");
  if (!a || a->t != JVal::Str || !ap || !aq) return false;
  auto p = ap->nums();
  if (p.size() < 3) return false;
  anchor = a->s;
  anchor_p = {p[0], p[1], p[2]};
  anchor_R = ToM3(QuatFromJ(aq));
  if (const JVal *lq = d.get("level")) level = ToM3(QuatFromJ(lq));
  if (const JVal *st = d.get("stations"))
    for (auto &kv : st->o) {
      if (const JVal *pos = kv.second.get("pos")) {
        auto v = pos->nums();
        if (v.size() >= 3) fix[kv.first] = {v[0], v[1], v[2]};
      }
      if (const JVal *rp = kv.second.get("ref")) {
        auto v = rp->nums();
        if (v.size() >= 3) ref[kv.first] = {V3{v[0], v[1], v[2]}, ToM3(QuatFromJ(kv.second.get("ref_q")))};
      }
    }
  const JVal *ag = d.get("auto");
  autogen = ag && ag->t == JVal::Bool && ag->b;
  loaded = true;
  return true;
}

bool StationsFile::SaveAuto(const std::string &path) const {
  Quat q = ToQuat(anchor_R), l = ToQuat(level);
  std::string st;
  for (auto &kv : ref) {
    Quat r = ToQuat(kv.second.second);
    const V3 &p = kv.second.first;
    st += Fmt("%s\n  \"%s\": {\"ref\": [%.6f, %.6f, %.6f], \"ref_q\": [%.6f, %.6f, %.6f, %.6f]}", st.empty() ? "" : ",",
              kv.first.c_str(), p.x, p.y, p.z, r.w, r.x, r.y, r.z);
  }
  if (!st.empty()) st += "\n ";
  std::string s = Fmt(
      "{\n \"auto\": true,\n \"note\": \"QuestLHSync's reference frame: %s's pose when it was first seen. SteamVR re-tilts "
      "its lighthouse universe at every start; this pins it. stations: each base station's pose in it; with three or "
      "more, the frame follows the best fit of them all. level: the turn that levels it with the headset's gravity, "
      "found from the base stations' sightings. Delete to start over.\",\n", anchor.c_str());
  s += Fmt(" \"anchor\": \"%s\",\n \"anchor_p\": [%.6f, %.6f, %.6f],\n \"anchor_q\": [%.6f, %.6f, %.6f, %.6f],\n"
           " \"level\": [%.8f, %.8f, %.8f, %.8f],\n \"stations\": {", anchor.c_str(), anchor_p.x, anchor_p.y,
           anchor_p.z, q.w, q.x, q.y, q.z, l.w, l.x, l.y, l.z);
  s += st + Fmt("},\n \"saved\": \"%s\"\n}\n", Now().c_str());
  return WriteFileAtomic(path, s);
}

// symmetric 4 x 4: the eigenvector of the largest eigenvalue (Jacobi)
static void MaxEigVec4(double A[4][4], double v[4]) {
  double a[4][4], V[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  memcpy(a, A, sizeof a);
  for (int sweep = 0; sweep < 50; sweep++) {
    double off = 0;
    for (int i = 0; i < 4; i++)
      for (int j = i + 1; j < 4; j++) off += a[i][j] * a[i][j];
    if (off < 1e-30) break;
    for (int p = 0; p < 4; p++)
      for (int q = p + 1; q < 4; q++) {
        if (std::fabs(a[p][q]) < 1e-300) continue;
        double th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        double t = (th >= 0 ? 1 : -1) / (std::fabs(th) + std::sqrt(th * th + 1));
        double c = 1 / std::sqrt(t * t + 1), s = t * c;
        for (int k = 0; k < 4; k++) {
          double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
          double vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
        for (int k = 0; k < 4; k++) {
          double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
      }
  }
  int b = 0;
  for (int i = 1; i < 4; i++)
    if (a[i][i] > a[b][b]) b = i;
  for (int k = 0; k < 4; k++) v[k] = V[k][b];
}

// the turn R that puts points a best on points b about their centres: R (a - ca) ~ b - cb. Three or more off one line:
// Horn's quaternion method; fewer: a turn about the vertical alone, keeping `prev`'s tilt (one: `prev` itself).
// Returns the worst miss.
static double FitTurn(const std::vector<V3> &a, const std::vector<V3> &b, const std::vector<char> &use, const M3 &prev,
                      M3 &R, V3 &ca, V3 &cb) {
  std::vector<size_t> u;
  for (size_t i = 0; i < a.size(); i++)
    if (use[i]) u.push_back(i);
  ca = cb = {};
  for (size_t i : u) { ca += a[i]; cb += b[i]; }
  ca = ca * (1.0 / u.size());
  cb = cb * (1.0 / u.size());
  double line = 0;  // how far the stations reach off the line through the two furthest apart
  if (u.size() >= 3) {
    size_t i0 = u[0], i1 = u[1];
    for (size_t i : u)
      for (size_t j : u)
        if (norm(a[i] - a[j]) > norm(a[i0] - a[i1])) { i0 = i; i1 = j; }
    V3 d = a[i1] - a[i0];
    d = d * (1.0 / norm(d));
    for (size_t i : u) line = std::max(line, norm(cross(a[i] - a[i0], d)));
  }
  if (line >= kLine) {
    double S[3][3] = {};
    for (size_t i : u) {
      V3 p = a[i] - ca, q = b[i] - cb;
      for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) S[r][c] += p[r] * q[c];
    }
    double N[4][4] = {
        {S[0][0] + S[1][1] + S[2][2], S[1][2] - S[2][1], S[2][0] - S[0][2], S[0][1] - S[1][0]},
        {S[1][2] - S[2][1], S[0][0] - S[1][1] - S[2][2], S[0][1] + S[1][0], S[2][0] + S[0][2]},
        {S[2][0] - S[0][2], S[0][1] + S[1][0], -S[0][0] + S[1][1] - S[2][2], S[1][2] + S[2][1]},
        {S[0][1] - S[1][0], S[2][0] + S[0][2], S[1][2] + S[2][1], -S[0][0] - S[1][1] + S[2][2]}};
    double v[4];
    MaxEigVec4(N, v);
    R = ToM3(Quat{v[0], v[1], v[2], v[3]});
  } else {
    const auto &m = prev.m;
    M3 tilt = Ry(-std::atan2(m[0][2] - m[2][0], m[0][0] + m[2][2])) * prev;
    double sn = 0, cs = 0;
    for (size_t i : u) {
      V3 p = tilt * (a[i] - ca), q = b[i] - cb;
      sn += p.z * q.x - p.x * q.z;
      cs += p.x * q.x + p.z * q.z;
    }
    R = u.size() >= 2 ? Ry(std::atan2(sn, cs)) * tilt : prev;
  }
  double worst = 0;
  for (size_t i : u) worst = std::max(worst, norm(R * (a[i] - ca) - (b[i] - cb)));
  return worst;
}

// An automatic frame of three or more stations: SteamVR's frame -> the reference frame is the best fit of all the
// stations' places, not one station's pose. SteamVR re-solves the stations now and then, each a few cm and a degree or
// so its own way; one station's pose then turns the frame, and the level with it, while the fit of them all hardly
// moves. A station kMoved off when the rest fit within it is left out (`left`, `off`: how far); `miss`: the worst miss
// of the stations kept.
bool Frame::Layout(const std::map<std::string, std::pair<V3, M3>> &raw, M3 &Cs, V3 &o, std::string &left,
                   double &off, double &miss) const {
  std::vector<std::string> keys;
  std::vector<V3> a, b;
  M3 prev = Cs_;
  for (auto &kv : raw) {
    auto r = f_.ref.find(kv.first);
    if (r == f_.ref.end()) continue;
    if (keys.empty() && ok_ != 1) prev = r->second.second * T(kv.second.second);  // no fit yet: a station's own turn
    keys.push_back(kv.first);
    a.push_back(kv.second.first);
    b.push_back(r->second.first);
  }
  if (keys.empty()) return false;
  std::vector<char> use(keys.size(), 1);
  V3 ca, cb;
  double worst = FitTurn(a, b, use, prev, Cs, ca, cb);
  left.clear();
  off = 0;
  if (keys.size() >= 3 && worst > kMoved) {  // one station moved: the fit of the rest that misses least
    double best = kMoved;
    M3 R;
    V3 pa, pb;
    for (size_t i = 0; i < keys.size(); i++) {
      use.assign(keys.size(), 1);
      use[i] = 0;
      double w = FitTurn(a, b, use, prev, R, pa, pb);
      if (w < best) {
        best = w;
        left = keys[i];
        off = norm(R * (a[i] - pa) - (b[i] - pb));
        Cs = R; ca = pa; cb = pb;
      }
    }
    if (!left.empty()) worst = best;
  }
  miss = worst;
  o = ca + T(Cs) * (f_.anchor_p - cb);  // about the anchor's place, as the level turns it
  return true;
}

void Frame::Update(const std::map<std::string, std::pair<V3, M3>> &raw) {
  if (!f_.loaded) return;
  M3 Cs;
  V3 o;
  bool lay = layout();
  if (lay) {
    std::string left;
    double off;
    if (!Layout(raw, Cs, o, left, off, miss_)) return;
    if (left != left_ && !left.empty())
      log_(Fmt("lighthouse frame: base station %s is %.0f cm off against the others: left out of the fit", left.c_str(),
               off * 100));
    left_ = left;
  } else {
    auto it = raw.find(f_.anchor);
    if (it == raw.end()) return;
    Cs = f_.anchor_R * T(it->second.second);
    o = it->second.first;
  }
  double ang = RotDeg(Cs);
  int ok = lay || ang < kMaxRot;
  if (ok != ok_ || (ok && RotDeg(Cs * T(Cs_)) > 0.01))
    log_(Fmt("lighthouse frame: SteamVR's is %.2f deg from the reference%s", ang,
             ok ? "" : " (over 3 deg: the anchor moved or a new universe; SteamVR's frame as it is)"));
  ok_ = ok;
  deg_ = ang;
  if (ok) {
    Cs_ = Cs;
    GC_ = grav_ ? G_ : M3();
    C_ = GC_ * f_.level * Cs;
    o_ = o;
    oref_ = f_.anchor_p;
  } else {
    Cs_ = C_ = GC_ = M3();
    o_ = {};
    oref_ = {};
  }
}

// ================================================================ solver
Solver::Solver(LogFn log, uint64_t seed, std::map<std::string, V3> fix)
    : fix_(std::move(fix)), log_(std::move(log)), rng_(seed) {}

void Solver::Reset(const X4 &x) {
  x_ = x; has_x_ = true;
  resets_++;
  anchor_ = x; has_anchor_ = true;
  id_said_.clear();
}

void Solver::Add(double t, V3 o, V3 d, double gt, int cam, bool bright) {
  std::lock_guard<std::mutex> g(m_);
  rt_.push_back(t); ro_.push_back(o); rd_.push_back(d); rg_.push_back(gt); rc_.push_back(cam); rb_.push_back(bright);
}

void Solver::Replace(double from, const std::vector<double> &T, const std::vector<V3> &O, const std::vector<V3> &D,
                     const std::vector<double> &G, const std::vector<int> &C, const std::vector<char> &B) {
  std::lock_guard<std::mutex> g(m_);
  size_t j = std::lower_bound(rt_.begin(), rt_.end(), from) - rt_.begin();
  rt_.resize(j); ro_.resize(j); rd_.resize(j); rg_.resize(j); rc_.resize(j); rb_.resize(j);
  rt_.insert(rt_.end(), T.begin(), T.end());
  ro_.insert(ro_.end(), O.begin(), O.end());
  rd_.insert(rd_.end(), D.begin(), D.end());
  rg_.insert(rg_.end(), G.begin(), G.end());
  rc_.insert(rc_.end(), C.begin(), C.end());
  rb_.insert(rb_.end(), B.begin(), B.end());
}

void Solver::SetChannel(const std::string &serial, int channel) {
  std::lock_guard<std::mutex> g(m_);
  ch_[serial] = channel;
}

void Solver::AddFrame(double t, double gt, int cam, V3 o, const M3 &R) {
  std::lock_guard<std::mutex> g(m_);
  frames_.push_back({t, gt, cam, o, R});
  while (frames_.front().t < t - kKeep) frames_.pop_front();
}

void Solver::SetStation(const std::string &serial, V3 p, const M3 &R) {
  auto f = fix_.find(serial);
  if (f != fix_.end()) {
    double d = norm(p - f->second);
    if (d < kMoved) { p = f->second; stale_.erase(serial); }
    else if (!stale_.count(serial)) {
      stale_.insert(serial);
      log_(Fmt("%s: SteamVR has it %.0f cm from where it was measured: moved? using SteamVR's pose", serial.c_str(), d * 100));
    }
  }
  std::lock_guard<std::mutex> g(m_);
  S_[serial] = {p, R};
}

Solver::Rays Solver::GetRays(double t0) {
  std::lock_guard<std::mutex> g(m_);
  if (!rt_.empty() && rt_.front() < rt_.back() - kKeep) {
    size_t j = std::lower_bound(rt_.begin(), rt_.end(), rt_.back() - kKeep) - rt_.begin();
    rt_.erase(rt_.begin(), rt_.begin() + j);
    ro_.erase(ro_.begin(), ro_.begin() + j);
    rd_.erase(rd_.begin(), rd_.begin() + j);
    rg_.erase(rg_.begin(), rg_.begin() + j);
    rc_.erase(rc_.begin(), rc_.begin() + j);
    rb_.erase(rb_.begin(), rb_.begin() + j);
  }
  Rays r;
  size_t j = std::lower_bound(rt_.begin(), rt_.end(), t0) - rt_.begin();
  r.T.assign(rt_.begin() + j, rt_.end());
  r.O.assign(ro_.begin() + j, ro_.end());
  r.D.assign(rd_.begin() + j, rd_.end());
  r.G.assign(rg_.begin() + j, rg_.end());
  r.C.assign(rc_.begin() + j, rc_.end());
  r.B.assign(rb_.begin() + j, rb_.end());
  return r;
}

void Solver::Stations(std::vector<std::string> &keys, std::vector<V3> &S, std::vector<V3> &Z) const {
  std::lock_guard<std::mutex> g(m_);
  keys.clear(); S.clear(); Z.clear();
  for (auto &kv : S_) { keys.push_back(kv.first); S.push_back(kv.second.first); Z.push_back(kv.second.second.col(2)); }
}

void Solver::Predict(const X4 &x, const std::vector<V3> &S, const std::vector<V3> &Z, std::vector<V3> &P, std::vector<V3> &Zq) {
  double c = std::cos(x[0]), s = std::sin(x[0]);
  V3 t{x[1], x[2], x[3]};
  P.resize(S.size()); Zq.resize(S.size());
  for (size_t k = 0; k < S.size(); k++) { P[k] = RyMul(c, s, S[k]) + t; Zq[k] = RyMul(c, s, Z[k]); }
}

// nearest station to a ray by angle (deg); inf where the head is behind every station
void Solver::Nearest(const std::vector<V3> &P, const std::vector<V3> &Zq, V3 o, V3 d, int &k, double &a) {
  k = 0; a = kInf;
  for (size_t j = 0; j < P.size(); j++) {
    V3 U = P[j] - o;
    double n = norm(U);
    double ang = kInf;
    if (dot(U, Zq[j]) >= 0.2 * n) {
      double c = dot(U, d) / n;
      ang = std::acos(c < -1 ? -1 : c > 1 ? 1 : c) * kDeg;
    }
    if (ang < a) { a = ang; k = (int)j; }
  }
}

void Solver::Support(const X4 &x, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r, double gate,
                     std::vector<int> &cnt) const {
  std::vector<V3> P, Zq;
  Predict(x, S, Z, P, Zq);
  cnt.assign(S.size(), 0);
  for (size_t n = 0; n < r.size(); n++) {
    int k; double a;
    Nearest(P, Zq, r.O[n], r.D[n], k, a);
    if (a < gate) cnt[k]++;
  }
}

int Solver::Score(const std::vector<int> &c) {  // yaw needs two stations: the second-best count
  if (c.size() < 2) return 0;
  std::vector<int> v = c;
  std::sort(v.begin(), v.end());
  return v[v.size() - 2];
}

struct CellKey {
  int64_t k[6];
  bool operator==(const CellKey &o) const { return !memcmp(k, o.k, sizeof k); }
};
struct CellHash {
  size_t operator()(const CellKey &c) const {
    uint64_t h = 1469598103934665603ull;
    for (int i = 0; i < 6; i++) { h ^= (uint64_t)c.k[i]; h *= 1099511628211ull; }
    return (size_t)h;
  }
};
// (head 5 cm, direction ~2 deg) cell of a sighting; np.round is round-half-even, so is nearbyint
static CellKey KeyOf(V3 o, V3 d) {
  return {{(int64_t)std::nearbyint(o.x / 0.05), (int64_t)std::nearbyint(o.y / 0.05), (int64_t)std::nearbyint(o.z / 0.05),
           (int64_t)std::nearbyint(d.x * 30), (int64_t)std::nearbyint(d.y * 30), (int64_t)std::nearbyint(d.z * 30)}};
}

void Solver::Cells(const Rays &r, const std::vector<char> &use, std::vector<int> &cnt) {
  std::unordered_map<CellKey, int, CellHash> m;
  for (size_t n = 0; n < r.size(); n++)
    if (use[n]) m[KeyOf(r.O[n], r.D[n])]++;
  cnt.assign(r.size(), 0);
  for (size_t n = 0; n < r.size(); n++)
    if (use[n]) cnt[n] = m[KeyOf(r.O[n], r.D[n])];
}

constexpr int kMaxDof = 6;  // Fit's unknowns: yaw, translation and, when levelling, a tilt

// symmetric n x n (n <= kMaxDof): smallest eigenvalue (Jacobi)
static double MinEig(double A[kMaxDof][kMaxDof], int n) {
  double a[kMaxDof][kMaxDof];
  memcpy(a, A, sizeof a);
  for (int sweep = 0; sweep < 50; sweep++) {
    double off = 0;
    for (int i = 0; i < n; i++)
      for (int j = i + 1; j < n; j++) off += a[i][j] * a[i][j];
    if (off < 1e-30) break;
    for (int p = 0; p < n; p++)
      for (int q = p + 1; q < n; q++) {
        if (std::fabs(a[p][q]) < 1e-300) continue;
        double th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        double t = (th >= 0 ? 1 : -1) / (std::fabs(th) + std::sqrt(th * th + 1));
        double c = 1 / std::sqrt(t * t + 1), s = t * c;
        for (int k = 0; k < n; k++) {
          double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < n; k++) {
          double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
      }
  }
  double m = a[0][0];
  for (int i = 1; i < n; i++) m = std::min(m, a[i][i]);
  return m;
}

static bool Solve(double A[kMaxDof][kMaxDof], const double *b, double *x, int n) {
  double M[kMaxDof][kMaxDof + 1];
  for (int i = 0; i < n; i++) { for (int j = 0; j < n; j++) M[i][j] = A[i][j]; M[i][n] = b[i]; }
  for (int c = 0; c < n; c++) {
    int p = c;
    for (int r = c + 1; r < n; r++) if (std::fabs(M[r][c]) > std::fabs(M[p][c])) p = r;
    if (std::fabs(M[p][c]) < 1e-300) return false;
    if (p != c) for (int j = 0; j <= n; j++) std::swap(M[p][j], M[c][j]);
    for (int r = 0; r < n; r++) {
      if (r == c) continue;
      double f = M[r][c] / M[c][c];
      for (int j = c; j <= n; j++) M[r][j] -= f * M[c][j];
    }
  }
  for (int i = 0; i < n; i++) x[i] = M[i][n] / M[i][i];
  return true;
}

// Robust fit (soft_l1 on the sightings, quadratic prior rows), sightings assigned to the nearest station under x0; yaw
// about the station centroid. A (head, direction) cell weighs one sighting however many it holds, so a still head adds
// nothing by looking longer. xa: the prior's center (default x0). cond: the sightings alone pin every DOF.
// 4 DOF; with a pivot 6: the stations also tilt about it (Tilt(y[4], y[5])), and the prior holds only the tilt.
Solver::FitR Solver::Fit(const X4 &x0, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r, double now,
                         double gate, const X4 *xa_in, const V3 *pivot, double dim_gate, const V3 *axis) {
  FitR out;
  std::vector<V3> P, Zq;
  Predict(x0, S, Z, P, Zq);
  std::vector<char> use(r.size(), 0);
  std::vector<int> kk(r.size(), 0);
  int m = 0;
  for (size_t n = 0; n < r.size(); n++) {
    double a;
    Nearest(P, Zq, r.O[n], r.D[n], kk[n], a);
    if (a < (r.B.empty() || r.B[n] ? gate : std::min(gate, dim_gate))) { use[n] = 1; m++; }
  }
  out.n = m;
  if (m < 15) return out;
  std::vector<int> cnt;
  Cells(r, use, cnt);
  struct Row { int k; V3 o, d; double sw; };
  std::vector<Row> rows;
  rows.reserve(m);
  for (size_t n = 0; n < r.size(); n++)
    if (use[n]) rows.push_back({kk[n], r.O[n], r.D[n], std::sqrt(std::exp(-(now - r.T[n]) / TAU) / cnt[n])});
  const X4 xa = xa_in ? *xa_in : x0;
  V3 c;
  for (auto &s : S) c += s;
  c = c * (1.0 / S.size());
  V3 cq = Ry(x0[0]) * c + V3{x0[1], x0[2], x0[3]};
  V3 cqa = Ry(xa[0]) * c + V3{xa[1], xa[2], xa[3]};
  std::vector<V3> Sc(S.size());  // about the centroid; with a pivot, about the pivot (pc: the pivot about the centroid)
  for (size_t k = 0; k < S.size(); k++) Sc[k] = S[k] - (pivot ? *pivot : c);
  const V3 pc = pivot ? *pivot - c : V3{};
  if (!pivot) axis = nullptr;
  const int np = pivot ? (axis ? 5 : 6) : 4;
  const double f = FSCALE;
  const size_t nrob = rows.size() * 3, nr = nrob + (pivot ? (axis ? 1 : 2) : 4);

  auto res = [&](const double *y, std::vector<double> &rv) {
    rv.resize(nr);
    double cy = std::cos(y[0]), sy = std::sin(y[0]);
    V3 sh = cq + V3{y[1], y[2], y[3]};
    M3 Rt;
    if (pivot) Rt = axis ? Tilt(y[4] * axis->x, y[4] * axis->z) : Tilt(y[4], y[5]);
    for (size_t i = 0; i < rows.size(); i++) {
      const Row &w = rows[i];
      V3 U = RyMul(cy, sy, pivot ? pc + Rt * Sc[w.k] : Sc[w.k]) + sh - w.o;
      U = U * (1 / norm(U));
      V3 e = cross(U, w.d) * w.sw;
      rv[3 * i] = e.x; rv[3 * i + 1] = e.y; rv[3 * i + 2] = e.z;
    }
    if (pivot) {
      rv[nrob] = f * y[4] / SIG_LEVEL;
      if (!axis) rv[nrob + 1] = f * y[5] / SIG_LEVEL;
      return;
    }
    rv[nrob] = f * Wrap(y[0] - xa[0]) / SIG_YAW;
    rv[nrob + 1] = f * (sh.x - cqa.x) / SIG_T;
    rv[nrob + 2] = f * (sh.y - cqa.y) / SIG_T;
    rv[nrob + 3] = f * (sh.z - cqa.z) / SIG_T;
  };
  auto cost = [&](const std::vector<double> &rv) {
    double s = 0;
    for (size_t i = 0; i < nrob; i++) { double z = rv[i] / f; s += 2 * f * f * (std::sqrt(1 + z * z) - 1); }
    for (size_t i = nrob; i < nr; i++) s += rv[i] * rv[i];
    return s;
  };
  auto jac = [&](const double *y, std::vector<double> &J) {  // central differences, row-major nr x np
    J.assign(nr * np, 0);
    std::vector<double> rp, rm;
    for (int j = 0; j < np; j++) {
      double h = 1e-7, yp[kMaxDof], ym[kMaxDof];
      memcpy(yp, y, sizeof yp); memcpy(ym, y, sizeof ym);
      yp[j] += h; ym[j] -= h;
      res(yp, rp); res(ym, rm);
      for (size_t i = 0; i < nr; i++) J[i * np + j] = (rp[i] - rm[i]) / (2 * h);
    }
  };

  double y[kMaxDof] = {x0[0], 0, 0, 0, 0, 0};
  std::vector<double> rv, J, rn;
  res(y, rv);
  double cst = cost(rv), lam = 1e-3;
  for (int it = 0; it < 100; it++) {
    jac(y, J);
    double A[kMaxDof][kMaxDof] = {}, g[kMaxDof] = {};
    for (size_t i = 0; i < nr; i++) {
      double w = 1;
      if (i < nrob) { double z = rv[i] / f; w = 1 / std::sqrt(1 + z * z); }  // IRLS weight of soft_l1
      const double *Ji = &J[i * np];
      for (int a = 0; a < np; a++) {
        g[a] += w * Ji[a] * rv[i];
        for (int b = 0; b < np; b++) A[a][b] += w * Ji[a] * Ji[b];
      }
    }
    bool moved = false, done = false;
    for (int tries = 0; tries < 20; tries++) {
      double Al[kMaxDof][kMaxDof], ng[kMaxDof], d[kMaxDof];
      for (int a = 0; a < np; a++) {
        for (int b = 0; b < np; b++) Al[a][b] = A[a][b];
        Al[a][a] += lam * std::max(A[a][a], 1e-12);
        ng[a] = -g[a];
      }
      if (!Solve(Al, ng, d, np)) { lam *= 10; continue; }
      double yn[kMaxDof], step = 0;
      memcpy(yn, y, sizeof yn);
      for (int a = 0; a < np; a++) yn[a] += d[a];
      res(yn, rn);
      double cn = cost(rn);
      if (cn <= cst) {
        for (int a = 0; a < np; a++) step += std::fabs(d[a]);
        done = step < 1e-11 || cst - cn <= 1e-14 * std::max(cst, 1e-30);
        memcpy(y, yn, sizeof y);
        rv.swap(rn);
        cst = cn;
        lam = std::max(lam / 3, 1e-9);
        moved = true;
        break;
      }
      lam *= 4;
      if (lam > 1e12) break;
    }
    if (!moved || done) break;
  }
  // conditioning: scipy's loss-scaled Jacobian of the sighting rows ((1+z)^-3/2 for soft_l1), per-DOF scale / SIG_R
  jac(y, J);
  double H[kMaxDof][kMaxDof] = {};
  const double sc[kMaxDof] = {COND_YAW, COND_T, COND_T, COND_T, COND_LEVEL, COND_LEVEL};
  for (size_t i = 0; i < nrob; i++) {
    double z = rv[i] / f, w = std::pow(1 + z * z, -1.5);
    const double *Ji = &J[i * np];
    for (int a = 0; a < np; a++)
      for (int b = 0; b < np; b++) H[a][b] += w * Ji[a] * Ji[b] * sc[a] * sc[b] / (SIG_R * SIG_R);
  }
  out.cond = MinEig(H, np) >= 1.0;
  if (pivot) out.tilt = axis ? Tilt(y[4] * axis->x, y[4] * axis->z) : Tilt(y[4], y[5]);
  if (axis) out.theta = y[4];
  double yaw = Wrap(y[0]);
  V3 t = cq + V3{y[1], y[2], y[3]} - Ry(yaw) * c;
  out.x = {yaw, t.x, t.y, t.z};
  out.ok = true;
  return out;
}

// acquisition: 2-ray minimal solver (one ray per station, both through their stations) + consensus
void Solver::Hypotheses(const Rays &r, const std::vector<V3> &S, std::vector<X4> &H, int M) {
  H.clear();
  size_t N = r.size(), K = S.size();
  if (N < 2 || K < 2) return;
  std::uniform_int_distribution<size_t> U(0, N - 1);
  const double c10 = std::cos(10 / kDeg);
  std::vector<std::pair<size_t, size_t>> pr;
  for (int t = 0; t < M; t++) {
    size_t i = U(rng_), j = U(rng_);
    if (dot(r.D[i], r.D[j]) < c10) pr.push_back({i, j});
  }
  for (size_t ka = 0; ka < K; ka++)
    for (size_t kb = 0; kb < K; kb++) {
      if (ka == kb) continue;
      V3 Dv = S[ka] - S[kb];  // R Dv = (oi + l di) - (oj + m dj)
      for (auto &ij : pr) {
        V3 oi = r.O[ij.first], di = r.D[ij.first], oj = r.O[ij.second], dj = r.D[ij.second];
        V3 e = oi - oj;
        double a = di.y, b = -dj.y, c = Dv.y - e.y, n2 = a * a + b * b;
        if (!(n2 > 1e-6)) continue;
        double l0 = c * a / n2, m0 = c * b / n2, dl = -b, dm = a;
        double Ax = e.x + l0 * di.x - m0 * dj.x, Az = e.z + l0 * di.z - m0 * dj.z;
        double Bx = dl * di.x - dm * dj.x, Bz = dl * di.z - dm * dj.z;
        double qa = Bx * Bx + Bz * Bz, qb = 2 * (Ax * Bx + Az * Bz), qc = Ax * Ax + Az * Az - Dv.x * Dv.x - Dv.z * Dv.z;
        double disc = qb * qb - 4 * qa * qc;
        if (!(disc >= 0) || !(qa > 1e-9)) continue;
        double sq = std::sqrt(disc);
        for (int sg = -1; sg <= 1; sg += 2) {
          double s = (-qb + sg * sq) / (2 * qa);
          double lam = l0 + s * dl, mu = m0 + s * dm;
          if (!(lam > 0.3 && mu > 0.3 && lam < 15 && mu < 15)) continue;
          V3 v = e + di * lam - dj * mu;
          double yaw = std::atan2(v.x, v.z) - std::atan2(Dv.x, Dv.z);
          double cy = std::cos(yaw), sy = std::sin(yaw);
          V3 t = oi + di * lam - RyMul(cy, sy, S[ka]);
          H.push_back({yaw, t.x, t.y, t.z});
        }
      }
    }
}

// (H, K) support counts; each ray counts for the station it's most in front of / closest to
void Solver::BatchSupport(const std::vector<X4> &H, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r,
                          const std::vector<int> &idx, double gate, std::vector<int> &out) const {
  size_t K = S.size();
  out.assign(H.size() * K, 0);
  double cg = std::cos(gate / kDeg);
  std::vector<V3> P(K), Zq(K);
  std::vector<double> cs(K);
  std::vector<char> fr(K);
  for (size_t h = 0; h < H.size(); h++) {
    double c = std::cos(H[h][0]), s = std::sin(H[h][0]);
    V3 t{H[h][1], H[h][2], H[h][3]};
    for (size_t k = 0; k < K; k++) { P[k] = RyMul(c, s, S[k]) + t; Zq[k] = RyMul(c, s, Z[k]); }
    int *o = &out[h * K];
    for (int n : idx) {
      int best = 0;
      double bv = -kInf;
      for (size_t k = 0; k < K; k++) {
        V3 U = P[k] - r.O[n];
        double nn = norm(U);
        cs[k] = dot(U, r.D[n]) / nn;
        fr[k] = dot(U, Zq[k]) > 0.2 * nn;
        double v = fr[k] ? cs[k] : -2;
        if (v > bv) { bv = v; best = (int)k; }
      }
      if (cs[best] > cg && fr[best]) o[best]++;
    }
  }
}

bool Solver::Starved(const Rays &r) {
  size_t nb = 0;
  for (char b : r.B) nb += b != 0;
  return !r.B.empty() && r.size() >= 200 && nb * 20 < r.size();
}

Solver::Rays Solver::Bright(const Rays &r) {
  size_t nb = 0;
  for (char b : r.B) nb += b != 0;
  if (r.B.empty() || nb == r.size() || Starved(r)) return r;
  Rays o;
  for (size_t n = 0; n < r.size(); n++) {
    if (!r.B[n]) continue;
    o.T.push_back(r.T[n]); o.G.push_back(r.G[n]); o.O.push_back(r.O[n]); o.D.push_back(r.D[n]);
    o.C.push_back(r.C[n]); o.B.push_back(1);
  }
  return o;
}

// one ray per (5 cm head cell, ~2 deg world direction): a lamp seen from a still head collapses to a few rays,
// so consensus only counts parallax
Solver::Rays Solver::Thin(const Rays &r) {
  std::unordered_map<CellKey, size_t, CellHash> first;
  std::vector<size_t> keep;
  for (size_t n = 0; n < r.size(); n++)
    if (first.emplace(KeyOf(r.O[n], r.D[n]), n).second) keep.push_back(n);
  Rays o;
  for (size_t n : keep) {
    o.T.push_back(r.T[n]); o.O.push_back(r.O[n]); o.D.push_back(r.D[n]);
    if (!r.B.empty()) o.B.push_back(r.B[n]);
  }
  return o;
}

static int SecondBest(const int *c, size_t K) {
  if (K < 2) return 0;
  int a = -1, b = -1;  // best, second
  for (size_t k = 0; k < K; k++) {
    if (c[k] > a) { b = a; a = c[k]; }
    else if (c[k] > b) b = c[k];
  }
  return b;
}

// best hypothesis from scratch on the last ACQ_WIN s: score = thinned support, tight = the same within TIGHT_DEG
bool Solver::Acquire(double now, X4 &best, int &bs, int &tight) {
  std::vector<std::string> keys;
  std::vector<V3> S, Z;
  Stations(keys, S, Z);
  Rays all = GetRays(std::max(now - ACQ_WIN, since_)), r = Bright(all);
  acq_dim_ = r.size() == all.size() && std::count(all.B.begin(), all.B.end(), 0) > 0;
  if (S.size() < 2 || r.size() < 30) return false;
  Rays t = Thin(r);
  std::vector<X4> H;
  Hypotheses(t, S, H);
  std::vector<X4> cand;
  if (has_acq_x_) cand.push_back(acq_x_);
  size_t K = S.size();
  if (!H.empty()) {
    std::vector<int> sub(t.size());
    for (size_t i = 0; i < sub.size(); i++) sub[i] = (int)i;
    size_t ns = std::min<size_t>(t.size(), 400);
    for (size_t i = 0; i < ns; i++) {  // partial Fisher-Yates: a random subset without replacement
      std::uniform_int_distribution<size_t> u(i, sub.size() - 1);
      std::swap(sub[i], sub[u(rng_)]);
    }
    sub.resize(ns);
    std::vector<int> c1;
    BatchSupport(H, S, Z, t, sub, 1.5, c1);
    std::vector<size_t> ord(H.size());
    for (size_t i = 0; i < ord.size(); i++) ord[i] = i;
    size_t n1 = std::min<size_t>(40, ord.size());
    std::partial_sort(ord.begin(), ord.begin() + n1, ord.end(),
                      [&](size_t a, size_t b) { return SecondBest(&c1[a * K], K) > SecondBest(&c1[b * K], K); });
    std::vector<X4> top;
    for (size_t i = 0; i < n1; i++) top.push_back(H[ord[i]]);
    std::vector<int> all(t.size()), c2;
    for (size_t i = 0; i < all.size(); i++) all[i] = (int)i;
    BatchSupport(top, S, Z, t, all, 1.5, c2);
    std::vector<size_t> o2(top.size());
    for (size_t i = 0; i < o2.size(); i++) o2[i] = i;
    size_t n2 = std::min<size_t>(5, o2.size());
    std::partial_sort(o2.begin(), o2.begin() + n2, o2.end(),
                      [&](size_t a, size_t b) { return SecondBest(&c2[a * K], K) > SecondBest(&c2[b * K], K); });
    for (size_t i = 0; i < n2; i++) cand.push_back(top[o2[i]]);
  }
  bs = -1;
  bool have = false;
  for (X4 x : cand) {
    for (double gate : {1.5, 0.6}) {
      FitR f = Fit(x, S, Z, r, now, gate, nullptr);
      if (!f.ok) break;
      x = f.x;
    }
    std::vector<int> c;
    Support(x, S, Z, t, INLIER, c);
    int sc = Score(c);
    if (sc > bs) { best = x; bs = sc; have = true; }
  }
  if (!have) return false;
  CheckMirror(best, bs, S, Z, r, t, now);
  acq_x_ = best; has_acq_x_ = true;
  std::vector<int> c;
  Support(best, S, Z, t, TIGHT_DEG, c);
  tight = Score(c);
  return true;
}

int Solver::ThinnedScore(const X4 &x, double now) {
  std::vector<std::string> keys;
  std::vector<V3> S, Z;
  Stations(keys, S, Z);
  Rays t = Thin(Bright(GetRays(std::max(now - ACQ_WIN, since_))));
  if (!t.size()) return 0;
  std::vector<int> c;
  Support(x, S, Z, t, INLIER, c);
  return Score(c);
}

// x turned 180 deg about the vertical through the midpoint of stations a and b (reference frame): their places swap
static X4 Mirror(const X4 &x, V3 a, V3 b) {
  V3 m = (a + b) * 0.5, d = RyMul(std::cos(x[0]), std::sin(x[0]), V3{2 * m.x, 0, 2 * m.z});
  return {Wrap(x[0] + kPi), x[1] + d.x, x[2] + d.y, x[3] + d.z};
}

// how far apart two alignments put the stations (m, the farthest one)
static double Apart(const X4 &a, const X4 &b, const std::vector<V3> &S) {
  double ca = std::cos(a[0]), sa = std::sin(a[0]), cb = std::cos(b[0]), sb = std::sin(b[0]), far = 0;
  for (const V3 &s : S)
    far = std::max(far, norm(RyMul(ca, sa, s) + V3{a[1], a[2], a[3]} - RyMul(cb, sb, s) - V3{b[1], b[2], b[3]}));
  return far;
}

static std::string Meters(double d) { return std::isfinite(d) ? Fmt("%.1f m", d) : std::string("none"); }

// Their motion first: worn or held, they go where the head goes, and a fit and its mirror (turned 180 deg) move them
// opposite ways, wherever one stands and whatever lies about. Then how near the head they stay.
int Solver::BodyFavours(const X4 &a, const X4 &b, double &da, double &db, std::string &how) const {
  da = body_ ? body_(a) : NAN;
  db = body_ ? body_(b) : NAN;
  double moved = 0, ma = motion_ ? motion_(a, moved) : NAN, mb = motion_ ? motion_(b, moved) : NAN;
  if (std::isfinite(ma) && std::isfinite(mb) && moved >= kBodyMoved && std::max(ma, mb) >= kBodyCorr &&
      std::fabs(ma - mb) >= 2 * kBodyCorr) {
    how = Fmt("the lighthouse devices worn or held move with the head one way (%+.2f against %+.2f)", std::max(ma, mb),
              std::min(ma, mb));
    return ma > mb ? 1 : 2;
  }
  if (!std::isfinite(da) || !std::isfinite(db) || std::min(da, db) >= kBodyNear || std::fabs(da - db) <= kBodyGap)
    return 0;
  how = Fmt("the lighthouse devices worn or held stay near the head one way (%s from it, %s the other way)",
            Meters(std::min(da, db)).c_str(), Meters(std::max(da, db)).c_str());
  return da < db ? 1 : 2;
}

// Two stations facing each other at about the same height look the same with their places swapped: the fit turned
// 180 deg about their midpoint explains the sightings almost as well, and puts every other lighthouse device across the
// room. So the mirror of the best fit is fitted too. When the two score about the same, the lighthouse devices worn or
// held (they stay near the head) decide; without them the current fit stays.
void Solver::CheckMirror(X4 &best, int &bs, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r,
                         const Rays &t, double now) {
  acq_forced_ = false;
  std::vector<int> c;
  Support(best, S, Z, t, INLIER, c);
  size_t a = 0, b = 1;  // the two best-seen stations
  if (c[b] > c[a]) std::swap(a, b);
  for (size_t k = 2; k < c.size(); k++) {
    if (c[k] > c[a]) { b = a; a = k; }
    else if (c[k] > c[b]) b = k;
  }
  // the stations' own spacing and heights differ a little from where the cameras see their lasers: the exact mirror
  // starts a few degrees off, so its refit starts wide
  X4 m = Mirror(best, S[a], S[b]);
  for (double gate : {2.0, 1.0, 0.6}) {
    FitR f = Fit(m, S, Z, r, now, gate, nullptr);
    if (!f.ok) break;
    m = f.x;
  }
  if (Apart(m, best, S) < 0.3) return;  // the mirror slid back into the best fit
  std::vector<int> cm;
  Support(m, S, Z, t, INLIER, cm);
  int sm = Score(cm);
  // thinned support is a small, noisy count: within a factor 2 the two are a tie
  if (sm * 2 < bs) { amb_state_ = 0; return; }  // clearly worse: not ambiguous
  if (bs * 2 < sm) { amb_state_ = 0; best = m; bs = sm; return; }
  double db, dm;
  std::string how;
  int fav = BodyFavours(best, m, db, dm, how);
  bool pick_m = false;
  int state;
  std::string why;
  if (fav) {
    pick_m = fav == 2;
    state = pick_m ? 1 : 2;
    why = how + (pick_m ? ": the mirror" : ": the fit");
  } else if (has_x_) {
    pick_m = Apart(m, x_, S) < Apart(best, x_, S);
    state = 3;
    why = "no lighthouse device near the head tells them apart: the current fit stays";
  } else {
    state = 4;
    why = "no lighthouse device near the head tells them apart: wear or hold one";
  }
  if (state != amb_state_)
    log_(Fmt("mirror check: yaw %+.1f (support %d, devices %s from the head) and its mirror yaw %+.1f (support %d, %s) "
             "fit about as well: %s", best[0] * kDeg, bs, Meters(db).c_str(), m[0] * kDeg, sm, Meters(dm).c_str(),
             why.c_str()));
  amb_state_ = state;
  X4 other = pick_m ? best : m;
  if (pick_m) { best = m; bs = sm; }
  // the devices' evidence flips the current fit to its own mirror; a pair without the current fit in it is just
  // another candidate
  acq_forced_ = (state == 1 || state == 2) && has_x_ && Apart(best, x_, S) > 0.3 && Apart(other, x_, S) < 1.0;
}

// the HMD pose stream broke: the Quest's space may have changed under the older sightings (boundary reset)
void Solver::PoseBreak(double t) {
  brk_ = since_ = std::max(since_, t);
  last_acq_ = -1e18;
  has_acq_x_ = false;
}

// ================================================================ station identity
// A 2.0 base station's rotor period is set by its channel (ticks of 48 MHz). A short frame sees a station only in
// part of its rotor turn, so which frames see it tells the stations apart where the geometry can't.
static const int kRotorTicks[16] = {959000, 957000, 953000, 949000, 947000, 943000, 941000, 939000,
                                    937000, 929000, 919000, 911000, 907000, 901000, 893000, 887000};

double RotorPeriod(int channel) { return channel >= 1 && channel <= 16 ? kRotorTicks[channel - 1] / 48e6 : 0; }

struct InView { int cam; double g; bool seen; };  // a short frame a station was in view of

// turns/s the phase at period tau moves from one short frame to the next
static double Drift(double tau, double P) {
  double f = std::fmod(P / tau, 1.0);
  return std::min(f, 1 - f) / P;
}

// How much seeing a station (in the frames it was in view of, sorted by camera, time) follows the phase at each
// period, over fake periods near it. Both on the same per-camera windows of whole turns of the slower drift. False if
// too few frames.
static bool SeenScores(const std::vector<InView> &s, double tau1, double tau2, double P, double &s1, double &s2) {
  const double kTurns = 2, kEps[] = {0.0015, 0.0025, 0.0035, 0.0045, 0.0055, 0.0065, 0.0075, 0.0085};
  const int kMinN = 60, kMinSeen = 20;
  s1 = s2 = 0;
  double rate = std::min(Drift(tau1, P), Drift(tau2, P));
  if (!(rate > 0)) return false;
  double W = std::max(20.0, (kTurns + 1) / rate);
  struct Win { size_t a, b; double p; };  // frames [a, b), share seen
  std::vector<Win> win;
  int n = 0, ns = 0;
  for (size_t i = 0; i < s.size();) {
    size_t j = i;
    while (j < s.size() && s[j].cam == s[i].cam && s[j].g < s[i].g + W) j++;
    double turns = std::floor((s[j - 1].g - s[i].g) * rate);
    if (turns >= kTurns) {
      size_t k = i;
      int seen = 0;
      for (; k < j && s[k].g < s[i].g + turns / rate; k++) seen += s[k].seen;
      if (seen > 0 && seen < (int)(k - i)) {
        win.push_back({i, k, seen / (double)(k - i)});
        n += (int)(k - i);
        ns += seen;
      }
    }
    i = j;
  }
  if (n < kMinN || ns < kMinSeen) return false;
  auto R = [&](double T) {
    double tot = 0;
    for (auto &w : win) {
      double c = 0, sn = 0, g0 = s[w.a].g;
      for (size_t k = w.a; k < w.b; k++) {
        double a = 2 * kPi * (s[k].g - g0) / T, e = s[k].seen - w.p;
        c += e * std::cos(a);
        sn += e * std::sin(a);
      }
      tot += std::hypot(c, sn);
    }
    return tot / n;
  };
  auto score = [&](double tau) {
    double null = 0;
    for (double e : kEps) null += R(tau * (1 + e)) + R(tau * (1 - e));
    return std::max(0.0, R(tau) - null / (2 * (sizeof kEps / sizeof kEps[0])));
  };
  s1 = score(tau1);
  s2 = score(tau2);
  return true;
}

// yaw + translation taking points S onto Q, least squares
static X4 YawFit(const std::vector<V3> &S, const std::vector<V3> &Q) {
  V3 ms, mq;
  for (size_t i = 0; i < S.size(); i++) { ms += S[i]; mq += Q[i]; }
  ms = ms * (1.0 / S.size()); mq = mq * (1.0 / S.size());
  double sn = 0, cs = 0;
  for (size_t i = 0; i < S.size(); i++) {
    V3 a = S[i] - ms, b = Q[i] - mq;
    sn += a.z * b.x - a.x * b.z;
    cs += a.z * b.z + a.x * b.x;
  }
  double yaw = std::atan2(sn, cs);
  V3 t = mq - Ry(yaw) * ms;
  return {yaw, t.x, t.y, t.z};
}

// A pair of stations whose sightings follow each other's rotor periods better than their own is swapped: switch to
// the alignment with the two exchanged if it fits the rays about as well.
void Solver::CheckIdentity(double now, const std::vector<std::string> &keys, const std::vector<V3> &S,
                           const std::vector<V3> &Z) {
  double P = frame_p_;
  if (!(P > 0) || keys.size() < 2) return;
  std::vector<double> tau(keys.size(), 0.0);
  {
    std::lock_guard<std::mutex> g(m_);
    for (size_t k = 0; k < keys.size(); k++) {
      auto it = ch_.find(keys[k]);
      if (it != ch_.end()) tau[k] = RotorPeriod(it->second);
    }
  }
  if (!in_image_) return;
  Rays r = Bright(GetRays(since_));
  std::vector<CamFrame> fr;
  {
    std::lock_guard<std::mutex> g(m_);
    for (auto &f : frames_)
      if (f.t >= since_) fr.push_back(f);
  }
  std::vector<V3> Pq, Zq;
  Predict(x_, S, Z, Pq, Zq);
  std::vector<std::set<std::pair<int, double>>> saw(keys.size());  // (camera, frame) with a sighting within 1 deg
  for (size_t n = 0; n < r.size(); n++) {
    if (!std::isfinite(r.G[n])) continue;
    int k;
    double a;
    Nearest(Pq, Zq, r.O[n], r.D[n], k, a);
    if (a < 1.0) saw[k].insert({r.C[n], r.G[n]});
  }
  std::vector<std::vector<InView>> in(keys.size());
  for (auto &f : fr)
    for (size_t k = 0; k < keys.size(); k++) {
      V3 U = Pq[k] - f.o;
      double d = norm(U);
      if (!(dot(U, Zq[k]) >= 0.2 * d) || !in_image_(f.cam, T(f.R) * (U * (1 / d)))) continue;
      in[k].push_back({f.cam, f.g, saw[k].count({f.cam, f.g}) > 0});
    }
  for (auto &v : in)
    std::sort(v.begin(), v.end(), [](const InView &a, const InView &b) { return a.cam != b.cam ? a.cam < b.cam : a.g < b.g; });
  double worst = 0, best = 0;
  size_t wa = 0, wb = 0, ba = 0, bb = 0;
  for (size_t a = 0; a < keys.size(); a++)
    for (size_t b = a + 1; b < keys.size(); b++) {
      if (!tau[a] || !tau[b] || tau[a] == tau[b]) continue;
      double aa, ab, bb_, ba_;
      if (!SeenScores(in[a], tau[a], tau[b], P, aa, ab) || !SeenScores(in[b], tau[b], tau[a], P, bb_, ba_)) continue;
      double gain = ab + ba_ - aa - bb_;
      if (gain > worst) { worst = gain; wa = a; wb = b; }
      if (gain < best) { best = gain; ba = a; bb = b; }
    }
  auto say = [&](const std::string &key, const std::string &s) {  // once per alignment
    if (key == id_said_) return;
    id_said_ = key;
    log_(s);
  };
  if (worst > kSwapGain) {
    std::vector<V3> Q = Pq;
    std::swap(Q[wa], Q[wb]);
    X4 xs = YawFit(S, Q);
    for (double gate : {1.5, 0.6}) {
      FitR f = Fit(xs, S, Z, r, now, gate, nullptr);
      if (!f.ok) break;
      xs = f.x;
    }
    int cur = ThinnedScore(x_, now), sw = ThinnedScore(xs, now);
    std::string pair = keys[wa] + " and " + keys[wb];
    // the devices worn or held are the stronger evidence, and the mirror check goes by them: a switch against them
    // would only be switched back
    double dc, ds;
    std::string how;
    if (BodyFavours(x_, xs, dc, ds, how) == 1) {
      say("body " + pair, Fmt("channel check: %s look the wrong way round (by %.2f), but %s: kept as it is",
                              pair.c_str(), worst, how.c_str()));
    } else if (cur > 0 && sw >= kSwapFit * cur) {
      log_(Fmt("channel check: %s were the wrong way round (by %.2f): yaw %+.2f deg t [%.3f %.3f %.3f] support %d (was %d)",
               pair.c_str(), worst, xs[0] * kDeg, xs[1], xs[2], xs[3], sw, cur));
      Reset(xs);
      has_acq_x_ = false;
    } else {
      say("kept " + pair, Fmt("channel check: %s look the wrong way round (by %.2f), but the rays fit that way worse "
                              "(support %d vs %d): kept as it is", pair.c_str(), worst, sw, cur));
    }
  } else if (best < -kSwapGain) {
    say("ok " + keys[ba] + keys[bb], Fmt("channel check: %s are the right way round (by %.2f)",
                                         (keys[ba] + " and " + keys[bb]).c_str(), -best));
  }
}

// call ~1 Hz. The solver's clock is the newest sighting: with nothing new seen nothing ages out or is refitted
StepStat Solver::Step(double /*now*/) {
  std::vector<std::string> keys;
  std::vector<V3> S, Z;
  Stations(keys, S, Z);
  double newest;
  {
    std::lock_guard<std::mutex> g(m_);
    newest = rt_.empty() ? -kInf : rt_.back();
  }
  if (!(newest > seen_)) {
    StepStat st = has_stat_ ? stat_ : StepStat();
    st.jump = false; st.has_acq = false; st.cond = false;
    stat_ = st; has_stat_ = true;
    return st;
  }
  double now = seen_ = newest;
  starved_ = Starved(GetRays(std::max(now - ACQ_WIN, since_)));
  StepStat st;
  if (!S.empty() && has_x_) {
    Rays r = GetRays(std::max(now - WIN, since_));
    if (r.size()) {
      std::vector<V3> P, Zq;
      Predict(x_, S, Z, P, Zq);
      std::vector<int> k(r.size());
      std::vector<double> ak(r.size());
      for (size_t n = 0; n < r.size(); n++) Nearest(P, Zq, r.O[n], r.D[n], k[n], ak[n]);
      for (size_t kk = 0; kk < S.size(); kk++) {  // a station's recent sightings moved away from its older ones:
        std::vector<double> rec, old;          // the Quest's tracking jumped, forget the older sightings
        for (size_t n = 0; n < r.size(); n++)
          if (k[n] == (int)kk && ak[n] < (r.B[n] ? GATE : DimGate())) (r.T[n] > now - 4 ? rec : old).push_back(ak[n]);
        if (rec.size() >= 20 && old.size() >= 20 && Median(rec) - Median(old) > JUMP) {
          since_ = now - 4;
          st.jump = true;
          r = GetRays(since_);
          break;
        }
      }
      FitR f = Fit(x_, S, Z, r, now, GATE, has_anchor_ ? &anchor_ : nullptr, nullptr, DimGate());
      if (f.ok) {
        x_ = f.x;
        if (f.cond || !has_anchor_) { anchor_ = f.x; has_anchor_ = true; }
      }
      st.cond = f.cond;
      std::vector<int> cnt;
      Support(x_, S, Z, r, INLIER, cnt);
      Predict(x_, S, Z, P, Zq);
      std::vector<double> in;
      for (size_t n = 0; n < r.size(); n++) {
        int kk; double a;
        Nearest(P, Zq, r.O[n], r.D[n], kk, a);
        if (a < INLIER) {
          in.push_back(a);
          double &l = st.last[keys[kk]];
          l = std::max(l, r.T[n]);
        }
      }
      st.n = f.n;
      st.has_per = true;
      for (size_t kk = 0; kk < keys.size(); kk++) st.per[keys[kk]] = cnt[kk];
      if (!in.empty()) { st.has_med = true; st.med = Median(in); }
    }
  }
  if (has_x_ && now - last_id_ >= kIdEvery) {
    last_id_ = now;
    CheckIdentity(now, keys, S, Z);
  }
  bool locked = false;
  if (has_x_ && st.has_per) {
    std::vector<int> c;
    for (auto &kv : st.per) c.push_back(kv.second);
    locked = Score(c) >= 10;
  }
  if (now - last_acq_ > (locked ? 10 : 3)) {
    last_acq_ = now;
    X4 xa;
    int sa, tight;
    if (Acquire(now, xa, sa, tight)) {
      int cur = has_x_ ? ThinnedScore(x_, now) : 0;
      st.has_acq = true; st.acq_sa = sa; st.acq_cur = cur; st.acq_tight = tight;
      // the mirror check's flip has the worn devices for evidence as well: half the support will do. A fit far off the
      // current one (four stations in a square fit about as well turned by 90 deg) goes by the worn or held devices
      // too; without them it takes twice the support, or tight sightings: a wrong fit that lines stations up with other
      // stations or lamps gathers a dozen, few of them tight, and after a pose break the current one has few yet
      int k = acq_dim_ ? 2 : 1;  // dim rays: other lights line up with a wrong alignment more easily
      double dn, dc;
      std::string how;
      bool far = has_x_ && !acq_forced_ && Apart(xa, x_, S) > 1.0;
      int fav = far ? BodyFavours(xa, x_, dn, dc, how) : 0;
      int need = far && !fav ? 2 * ACQ : ACQ;
      bool enough = sa >= k * need || tight >= k * TIGHT_N || (acq_forced_ && sa >= k * ACQ / 2);
      if (!enough || !(sa > 2 * cur + 5 || acq_forced_)) {
      } else if (fav == 2) {
        if (now - kept_said_ > 60) {
          kept_said_ = now;
          log_(Fmt("acquisition: yaw %+.1f deg (support %d, was %d) would fit as well, but %s: the current fit stays",
                   xa[0] * kDeg, sa, cur, how.c_str()));
        }
      } else {
        log_(Fmt("acquired: yaw %+.2f deg t [%.3f %.3f %.3f] support %d (%d within %.1f deg, was %d)%s", xa[0] * kDeg, xa[1],
                 xa[2], xa[3], sa, tight, TIGHT_DEG, cur, acq_forced_ ? ", by the mirror check" : ""));
        Reset(xa);
        since_ = brk_;
      }
    }
  }
  stat_ = st; has_stat_ = true;
  return st;
}

// The 4-DOF fit's stations tilted about pivot as well, on the same sightings. Its first gate is wide: with the frame
// 1.5 deg off level the fit of two stations puts the third 4 deg away from where the cameras see it. Kept only if the
// tilt is pinned (cond) by enough stations, each seen from several (head, direction) cells within INLIER.
bool Solver::Level(V3 pivot, LevelR &out) {
  if (!has_x_) return false;
  std::vector<std::string> keys;
  std::vector<V3> S0, Z0;
  Stations(keys, S0, Z0);
  if ((int)S0.size() < LEVEL_STATIONS) return false;
  const double now = seen_;
  Rays r = GetRays(std::max(now - WIN, since_));
  X4 x = x_;
  M3 L;
  std::vector<V3> S(S0.size()), Z(Z0.size());
  auto level = [&] {
    for (size_t k = 0; k < S0.size(); k++) { S[k] = pivot + L * (S0[k] - pivot); Z[k] = L * Z0[k]; }
  };
  FitR f;
  for (double gate : {6.0, GATE, 0.6}) {
    level();
    f = Fit(x, S, Z, r, now, gate, nullptr, &pivot, DimGate());
    if (!f.ok) return false;
    x = f.x;
    L = f.tilt * L;
  }
  if (!f.cond) return false;
  level();
  std::vector<int> cells;
  Support(x, S, Z, Thin(r), INLIER, cells);
  int held = 0;
  for (int c : cells) held += c >= LEVEL_CELLS;
  std::vector<V3> P, Zq;
  Predict(x, S, Z, P, Zq);
  std::vector<double> in;
  for (size_t n = 0; n < r.size(); n++) {
    int k; double a;
    Nearest(P, Zq, r.O[n], r.D[n], k, a);
    if (a < INLIER) in.push_back(a);
  }
  if (held < LEVEL_STATIONS || in.empty()) return false;
  double med = Median(in);
  if (med > LEVEL_MED) return false;
  out.x = x;
  out.tilt = L;
  out.stations = held;
  out.med = med;
  return true;
}

// With two stations the cameras can't see a tilt about the line through them (both stay put), but they do see one
// across it: that raises one station and lowers the other. The 4-DOF fit's stations tilted about axis through
// pivot, on the last WIN s of sightings, and on four time slices of them: their scatter says how well it's known.
bool Solver::LevelAxis(V3 pivot, V3 axis, AxisR &out) {
  if (!has_x_) return false;
  std::vector<std::string> keys;
  std::vector<V3> S0, Z0;
  Stations(keys, S0, Z0);
  if (S0.size() < 2) return false;
  const double now = seen_;
  Rays r = GetRays(std::max(now - WIN, since_));
  if (r.size() < 200) return false;
  auto fit = [&](const Rays &rr, double &th, double *med) {
    X4 x = x_;
    double t = 0;
    std::vector<V3> S(S0.size()), Z(Z0.size());
    for (double gate : {GATE, 0.6}) {
      M3 L = Tilt(t * axis.x, t * axis.z);
      for (size_t k = 0; k < S0.size(); k++) { S[k] = pivot + L * (S0[k] - pivot); Z[k] = L * Z0[k]; }
      FitR f = Fit(x, S, Z, rr, now, gate, nullptr, &pivot, DimGate(), &axis);
      if (!f.ok) return false;
      x = f.x;
      t += f.theta;
    }
    th = t;
    if (med) {  // and each station seen from enough (head, direction) cells
      M3 L = Tilt(t * axis.x, t * axis.z);
      for (size_t k = 0; k < S0.size(); k++) { S[k] = pivot + L * (S0[k] - pivot); Z[k] = L * Z0[k]; }
      std::vector<int> cells;
      Support(x, S, Z, Thin(rr), INLIER, cells);
      for (int c : cells)
        if (c < LEVEL_CELLS) return false;
      std::vector<V3> P, Zq;
      Predict(x, S, Z, P, Zq);
      std::vector<double> in;
      for (size_t n = 0; n < rr.size(); n++) {
        int k;
        double a;
        Nearest(P, Zq, rr.O[n], rr.D[n], k, a);
        if (a < INLIER) in.push_back(a);
      }
      if (in.empty()) return false;
      *med = Median(in);
    }
    return true;
  };
  if (!fit(r, out.theta, &out.med) || out.med > LEVEL_MED) return false;
  std::vector<double> ts;
  double t0 = r.T.front(), t1 = r.T.back();
  for (int q = 0; q < 4; q++) {
    Rays s;
    for (size_t n = 0; n < r.size(); n++)
      if (r.T[n] >= t0 + (t1 - t0) * q / 4 && r.T[n] < t0 + (t1 - t0) * (q + 1) / 4 + (q == 3 ? 1 : 0)) {
        s.T.push_back(r.T[n]); s.G.push_back(r.G[n]); s.O.push_back(r.O[n]); s.D.push_back(r.D[n]);
        s.C.push_back(r.C[n]); s.B.push_back(r.B[n]);
      }
    double th;
    if (s.size() >= 50 && fit(s, th, nullptr)) ts.push_back(th);
  }
  out.slices = (int)ts.size();
  if (ts.size() < 3) return false;
  double m = 0, v = 0;
  for (double t : ts) m += t;
  m /= ts.size();
  for (double t : ts) v += (t - m) * (t - m);
  out.sd = std::sqrt(v / (ts.size() - 1) / ts.size());
  return true;
}

void Solver::Relevel(const X4 &x) {
  x_ = anchor_ = x;
  has_x_ = has_anchor_ = true;
  has_acq_x_ = false;  // found on the stations before the level
}

// ================================================================ timing
void Timing::Record(double tg, V3 od, V3 dd, double hg, int cam, bool bright) {
  rec_.push_back({tg, od, dd, hg, cam, bright});
  while (!rec_.empty() && (rec_.front().tg < tg - 120 || rec_.size() > 20000)) rec_.pop_front();
}

// The alignment to test timings against, refit from slow-head sightings only (a timing error moves those < 0.1 deg):
// the solver's own alignment was shaped by the old timing, or was just seeded from the last session, and holding it
// pulled estimates toward it (3 ms instead of ~18 right after a seed). IRLS Gauss-Newton, Cauchy 0.3 deg, a weak pull
// toward the start for directions the sightings don't pin. False unless both stations have slow sightings.
static bool RefitSlow(X4 &x, const std::vector<V3> &S, const std::vector<V3> &Z, const std::vector<V3> &O,
                      const std::vector<V3> &D) {
  if (S.size() < 2 || O.size() < 30) return false;
  const X4 x0 = x;
  std::vector<V3> P, Zq;
  Solver::Predict(x, S, Z, P, Zq);
  std::vector<int> ks;
  std::vector<size_t> use;
  std::vector<int> per(S.size(), 0);
  for (size_t i = 0; i < O.size(); i++) {
    int k;
    double a;
    Solver::Nearest(P, Zq, O[i], D[i], k, a);
    if (!(a < 1.0)) continue;
    use.push_back(i);
    ks.push_back(k);
    per[k]++;
  }
  for (int c : per)
    if (c < 10) return false;
  const double r0 = 0.3 / kDeg, c2 = r0 * r0;
  const double sig[4] = {0.5 / kDeg, 0.05, 0.05, 0.05};  // the pull toward the start: one sighting's worth at this far
  auto res = [&](const X4 &y, std::vector<V3> &E) {
    std::vector<V3> Py, Zy;
    Solver::Predict(y, S, Z, Py, Zy);
    E.resize(use.size());
    for (size_t j = 0; j < use.size(); j++) {
      V3 U = Py[ks[j]] - O[use[j]];
      E[j] = cross(U * (1 / norm(U)), D[use[j]]);
    }
  };
  std::vector<V3> E, Ep, Em;
  for (int it = 0; it < 10; it++) {
    res(x, E);
    double H[4][4] = {}, g[4] = {};
    std::vector<double> w(E.size());
    for (size_t j = 0; j < E.size(); j++) w[j] = 1 / (1 + dot(E[j], E[j]) / c2);
    std::vector<std::array<V3, 4>> J(E.size());
    for (int q = 0; q < 4; q++) {
      const double h = q == 0 ? 1e-6 : 1e-5;
      X4 xp = x, xm = x;
      xp[q] += h;
      xm[q] -= h;
      res(xp, Ep);
      res(xm, Em);
      for (size_t j = 0; j < E.size(); j++) J[j][q] = (Ep[j] - Em[j]) * (1 / (2 * h));
    }
    for (size_t j = 0; j < E.size(); j++)
      for (int a = 0; a < 4; a++) {
        g[a] += w[j] * dot(J[j][a], E[j]);
        for (int b = 0; b < 4; b++) H[a][b] += w[j] * dot(J[j][a], J[j][b]);
      }
    for (int a = 0; a < 4; a++) {
      double pw = c2 / (sig[a] * sig[a]);
      H[a][a] += pw;
      g[a] += pw * (a == 0 ? Wrap(x[0] - x0[0]) : x[a] - x0[a]);
    }
    double dx[4];  // solve H dx = -g (Gaussian elimination, partial pivoting)
    for (int a = 0; a < 4; a++) {
      int p = a;
      for (int b = a + 1; b < 4; b++)
        if (std::fabs(H[b][a]) > std::fabs(H[p][a])) p = b;
      if (std::fabs(H[p][a]) < 1e-18) return false;
      if (p != a) {
        for (int c = 0; c < 4; c++) std::swap(H[a][c], H[p][c]);
        std::swap(g[a], g[p]);
      }
      for (int b = a + 1; b < 4; b++) {
        double f = H[b][a] / H[a][a];
        for (int c = a; c < 4; c++) H[b][c] -= f * H[a][c];
        g[b] -= f * g[a];
      }
    }
    for (int a = 3; a >= 0; a--) {
      double s = -g[a];
      for (int c = a + 1; c < 4; c++) s -= H[a][c] * dx[c];
      dx[a] = s / H[a][a];
    }
    for (int a = 0; a < 4; a++) x[a] += dx[a];
    if (std::fabs(dx[0]) < 1e-7 && std::fabs(dx[1]) + std::fabs(dx[2]) + std::fabs(dx[3]) < 1e-5) break;
  }
  return std::isfinite(x[0] + x[1] + x[2] + x[3]);
}

// The time a sighting's HMD pose is looked up at is (frame grid point - offset). The offset is the camera's own
// delay plus how the streamer times the poses it hands SteamVR (some predict ahead). A wrong offset shows only while
// the head turns: grid search the robust error of fast-head sightings against an alignment from the slow ones.
bool Timing::Estimate(const PoseHist &poses, const X4 &x, const std::vector<V3> &S, const std::vector<V3> &Z, double cur,
                      bool wide, double &best, int &n) {
  n = 0;
  if (rec_.empty() || S.size() < 2) return false;
  double lo = wide ? -0.04 : cur - 0.015, hi = wide ? 0.10 : cur + 0.015, step = wide ? 0.0025 : 0.001;
  std::vector<const Rec *> fast;
  std::vector<V3> so, sd;
  double newest = rec_.back().tg;
  for (auto &r : rec_) {
    if (r.tg < newest - 120) continue;
    double w;
    M3 R; V3 p;
    if (!poses.Speed(r.tg - cur, w)) continue;
    if (w < 10) {
      if (!poses.Still(r.tg - cur) && poses.At(r.tg - cur, R, p)) { so.push_back(p + R * r.od); sd.push_back(R * r.dd); }
      continue;
    }
    if (w < 15 || w > 150) continue;
    if (!poses.At(r.tg - lo, R, p) || !poses.At(r.tg - hi, R, p)) continue;
    fast.push_back(&r);
  }
  X4 xs = x;
  if (!RefitSlow(xs, S, Z, so, sd)) return false;
  std::vector<V3> P, Zq;
  Solver::Predict(xs, S, Z, P, Zq);
  // only sightings of a base station count: other lights would pad n and flatten the error curve. kNearDeg is wide
  // enough for any timing in the search at 150 deg/s
  std::vector<const Rec *> near;
  for (const Rec *r : fast) {
    M3 R; V3 p;
    if (!poses.At(r->tg - cur, R, p)) continue;
    int k; double a;
    Solver::Nearest(P, Zq, p + R * r->od, R * r->dd, k, a);
    if (std::isfinite(a) && a < kNearDeg) near.push_back(r);
  }
  fast.swap(near);
  n = (int)fast.size();
  // the first (wide) estimate wants plenty of turning
  if (n < (wide ? kTimingFirstN : kTimingNextN)) return false;
  std::vector<double> es, cs;
  const double s2 = 0.3 * 0.3;
  for (double e = lo; e <= hi + 1e-9; e += step) {
    double c = 0;
    for (const Rec *r : fast) {
      M3 R; V3 p;
      if (!poses.At(r->tg - e, R, p)) { c += 1; continue; }
      int k; double a;
      Solver::Nearest(P, Zq, p + R * r->od, R * r->dd, k, a);
      c += std::isfinite(a) ? a * a / (a * a + s2) : 1;
    }
    es.push_back(e);
    cs.push_back(c / n);
  }
  size_t i = std::min_element(cs.begin(), cs.end()) - cs.begin();
  if (i == 0 || i + 1 == cs.size()) return false;  // at the edge of the search: no clear minimum
  double c0 = cs[i - 1], c1 = cs[i], c2 = cs[i + 1], den = c0 - 2 * c1 + c2;
  double sh = den > 0 ? 0.5 * (c0 - c2) / den : 0;
  best = es[i] + std::max(-1.0, std::min(1.0, sh)) * step;
  // a real minimum stands out of the error floor; a flat curve (no fast sightings of stations) doesn't
  double mx = *std::max_element(cs.begin(), cs.end());
  return mx - c1 > 0.05 * mx;
}

// ================================================================ the app
// The solver's recent sightings were placed with the old timing: place them again with the new one. Otherwise the
// next fit still leans on the old timing, and the next estimate (made against that fit) is pulled back toward it.
void Sync::Retime(double old_e, double new_e) {
  const auto &rec = timing_.recs();
  if (rec.empty()) return;
  std::vector<double> T, G;
  std::vector<V3> O, D;
  std::vector<int> C;
  std::vector<char> B;
  for (auto &r : rec) {
    double t = r.tg - new_e, w;
    M3 R;
    V3 p;
    if (!poses_.At(t, R, p) || !poses_.Speed(t, w) || poses_.Still(t) || w > kWmax) continue;
    T.push_back(t);
    O.push_back(p + R * r.od);
    D.push_back(R * r.dd);
    G.push_back(r.hg);
    C.push_back(r.cam);
    B.push_back(r.bright);
  }
  solver_.Replace(rec.front().tg - old_e, T, O, D, G, C, B);
}

static std::string Join(const std::string &dir, const char *name) {
  if (dir.empty()) return name;
  char e = dir.back();
  return dir + ((e == '\\' || e == '/') ? "" : "\\") + name;
}

static StationsFile LoadStations(const std::string &dir) {
  StationsFile f;
  f.Load(Join(dir, "stations.json"));
  return f;
}

Sync::Sync(SyncConfig cfg, LogFn log)
    : cfg_(std::move(cfg)), log_(std::move(log)), sfile_(LoadStations(cfg_.dir)), frame_(sfile_, log_),
      solver_(log_, 1, sfile_.autogen ? std::map<std::string, V3>() : sfile_.fix) {
  if (cfg_.arrival_clock) expo_ = kExpoArrival;
  solver_.SetBody([this](const X4 &x) { return BodyDist(x); });
  solver_.SetMotion([this](const X4 &x, double &moved) { return BodyMotion(x, moved); });
  solver_.SetInImage([this](int cam, V3 d) {
    std::lock_guard<std::mutex> g(net_);
    return have_optics_ && optics_.InImage(cam, d, 8.0);
  });
  if (sfile_.loaded)
    log_(Fmt("reference frame: anchor %s%s, %d measured station(s)%s", sfile_.anchor.c_str(),
             sfile_.autogen ? " (automatic)" : "", (int)sfile_.fix.size(),
             frame_.layout() ? Fmt(", the fit of %d stations", (int)sfile_.ref.size()).c_str() : ""));
  else
    log_("no stations.json yet: the reference frame is set the first time base stations show up");
  LoadState();
}

Sync::~Sync() {}

void Sync::LoadState() {
  seeded_ = true;
  std::string text;
  JVal d;
  if (!ReadFile(Join(cfg_.dir, "state.json"), text) || !JParse(text, d)) return;
  if (const JVal *q = d.get("quest_stations"))
    for (auto &kv : q->o) {
      auto v = kv.second.nums();
      if (v.size() >= 3) quest_[kv.first] = {v[0], v[1], v[2]};
    }
  if (const JVal *t = d.get("timing"))
    for (auto &kv : t->o)
      if (kv.second.t == JVal::Num) timings_[kv.first] = kv.second.n;
  seeded_ = quest_.empty();
}

void Sync::SaveState(const X4 &x, const std::map<std::string, V3> &cs) {
  bool moved = !has_saved_ || std::fabs(x[0] - saved_x_[0]) * kDeg > 0.01 || cs.size() != saved_cs_.size() ||
               timings_.size() != saved_timings_.size();
  for (int i = 1; i < 4; i++) moved = moved || std::fabs(x[i] - saved_x_[i]) > 0.001;
  for (auto &kv : cs) {
    auto it = saved_cs_.find(kv.first);
    moved = moved || it == saved_cs_.end() || norm(kv.second - it->second) > 0.001;
  }
  for (auto &kv : timings_) {
    auto it = saved_timings_.find(kv.first);
    moved = moved || it == saved_timings_.end() || std::fabs(kv.second - it->second) > 0.0001;
  }
  if (!moved) return;  // under 0.01 deg / 1 mm / 0.1 ms: the file already has it
  saved_x_ = x;
  saved_cs_ = cs;
  saved_timings_ = timings_;
  has_saved_ = true;
  std::string s = "{\n \"quest_stations\": {";
  bool first = true;
  M3 R = Ry(x[0]);
  for (auto &kv : cs) {
    V3 q = R * kv.second + V3{x[1], x[2], x[3]};
    s += Fmt("%s\n  \"%s\": [%.5f, %.5f, %.5f]", first ? "" : ",", kv.first.c_str(), q.x, q.y, q.z);
    first = false;
  }
  s += Fmt("\n },\n \"x\": [%.9f, %.9f, %.9f, %.9f],\n \"timing\": {", x[0], x[1], x[2], x[3]);
  first = true;
  for (auto &kv : timings_) { s += Fmt("%s\n  \"%s\": %.5f", first ? "" : ",", kv.first.c_str(), kv.second); first = false; }
  s += Fmt("\n },\n \"saved\": \"%s\"\n}\n", Now().c_str());
  WriteFileAtomic(Join(cfg_.dir, "state.json"), s);
}

// 4-DOF fit raw -> Quest from the stations both know
bool Sync::Seed(const std::map<std::string, V3> &raw, X4 &x, double &miss) {
  std::vector<V3> S, Q;
  for (auto &kv : raw) {
    auto it = quest_.find(kv.first);
    if (it != quest_.end()) { S.push_back(kv.second); Q.push_back(it->second); }
  }
  if (S.size() < 2) return false;
  x = YawFit(S, Q);
  V3 t{x[1], x[2], x[3]};
  miss = 0;
  for (size_t i = 0; i < S.size(); i++) miss = std::max(miss, norm(Ry(x[0]) * S[i] + t - Q[i]));
  return true;
}

bool Sync::SetCalibration(const std::string &json, std::string *err) {
  std::lock_guard<std::mutex> g(net_);
  if (!optics_.Load(json, err)) return false;
  have_optics_ = true;
  // a Quest's frame period is one of two, the Frame's is learned from its frames
  grid_quest_ = !optics_.exact_time() && !cfg_.learn_grid;
  grid_ = FrameGrid(grid_quest_);
  grid_logged_.clear();
  return true;
}

void Sync::HeadsetReset() {
  std::lock_guard<std::mutex> g(net_);
  clock_.Clear();
  grid_ = FrameGrid(grid_quest_);
  grid_logged_.clear();
}

void Sync::SetStreamer(const std::string &system) {
  std::lock_guard<std::mutex> g(st_m_);
  if (system == streamer_) return;
  streamer_ = system;
  auto it = timings_.find(system);
  if (it != timings_.end() && cfg_.learn_timing) {
    expo_ = it->second;
    timing_learned_ = true;
    log_(Fmt("timing for %s: %.1f ms (learned before)", system.c_str(), it->second * 1000));
  }
}

void Sync::OnHmdPose(double t, const Quat &q, V3 p) {
  poses_.Add(t, q, p);
  if (recording_ && t - last_prec_ >= 0.004) {  // recordings: the HMD at ~250 Hz like the old poller, 3x4 row-major
    last_prec_ = t;
    M3 R = ToM3(q);
    Rec(t, "P 0 %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f", R.m[0][0], R.m[0][1], R.m[0][2], p.x,
        R.m[1][0], R.m[1][1], R.m[1][2], p.y, R.m[2][0], R.m[2][1], R.m[2][2], p.z);
  }
}

void Sync::OnBodyPose(int dev, double t, V3 raw) {
  std::lock_guard<std::mutex> g(body_m_);
  double &last = body_last_[dev];
  if (t - last < 0.1) return;  // 10 Hz is plenty
  last = t;
  body_.push_back({t, dev, raw});
  if (recording_) body_new_.push_back({t, dev, raw});
  while (!body_.empty() && body_.front().t < t - Solver::ACQ_WIN) body_.pop_front();
}

// the median, over 0.1 s steps, of the nearest worn or held lighthouse device's horizontal distance to the head, were x
// the alignment (Tick's thread: frame_ is stable here)
double Sync::BodyDist(const X4 &x) {
  std::vector<BodyS> b;
  {
    std::lock_guard<std::mutex> g(body_m_);
    b.assign(body_.begin(), body_.end());
  }
  M3 C;
  V3 tc;
  frame_.Rotation(C, tc);
  double c = std::cos(x[0]), s = std::sin(x[0]);
  std::map<long long, double> nearest;
  for (const BodyS &e : b) {
    M3 R;
    V3 h;
    if (!poses_.At(e.t, R, h)) continue;
    V3 q = RyMul(c, s, C * e.p + tc) + V3{x[1], x[2], x[3]};
    double d = std::hypot(q.x - h.x, q.z - h.z);
    auto it = nearest.emplace((long long)std::floor(e.t * 10), d).first;
    it->second = std::min(it->second, d);
  }
  if (nearest.size() < 20) return NAN;
  std::vector<double> v;
  for (auto &kv : nearest) v.push_back(kv.second);
  return Median(v);
}

// How the worn or held lighthouse devices' horizontal motion follows the head's, were x the alignment: the
// correlation of their steps (about 0.3 s each) with the head's over the same times, all devices together. +1 every
// step along, -1 every step against; devices left lying add nothing. moved: the head's own path meanwhile (m).
double Sync::BodyMotion(const X4 &x, double &moved) {
  std::vector<BodyS> b;
  {
    std::lock_guard<std::mutex> g(body_m_);
    b.assign(body_.begin(), body_.end());
  }
  moved = 0;
  if (b.size() < 20) return NAN;
  M3 C;
  V3 tc;
  frame_.Rotation(C, tc);
  double c = std::cos(x[0]), s = std::sin(x[0]), num = 0, nh = 0, nd = 0;
  std::map<int, std::vector<const BodyS *>> per;
  for (const BodyS &e : b) per[e.dev].push_back(&e);
  for (auto &kv : per) {
    const std::vector<const BodyS *> &v = kv.second;
    for (size_t i = 0, j = 0; i < v.size(); i = j) {
      for (j = i + 1; j < v.size() && v[j]->t - v[i]->t < 0.25; j++) {}
      if (j >= v.size()) break;
      if (v[j]->t - v[i]->t > 0.5) continue;
      M3 R;
      V3 h0, h1;
      if (!poses_.At(v[i]->t, R, h0) || !poses_.At(v[j]->t, R, h1)) continue;
      V3 dq = RyMul(c, s, C * (v[j]->p - v[i]->p)), dh = h1 - h0;
      num += dh.x * dq.x + dh.z * dq.z;
      nh += dh.x * dh.x + dh.z * dh.z;
      nd += dq.x * dq.x + dq.z * dq.z;
    }
  }
  M3 R;
  V3 h0, h1;
  for (double t = b.front().t; t + 0.25 <= b.back().t; t += 0.25)
    if (poses_.At(t, R, h0) && poses_.At(t + 0.25, R, h1)) moved += std::hypot(h1.x - h0.x, h1.z - h0.z);
  return nh > 0 && nd > 0 ? num / std::sqrt(nh * nd) : NAN;
}

void Sync::SetRecord(FILE *f) {
  std::lock_guard<std::mutex> g(rec_m_);
  if (rec_) fclose(rec_);
  rec_ = f;
  recording_ = f != nullptr;
}

void Sync::Rec(double t, const char *fmt, ...) {
  if (!recording_) return;
  std::lock_guard<std::mutex> g(rec_m_);
  if (!rec_) return;
  fprintf(rec_, "%lld ", (long long)std::llround(t * 1e9));
  va_list ap;
  va_start(ap, fmt);
  vfprintf(rec_, fmt, ap);
  va_end(ap);
  fputc('\n', rec_);
}

FILE *g_ray_dump = nullptr;

// one lhsight line: "F cam k t_us mean nblobs [x y npx peak]..." (x, y in pixels * 10)
void Sync::OnLine(double pc, const char *line) {
  if (line[0] != 'F' || line[1] != ' ') return;
  Rec(pc, "%s", line);
  int cam, nb;
  unsigned k;
  long long tus;
  int mean, off = 0;
  if (sscanf(line, "F %d %u %lld %d %d%n", &cam, &k, &tus, &mean, &nb, &off) < 5) return;
  nframes_++;
  std::lock_guard<std::mutex> g(net_);
  double hs = tus / 1e6;
  if (cfg_.arrival_clock) clock_.AddArrival(pc, hs);
  double lag = 0;
  // the Frame stamps its frames itself; the Quest's are when lhsight found them, the grid takes the poll lag off. On
  // the Frame the grid still learns the frame period, for the channel check
  bool grid_lag = nb >= 0 && grid_.Lag(cam, hs, lag);
  if (optics_.exact_time()) lag = 0;
  bool have_lag = nb >= 0 && (optics_.exact_time() || grid_lag);
  if (nb >= 0) {  // the frame period: log it when it's found or moves
    double p = grid_.period(cam), &was = grid_logged_[cam];
    if (p > 0 && std::fabs(p - was) > 0.0001) {
      log_(Fmt("camera %d: short frames every %.3f ms", cam, p * 1000));
      was = p;
    }
  }
  if (nb < 0) return;
  struct Blob { int x10, y10, npx, peak; };
  std::vector<Blob> bl;
  {
    const char *s = line + off;
    for (int i = 0; i < nb; i++) {
      int x10, y10, npx, peak, used = 0;
      if (sscanf(s, " %d %d %d %d%n", &x10, &y10, &npx, &peak, &used) < 4) break;
      s += used;
      bl.push_back({x10, y10, npx, peak});
    }
  }
  nb = (int)bl.size();
  spots_.frames++;
  spots_.seen += nb;
  if (!have_optics_) { spots_.other += nb; return; }
  double hg = have_lag ? hs - lag : std::numeric_limits<double>::quiet_NaN();  // frame time, headset clock
  if (have_lag && grid_.period(cam) > 0) solver_.SetFramePeriod(grid_.period(cam));
  double grid_pc;
  if (!clock_.Map(hs - (have_lag ? lag : 0.0), grid_pc)) { spots_.other += nb; return; }
  double expo = expo_;
  double t = grid_pc - expo - (have_lag ? 0.0 : kLagFallback);
  M3 R; V3 p;
  double w;
  if (!poses_.At(t, R, p) || !poses_.Speed(t, w)) { spots_.other += nb; return; }
  bool still = poses_.Still(t);
  {  // while SteamVR's headset stands still, do the lamps and windows the cameras see stand still too?
    std::vector<std::pair<double, double>> big;
    for (const Blob &b : bl)
      if (b.npx >= kLampPx) big.push_back({b.x10 / 10.0, b.y10 / 10.0});
    auto &prev = prev_big_[cam];
    if (!still) img_moved_ = img_still_ = 0;
    else if (!prev.empty())
      for (auto &q : big) {
        double d = 1e9;
        for (auto &o : prev) d = std::min(d, std::hypot(q.first - o.first, q.second - o.second));
        img_still_ += d <= 3;
        img_moved_ += d > 10;
      }
    prev = big;
  }
  if (still) { spots_.still += nb; return; }
  double wmax = (cfg_.learn_timing && !timing_learned_) ? kWmaxUntimed : kWmax;
  M3 Rc;
  V3 tc;
  if (have_lag && w <= wmax && optics_.Pose(cam, Rc, tc)) solver_.AddFrame(t, hg, cam, p + R * tc, R * Rc);
  for (const Blob &b : bl) {
    int x10 = b.x10, y10 = b.y10, npx = b.npx;
    if (npx > kMaxPx) { spots_.big++; continue; }
    V3 o, d;
    if (!optics_.Ray(cam, x10 / 10.0 - 0.5, y10 / 10.0 - 0.5, o, d)) { spots_.other++; continue; }
    if (have_lag && cfg_.learn_timing) timing_.Record(grid_pc, o, d, hg, cam, b.peak >= kBright);
    if (g_ray_dump) {
      V3 O = p + R * o, D = R * d;
      fprintf(g_ray_dump, "R %.6f %d %.6f %.6f %.6f %.7f %.7f %.7f %.6f %.6f %.6f %.1f\n", t, cam, O.x, O.y, O.z, D.x, D.y,
              D.z, p.x, p.y, p.z, w);
    }
    if (w > wmax) { spots_.fast++; continue; }
    bool bright = b.peak >= kBright;
    solver_.Add(t, p + R * o, R * d, hg, cam, bright);  // dim ones too: they tell when the room is too light
    if (bright || solver_.starved()) spots_.used++;
    else spots_.dim++;
  }
}

std::string Sync::Describe(const Spots &a, const Spots &b) {
  long n = b.seen - a.seen;
  if (!n) return Fmt("no bright spots on %ld camera frames", b.frames - a.frames);
  std::string why;
  auto part = [&](long k, const char *what) { if (k > 0) why += Fmt("%s%ld %s", why.empty() ? "" : ", ", k, what); };
  part(b.still - a.still, "while the headset was still");
  part(b.fast - a.fast, "while it turned fast");
  part(b.big - a.big, "lamps or windows");
  part(b.other - a.other, "without a headset pose");
  part(b.dim - a.dim, "dim, kept for a room too light for the dots to saturate");
  return Fmt("%ld bright spots, %ld used", n, b.used - a.used) + (why.empty() ? "" : " (" + why + ")");
}

void Sync::SetStationsRaw(const std::map<std::string, std::pair<V3, M3>> &raw) {
  std::map<std::string, std::pair<V3, M3>> ok;  // a rotation, finite: anything else would turn the frame to NaN
  for (auto &kv : raw) {
    const M3 &R = kv.second.second;
    V3 p = kv.second.first;
    bool good = std::isfinite(p.x + p.y + p.z);
    for (int c = 0; c < 3 && good; c++) {
      double n = R.m[0][c] * R.m[0][c] + R.m[1][c] * R.m[1][c] + R.m[2][c] * R.m[2][c];
      good = std::isfinite(n) && std::fabs(n - 1) < 0.01;
    }
    if (good) ok.insert(kv);
  }
  std::lock_guard<std::mutex> g(raw_m_);
  raw_ = ok;
}

void Sync::SetChannels(const std::map<std::string, int> &ch) {
  std::lock_guard<std::mutex> g(raw_m_);
  channels_ = ch;
}

// One or two base stations leave the level to SteamVR, and the cameras can't fix it (gravity.h); with three, an
// automatic frame is levelled by the cameras (LevelStep)
bool Sync::GravityFrame(M3 &C, M3 &turn, std::string &key) {
  if (!sfile_.loaded || frame_.ok() != 1) return false;
  size_t n;
  {
    std::lock_guard<std::mutex> g(raw_m_);
    n = raw_.size();
  }
  if (sfile_.autogen && n >= (size_t)Solver::LEVEL_STATIONS) return false;
  C = frame_.Anchored();
  V3 t;
  frame_.Rotation(turn, t);
  turn = turn * T(C);
  Quat q = ToQuat(sfile_.anchor_R);
  key = Fmt("%s %.5f %.5f %.5f %.5f", sfile_.anchor.c_str(), q.w, q.x, q.y, q.z);
  return true;
}

// An automatic reference frame is SteamVR's frame as it was when the anchor was first seen, tilt and all. The headset
// knows gravity: level the frame by it whenever the sightings pin its tilt (Solver::Level), and keep the level in
// stations.json. A measured frame is the user's own and stays as it is. A wrong alignment (two stations swapped) can
// be levelled into fitting three stations too, so only a settled one whose worn or held devices sit at the head counts.
void Sync::LevelStep(const std::map<std::string, std::pair<V3, M3>> &raw, double now) {
  if (!sfile_.autogen || frame_.ok() != 1 || !solver_.has_x() || now - lock_time_ < kLevelSettle) return;
  if (!(BodyDist(solver_.x()) < kLevelBody)) return;
  Solver::LevelR lv;
  if (!solver_.Level(sfile_.anchor_p, lv)) return;
  double step = RotDeg(lv.tilt);
  M3 L = lv.tilt * sfile_.level;
  if (step < 0.05 || RotDeg(L) > kMaxRot) return;  // within the noise; or past what a level can be
  sfile_.level = L;
  sfile_.SaveAuto(Join(cfg_.dir, "stations.json"));
  frame_.Update(raw);
  for (auto &kv : raw) {
    V3 p; M3 R;
    frame_.Apply(kv.second.first, kv.second.second, p, R);
    solver_.SetStation(kv.first, p, R);
  }
  solver_.Relevel(lv.x);
  log_(Fmt("lighthouse frame levelled with the headset: tilted %.2f deg (%.2f deg in all), %d base stations within "
           "%.2f deg", step, RotDeg(L), lv.stations, lv.med));
}

// An automatic frame takes its stations' poses in it once three show up (Frame::Layout follows them from then on), adds
// new ones as they come, and moves a station's place when the fit has left it out for kLayoutMoved s (it was moved).
// When no fit gets within kMoved for kLayoutMoved s, more than one was moved: the frame starts over from SteamVR's.
void Sync::LayoutStep(const std::map<std::string, std::pair<V3, M3>> &raw, double now) {
  if (!sfile_.autogen || frame_.ok() != 1) return;
  bool was = frame_.layout();
  if (!was && raw.size() < 3) return;
  if (!was || frame_.miss() <= kMoved) bad_since_ = -1;
  else if (bad_since_ < 0) bad_since_ = now;
  if (bad_since_ >= 0 && now - bad_since_ >= kLayoutMoved && raw.size() >= 3) {
    sfile_.ref.clear();
    sfile_.ref.insert(raw.begin(), raw.end());
    sfile_.level = M3();  // SteamVR's frame as it is now: its level is found again
    sfile_.SaveAuto(Join(cfg_.dir, "stations.json"));
    bad_since_ = -1;
    log_(Fmt("lighthouse frame: base stations moved (%.0f cm off the fit): reference frame reset",
             frame_.miss() * 100));
    frame_.Update(raw);
    return;
  }
  std::string left = frame_.left();
  if (left != left_seen_) { left_seen_ = left; left_since_ = now; }
  int n = 0;
  for (auto &kv : raw) {
    bool known = sfile_.ref.count(kv.first) > 0;
    bool moved = known && kv.first == left && now - left_since_ >= kLayoutMoved;
    if (known && !moved) continue;
    V3 p; M3 R;
    frame_.Unlevelled(kv.second.first, kv.second.second, p, R);
    sfile_.ref[kv.first] = {p, R};
    n++;
    if (was)
      log_(Fmt("lighthouse frame: base station %s %s", kv.first.c_str(),
               moved ? "was moved: its new place taken" : "added to the fit"));
  }
  if (!n) return;
  if (!was) log_(Fmt("lighthouse frame: follows the fit of %d base stations from now on", n));
  sfile_.SaveAuto(Join(cfg_.dir, "stations.json"));
  frame_.Update(raw);
}

// Two stations: the level the cameras see across them, against gravity's (gravity.h) about the same axis. Logged every
// few minutes (and recorded), so the two can be compared; nothing is applied from it.
void Sync::LevelCheck(double now) {
  std::vector<std::string> keys;
  std::vector<V3> S, Z;
  solver_.Stations(keys, S, Z);
  if (S.size() != 2) return;  // with three, LevelStep levels the frame by the cameras themselves
  // a measured station stays where the cameras put it, gravity's levelling or not: they'd only check themselves
  if (solver_.measured(keys[0]) || solver_.measured(keys[1])) return;
  V3 b = S[1] - S[0];
  double h = std::hypot(b.x, b.z);
  if (h < 1) return;
  V3 axis{-b.z / h, 0, b.x / h}, pivot = (S[0] + S[1]) * 0.5;
  Solver::AxisR a;
  if (!solver_.LevelAxis(pivot, axis, a)) return;
  // gravity's tilt in the frame now, about the same axis: the cameras' view of the frame before it is the two together
  const M3 &G = frame_.gravity();
  V3 rg{G.m[2][1] - G.m[1][2], G.m[0][2] - G.m[2][0], G.m[1][0] - G.m[0][1]};
  double g = dot(rg, axis) * 0.5, cam = a.theta + g;
  Rec(now, "I level %.5f %.5f %.5f %.4f", cam * kDeg, a.sd * kDeg, g * kDeg, a.med);
  if (now - check_said_ < 180) return;
  check_said_ = now;
  log_(Fmt("level check across %s and %s: the cameras see the frame %+.2f deg off level (within %.2f) before "
           "gravity's levelling, which turns it %+.2f deg about the same axis%s", keys[0].c_str(), keys[1].c_str(),
           cam * kDeg, a.sd * kDeg, g * kDeg, std::fabs(a.theta) > 3 * a.sd ? "" : ": they agree"));
}

// two stations have 10 sightings that fit
static bool Locked(bool has_x, const StepStat &st) {
  if (!has_x || !st.has_per) return false;
  std::vector<int> c;
  for (auto &kv : st.per) c.push_back(kv.second);
  std::sort(c.begin(), c.end());
  return c.size() >= 2 && c[c.size() - 2] >= 10;
}

Transform Sync::Tick(double now) {
  if (now - last_step_ >= 1.0) {
    last_step_ = now;
    int b = poses_.brk_id();
    if (b != brk_seen_) {
      brk_seen_ = b;
      solver_.PoseBreak(poses_.brk_t());
      log_("HMD pose break (" + poses_.brk_what() + "): older sightings dropped, acquiring");
    }
    std::map<std::string, std::pair<V3, M3>> raw;
    std::map<std::string, int> chans;
    {
      std::lock_guard<std::mutex> g(raw_m_);
      raw = raw_;
      chans = channels_;
    }
    for (auto &kv : chans) solver_.SetChannel(kv.first, kv.second);
    if (!sfile_.loaded && !raw.empty()) {  // first run: pin the reference frame to the station at SteamVR's origin
      auto best = raw.begin();
      for (auto it = raw.begin(); it != raw.end(); ++it)
        if (norm(it->second.first) < norm(best->second.first)) best = it;
      sfile_.loaded = sfile_.autogen = true;
      sfile_.anchor = best->first;
      sfile_.anchor_p = best->second.first;
      sfile_.anchor_R = best->second.second;
      sfile_.SaveAuto(Join(cfg_.dir, "stations.json"));
      log_("reference frame set: anchor " + best->first);
    } else if (sfile_.autogen && !frame_.layout() && raw.count(sfile_.anchor)) {  // follows a moved anchor
      auto &a = raw[sfile_.anchor];
      if (RotDeg(sfile_.anchor_R * T(a.second)) > kMaxRot || norm(a.first - sfile_.anchor_p) > kMoved) {
        sfile_.anchor_p = a.first;
        sfile_.anchor_R = a.second;
        sfile_.level = M3();  // SteamVR's frame as it is now: its level is found again
        sfile_.SaveAuto(Join(cfg_.dir, "stations.json"));
        log_("anchor " + sfile_.anchor + " moved: reference frame reset");
      }
    }
    frame_.Update(raw);
    LayoutStep(raw, now);
    std::map<std::string, V3> cs;
    for (auto &kv : raw) {
      V3 p; M3 R;
      frame_.Apply(kv.second.first, kv.second.second, p, R);
      solver_.SetStation(kv.first, p, R);
      if (now - last_s_ >= 10) {
        frame_.Unlevelled(kv.second.first, kv.second.second, p, R);
        Quat q = ToQuat(R);
        auto c = chans.find(kv.first);
        Rec(now, "S %s %.5f %.5f %.5f %.5f %.5f %.5f %.5f %d", kv.first.c_str(), p.x, p.y, p.z, q.w, q.x, q.y, q.z,
            c == chans.end() ? 0 : c->second);
      }
    }
    if (now - last_s_ >= 10 && !raw.empty()) last_s_ = now;
    if (recording_) {  // worn and held lighthouse devices, in the reference frame like the stations (before the level)
      std::deque<BodyS> nb;
      {
        std::lock_guard<std::mutex> g(body_m_);
        nb.swap(body_new_);
      }
      for (const BodyS &e : nb) {
        V3 p;
        M3 R;
        frame_.Unlevelled(e.p, M3(), p, R);
        Rec(e.t, "B %d %.4f %.4f %.4f", e.dev, p.x, p.y, p.z);
      }
    }
    {
      std::vector<std::string> keys;
      std::vector<V3> S, Z;
      solver_.Stations(keys, S, Z);
      for (size_t i = 0; i < keys.size(); i++)
        if (raw.count(keys[i])) cs[keys[i]] = S[i];
    }
    if (!seeded_ && cs.size() >= 2) {
      seeded_ = true;
      X4 x;
      double miss;
      if (Seed(cs, x, miss) && miss < 0.05) {
        solver_.Reset(x);
        log_(Fmt("seeded from saved stations: yaw %+.3f t [%.4f, %.4f, %.4f] miss %.1f cm", x[0] * kDeg, x[1], x[2], x[3], miss * 100));
      } else {
        log_("saved stations don't fit: acquiring");
      }
    }
    StepStat st = solver_.Step(now);
    if (solver_.resets() != resets_seen_) { resets_seen_ = solver_.resets(); lock_time_ = now; }
    {
      std::lock_guard<std::mutex> g(st_m_);
      last_st_ = st;
      has_last_st_ = true;
    }
    // not acquired: what became of the cameras' bright spots, after a minute and then every 5
    if (Locked(solver_.has_x(), st)) sum_t_ = -1;
    else {
      Spots sp = spots();
      if (sum_t_ < 0) { sum_t_ = now; sum_sp_ = sp; sum_n_ = 0; }
      else if (now - sum_t_ >= (sum_n_ ? 300 : 60)) {
        if (sp.frames > sum_sp_.frames) log_(Fmt("acquiring, last %.0f s: ", now - sum_t_) + Describe(sum_sp_, sp));
        sum_t_ = now; sum_sp_ = sp; sum_n_++;
      }
    }
    X4 x = solver_.x();
    if (recording_) {
      std::string per;
      for (auto &kv : st.per) per += Fmt("%s'%s': %d", per.empty() ? "{" : ", ", kv.first.c_str(), kv.second);
      per = st.has_per ? per + "}" : "None";
      Rec(now, "A %s | %s | %d %s %s%s%s", have_applied_ ? Fmt("%+.4f %.4f %.4f %.4f", applied_[0] * kDeg, applied_[1], applied_[2], applied_[3]).c_str() : "-",
          solver_.has_x() ? Fmt("%+.4f %.4f %.4f %.4f", x[0] * kDeg, x[1], x[2], x[3]).c_str() : "-", st.n,
          st.has_med ? Fmt("%.4f", st.med).c_str() : "None", per.c_str(), st.cond ? " cond" : "", st.jump ? " JUMP" : "");
    }
    if (Locked(solver_.has_x(), st) && st.cond && st.has_med && st.med < 0.3 && now - last_save_ > 10) {
      last_save_ = now;
      SaveState(x, cs);
    }
    // timing: every 15 s on a well-pinned alignment
    if (cfg_.learn_timing && solver_.has_x() && st.cond && st.has_med && st.med < 0.3 && now - last_timing_ > 15) {
      last_timing_ = now;
      std::vector<std::string> keys;
      std::vector<V3> S, Z;
      solver_.Stations(keys, S, Z);
      double best;
      int n;
      bool ok;
      double cur = expo_;
      {
        std::lock_guard<std::mutex> g(net_);
        ok = timing_.Estimate(poses_, x, S, Z, cur, !timing_learned_, best, n);
      }
      if (ok) {
        // the step trusts the estimate by how much turning it saw
        double e = timing_learned_ ? cur + std::min(1.0, (double)n / kTimingFullN) * (best - cur) : best;
        e = std::max(-0.05, std::min(0.12, e));
        expo_ = e;
        if (std::fabs(e - cur) > 1e-4) {
          std::lock_guard<std::mutex> g(net_);
          Retime(cur, e);
        }
        bool first = !timing_learned_;
        timing_learned_ = true;
        std::string sys;
        {
          std::lock_guard<std::mutex> g(st_m_);
          sys = streamer_;
        }
        if (!sys.empty()) timings_[sys] = e;
        if (first || std::fabs(best - cur) > 0.002)
          log_(Fmt("timing%s%s: %.1f ms (best fit %.1f ms on %d fast-head sightings)", sys.empty() ? "" : " for ",
                   sys.c_str(), e * 1000, best * 1000, n));
        Rec(now, "I timing %.5f best %.5f n %d", e, best, n);
      }
    }
    if (now - last_level_ >= 10) {
      last_level_ = now;
      LevelStep(raw, now);
    }
    if (now - last_check_ >= 60 && Locked(solver_.has_x(), st) && st.cond) {
      last_check_ = now;
      LevelCheck(now);
    }
  }
  // slew toward the solution, faster while the head turns or walks (kSlew*), jump if far (acquisition)
  Transform out;
  if (!solver_.has_x()) return out;
  X4 x = solver_.x();
  if (!have_applied_) { applied_ = x; have_applied_ = true; }
  else if (!paused_) {
    V3 h{0, 1.5, 0};
    double th, w = 0, v = 0;
    if (poses_.Latest(h, &th)) {
      M3 R0;
      V3 p0;
      if (!poses_.Speed(th, w)) w = 0;
      if (poses_.At(th - 0.1, R0, p0)) v = norm(h - p0) / 0.1;
    }
    X4 dx = {Wrap(x[0] - applied_[0]), x[1] - applied_[1], x[2] - applied_[2], x[3] - applied_[3]};
    V3 at{applied_[1], applied_[2], applied_[3]};
    double d = norm(Ry(dx[0]) * (h - at) + at + V3{dx[1], dx[2], dx[3]} - h);
    if (d > kJumpApply) applied_ = x;
    else if (d > 1e-6) {
      double s = std::min(1.0, (kSlewStill + kSlewTurn * w + kSlewWalk * v) * 0.05 / d);
      for (int i = 0; i < 4; i++) applied_[i] += dx[i] * s;
    }
  }
  M3 C; V3 tc;
  frame_.Rotation(C, tc);
  M3 R = Ry(applied_[0]);
  out.active = true;
  out.q = ToQuat(R * C);
  out.t = R * tc + V3{applied_[1], applied_[2], applied_[3]};
  return out;
}

Sync::Status Sync::GetStatus(double now) {
  Status s;
  Spots sp = spots();
  if (now - rate_t_ >= 2.0) {
    long f = nframes_;
    if (rate_t_ > 0) {
      double dt = now - rate_t_;
      cam_fps_ = (f - rate_f_) / dt;
      spot_rate_ = (sp.seen - rate_sp_.seen) / dt;
      sight_rate_ = (sp.used - rate_sp_.used) / dt;
    }
    rate_t_ = now; rate_f_ = f; rate_sp_ = sp;
  }
  s.cam_fps = cam_fps_;
  s.spot_rate = spot_rate_;
  s.sight_rate = sight_rate_;
  double th;
  V3 h;
  s.head_still = poses_.Latest(h, &th) && poses_.Still(th);
  // still for kFrozen s while the lamps and windows the cameras see keep moving: SteamVR's pose isn't the head's
  if (!s.head_still || !(cam_fps_ > 5)) still_since_ = -1;
  else if (still_since_ < 0) still_since_ = now;
  long moved, stayed;
  {
    std::lock_guard<std::mutex> g(net_);
    moved = img_moved_;
    stayed = img_still_;
  }
  bool frozen = still_since_ >= 0 && now - still_since_ >= kFrozen && moved >= 20 && moved > 3 * stayed;
  if (frozen) {
    s.head_still = 2;
    if (!frozen_said_)
      log_(Fmt("SteamVR's headset pose hasn't moved for %.0f s while the cameras see the room move (%ld lamp or window "
               "spots moved, %ld stayed): SteamVR isn't getting the head's motion. Is its view in the headset?",
               now - still_since_, moved, stayed));
  }
  frozen_said_ = frozen || (frozen_said_ && still_since_ >= 0);
  s.has_x = solver_.has_x();
  s.x = solver_.x();
  s.expo = expo_;
  s.timing_learned = timing_learned_;
  {
    std::lock_guard<std::mutex> g(net_);
    s.rtt = clock_.rtt();
  }
  StepStat st;
  {
    std::lock_guard<std::mutex> g(st_m_);
    st = last_st_;
  }
  s.cond = st.cond;
  s.n = st.n;
  s.med = st.has_med ? st.med : -1;
  s.locked = Locked(s.has_x, st);
  s.locked_for = lock_time_ >= 0 ? now - lock_time_ : -1;
  if (s.has_x && have_applied_) {
    V3 h{0, 1.5, 0};
    poses_.Latest(h);
    X4 dx = {Wrap(s.x[0] - applied_[0]), s.x[1] - applied_[1], s.x[2] - applied_[2], s.x[3] - applied_[3]};
    V3 at{applied_[1], applied_[2], applied_[3]};
    s.lag_cm = norm(Ry(dx[0]) * (h - at) + at + V3{dx[1], dx[2], dx[3]} - h) * 100;
  }
  std::vector<std::string> keys;
  std::vector<V3> S, Z;
  solver_.Stations(keys, S, Z);
  s.nstations = (int)keys.size();
  V3 head;
  bool hh = poses_.Latest(head);
  for (size_t i = 0; i < keys.size(); i++) {
    Status::St e;
    e.serial = keys[i];
    e.anchor = sfile_.loaded && keys[i] == sfile_.anchor;
    e.measured = solver_.measured(keys[i]);
    auto it = st.per.find(keys[i]);
    e.support = it == st.per.end() ? 0 : it->second;
    auto lt = st.last.find(keys[i]);
    e.last_seen = lt == st.last.end() ? -1 : std::max(0.0, now - lt->second);
    if (s.has_x && hh) e.dist = norm(Ry(s.x[0]) * S[i] + V3{s.x[1], s.x[2], s.x[3]} - head);
    s.st.push_back(e);
  }
  return s;
}
