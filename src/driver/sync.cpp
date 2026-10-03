#include "sync.h"

#include <algorithm>
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
  double tu = (px - cx) / f, tv = (py - cy) / f;
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
  const JVal *cams = root.get("CameraCalibration");
  if (!cams || cams->t != JVal::Arr) { if (err) *err = "no CameraCalibration"; return false; }
  CamCal out[4];
  for (auto &c : cams->a) {
    const JVal *id = c.get("Id"), *size = c.get("ImageSize"), *dfc = c.get("DeviceFromCamera");
    const JVal *proj = c.get("Projection"), *dist = c.get("Distortion");
    if (!id || !size || !dfc || !proj || !dist) continue;
    int i = id->t == JVal::Str ? atoi(id->s.c_str()) : (int)id->num(-1);  // "Id": "2" in Meta's files
    if (i < 0 || i > 3) continue;
    auto sz = size->nums(), T = dfc->nums();
    auto pc = proj->get("Coefficients") ? proj->get("Coefficients")->nums() : std::vector<double>();
    auto dc = dist->get("Coefficients") ? dist->get("Coefficients")->nums() : std::vector<double>();
    if (sz.size() < 2 || T.size() < 16 || pc.size() < 3 || dc.size() < 8) continue;
    CamCal &k = out[i];
    k.w = (int)sz[0]; k.h = (int)sz[1];
    for (int r = 0; r < 3; r++)
      for (int q = 0; q < 3; q++) k.R.m[r][q] = T[r * 4 + q];
    k.t = {T[3], T[7], T[11]};
    k.f = pc[0]; k.cx = pc[1]; k.cy = pc[2];
    for (int j = 0; j < 6; j++) k.k[j] = dc[j];
    k.p[0] = dc[6]; k.p[1] = dc[7];
    k.valid = true;
  }
  if (!out[2].valid && !out[3].valid) { if (err) *err = "no side camera calibration (ids 2, 3)"; return false; }
  for (int i = 0; i < 4; i++) cams_[i] = out[i];
  return true;
}

bool Optics::Ray(int cam, double x, double y, V3 &o, V3 &d) const {
  if (cam < 0 || cam > 3 || !cams_[cam].valid) return false;
  const CamCal &c = cams_[cam];
  double k = (cam == 2 || cam == 3) ? kIR : 1.0;
  V3 dc;
  if (!c.Unproject(c.cx + (x - c.cx) * k, c.cy + (y - c.cy) * k, dc)) return false;
  d = c.R * dc;
  double n = norm(d);
  if (!std::isfinite(n) || n <= 0) return false;
  d = d * (1 / n);
  o = c.t;
  return true;
}

// ================================================================ frame grid, clock
static double PyMod(double a, double m) {
  double r = std::fmod(a, m);
  return r < 0 ? r + m : r;
}

bool FrameGrid::Lag(int cam, double t, double &lag) {
  auto &h = h_[cam];
  h.push_back(t);
  while (!h.empty() && h.front() < t - WIN) h.pop_front();
  if (h.size() < 40) return false;
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
  double l = PyMod(t - ph[(best + 1) % n], P);
  lag = l > P - 0.01 ? l - P : l;
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
void PoseHist::Add(double t, const Quat &q, V3 p) {
  std::lock_guard<std::mutex> g(m_);
  if (i_) {
    const PoseS &l = ring_[(i_ - 1) % N];
    double dt = t - l.t;
    if (dt <= 0) return;  // poses must move forward in time
    double dp = norm(p - l.p), da = QuatDeg(q, l.q);
    if (dt > 1.0 || dp > 0.05 + 3.0 * dt || da > 5.0 + 500.0 * dt) {
      brk_t_ = t;
      brk_what_ = Fmt("%.2f s, %.0f cm, %.1f deg", dt, dp * 100, da);
      brk_id_++;
    }
  }
  ring_[i_ % N] = {t, q, p};
  i_++;
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
  if (const JVal *st = d.get("stations"))
    for (auto &kv : st->o)
      if (const JVal *pos = kv.second.get("pos")) {
        auto v = pos->nums();
        if (v.size() >= 3) fix[kv.first] = {v[0], v[1], v[2]};
      }
  const JVal *ag = d.get("auto");
  autogen = ag && ag->t == JVal::Bool && ag->b;
  loaded = true;
  return true;
}

bool StationsFile::SaveAuto(const std::string &path) const {
  Quat q = ToQuat(anchor_R);
  std::string s = Fmt(
      "{\n \"auto\": true,\n \"note\": \"QuestLHSync's reference frame: %s's pose when it was first seen. SteamVR re-tilts "
      "its lighthouse universe at every start; this pins it. Delete to start over.\",\n \"anchor\": \"%s\",\n"
      " \"anchor_p\": [%.6f, %.6f, %.6f],\n \"anchor_q\": [%.6f, %.6f, %.6f, %.6f],\n \"stations\": {},\n \"saved\": \"%s\"\n}\n",
      anchor.c_str(), anchor.c_str(), anchor_p.x, anchor_p.y, anchor_p.z, q.w, q.x, q.y, q.z, Now().c_str());
  return WriteFileAtomic(path, s);
}

void Frame::Update(const std::map<std::string, std::pair<V3, M3>> &raw) {
  if (!f_.loaded) return;
  auto it = raw.find(f_.anchor);
  if (it == raw.end()) return;
  M3 C = f_.anchor_R * T(it->second.second);
  double ang = RotDeg(C);
  int ok = ang < kMaxRot;
  if (ok != ok_ || (ok && RotDeg(C * T(C_)) > 0.01))
    log_(Fmt("lighthouse frame: SteamVR's is %.2f deg from the reference%s", ang,
             ok ? "" : " (over 3 deg: the anchor moved or a new universe; SteamVR's frame as it is)"));
  ok_ = ok;
  deg_ = ang;
  if (ok) { C_ = C; o_ = it->second.first; oref_ = f_.anchor_p; }
  else { C_ = M3(); o_ = {}; oref_ = {}; }
}

// ================================================================ solver
Solver::Solver(LogFn log, uint64_t seed, std::map<std::string, V3> fix)
    : fix_(std::move(fix)), log_(std::move(log)), rng_(seed) {}

void Solver::Reset(const X4 &x) {
  x_ = x; has_x_ = true;
  resets_++;
  anchor_ = x; has_anchor_ = true;
}

void Solver::Add(double t, V3 o, V3 d) {
  std::lock_guard<std::mutex> g(m_);
  rt_.push_back(t); ro_.push_back(o); rd_.push_back(d);
}

void Solver::Replace(double from, const std::vector<double> &T, const std::vector<V3> &O, const std::vector<V3> &D) {
  std::lock_guard<std::mutex> g(m_);
  size_t j = std::lower_bound(rt_.begin(), rt_.end(), from) - rt_.begin();
  rt_.resize(j); ro_.resize(j); rd_.resize(j);
  rt_.insert(rt_.end(), T.begin(), T.end());
  ro_.insert(ro_.end(), O.begin(), O.end());
  rd_.insert(rd_.end(), D.begin(), D.end());
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
  }
  Rays r;
  size_t j = std::lower_bound(rt_.begin(), rt_.end(), t0) - rt_.begin();
  r.T.assign(rt_.begin() + j, rt_.end());
  r.O.assign(ro_.begin() + j, ro_.end());
  r.D.assign(rd_.begin() + j, rd_.end());
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

// symmetric 4x4: smallest eigenvalue (Jacobi)
static double MinEig4(double A[4][4]) {
  double a[4][4];
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
        }
        for (int k = 0; k < 4; k++) {
          double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
      }
  }
  return std::min(std::min(a[0][0], a[1][1]), std::min(a[2][2], a[3][3]));
}

static bool Solve4(double A[4][4], const double b[4], double x[4]) {
  double M[4][5];
  for (int i = 0; i < 4; i++) { for (int j = 0; j < 4; j++) M[i][j] = A[i][j]; M[i][4] = b[i]; }
  for (int c = 0; c < 4; c++) {
    int p = c;
    for (int r = c + 1; r < 4; r++) if (std::fabs(M[r][c]) > std::fabs(M[p][c])) p = r;
    if (std::fabs(M[p][c]) < 1e-300) return false;
    if (p != c) for (int j = 0; j < 5; j++) std::swap(M[p][j], M[c][j]);
    for (int r = 0; r < 4; r++) {
      if (r == c) continue;
      double f = M[r][c] / M[c][c];
      for (int j = c; j < 5; j++) M[r][j] -= f * M[c][j];
    }
  }
  for (int i = 0; i < 4; i++) x[i] = M[i][4] / M[i][i];
  return true;
}

// 4-DOF robust fit (soft_l1 on the sightings, quadratic prior rows), sightings assigned to the nearest station under
// x0; yaw about the station centroid. A (head, direction) cell weighs one sighting however many it holds, so a still
// head adds nothing by looking longer. xa: the prior's center (default x0). cond: the sightings alone pin all 4 DOF.
Solver::FitR Solver::Fit(const X4 &x0, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r, double now,
                         double gate, const X4 *xa_in) {
  FitR out;
  std::vector<V3> P, Zq;
  Predict(x0, S, Z, P, Zq);
  std::vector<char> use(r.size(), 0);
  std::vector<int> kk(r.size(), 0);
  int m = 0;
  for (size_t n = 0; n < r.size(); n++) {
    double a;
    Nearest(P, Zq, r.O[n], r.D[n], kk[n], a);
    if (a < gate) { use[n] = 1; m++; }
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
  std::vector<V3> Sc(S.size());
  for (size_t k = 0; k < S.size(); k++) Sc[k] = S[k] - c;
  const double f = FSCALE;
  const size_t nr = rows.size() * 3 + 4;

  auto res = [&](const double y[4], std::vector<double> &rv) {
    rv.resize(nr);
    double cy = std::cos(y[0]), sy = std::sin(y[0]);
    V3 sh = cq + V3{y[1], y[2], y[3]};
    for (size_t i = 0; i < rows.size(); i++) {
      const Row &w = rows[i];
      V3 U = RyMul(cy, sy, Sc[w.k]) + sh - w.o;
      U = U * (1 / norm(U));
      V3 e = cross(U, w.d) * w.sw;
      rv[3 * i] = e.x; rv[3 * i + 1] = e.y; rv[3 * i + 2] = e.z;
    }
    size_t b = rows.size() * 3;
    rv[b] = f * Wrap(y[0] - xa[0]) / SIG_YAW;
    rv[b + 1] = f * (sh.x - cqa.x) / SIG_T;
    rv[b + 2] = f * (sh.y - cqa.y) / SIG_T;
    rv[b + 3] = f * (sh.z - cqa.z) / SIG_T;
  };
  const size_t nrob = rows.size() * 3;
  auto cost = [&](const std::vector<double> &rv) {
    double s = 0;
    for (size_t i = 0; i < nrob; i++) { double z = rv[i] / f; s += 2 * f * f * (std::sqrt(1 + z * z) - 1); }
    for (size_t i = nrob; i < nr; i++) s += rv[i] * rv[i];
    return s;
  };
  auto jac = [&](const double y[4], std::vector<double> &J) {  // central differences, row-major nr x 4
    J.assign(nr * 4, 0);
    std::vector<double> rp, rm;
    for (int j = 0; j < 4; j++) {
      double h = 1e-7, yp[4], ym[4];
      memcpy(yp, y, sizeof yp); memcpy(ym, y, sizeof ym);
      yp[j] += h; ym[j] -= h;
      res(yp, rp); res(ym, rm);
      for (size_t i = 0; i < nr; i++) J[i * 4 + j] = (rp[i] - rm[i]) / (2 * h);
    }
  };

  double y[4] = {x0[0], 0, 0, 0};
  std::vector<double> rv, J, rn;
  res(y, rv);
  double cst = cost(rv), lam = 1e-3;
  for (int it = 0; it < 100; it++) {
    jac(y, J);
    double A[4][4] = {}, g[4] = {};
    for (size_t i = 0; i < nr; i++) {
      double w = 1;
      if (i < nrob) { double z = rv[i] / f; w = 1 / std::sqrt(1 + z * z); }  // IRLS weight of soft_l1
      const double *Ji = &J[i * 4];
      for (int a = 0; a < 4; a++) {
        g[a] += w * Ji[a] * rv[i];
        for (int b = 0; b < 4; b++) A[a][b] += w * Ji[a] * Ji[b];
      }
    }
    bool moved = false, done = false;
    for (int tries = 0; tries < 20; tries++) {
      double Al[4][4], ng[4], d[4];
      for (int a = 0; a < 4; a++) {
        for (int b = 0; b < 4; b++) Al[a][b] = A[a][b];
        Al[a][a] += lam * std::max(A[a][a], 1e-12);
        ng[a] = -g[a];
      }
      if (!Solve4(Al, ng, d)) { lam *= 10; continue; }
      double yn[4] = {y[0] + d[0], y[1] + d[1], y[2] + d[2], y[3] + d[3]};
      res(yn, rn);
      double cn = cost(rn);
      if (cn <= cst) {
        double step = std::fabs(d[0]) + std::fabs(d[1]) + std::fabs(d[2]) + std::fabs(d[3]);
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
  double H[4][4] = {};
  const double sc[4] = {COND_YAW, COND_T, COND_T, COND_T};
  for (size_t i = 0; i < nrob; i++) {
    double z = rv[i] / f, w = std::pow(1 + z * z, -1.5);
    const double *Ji = &J[i * 4];
    for (int a = 0; a < 4; a++)
      for (int b = 0; b < 4; b++) H[a][b] += w * Ji[a] * Ji[b] * sc[a] * sc[b] / (SIG_R * SIG_R);
  }
  out.cond = MinEig4(H) >= 1.0;
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

// one ray per (5 cm head cell, ~2 deg world direction): a lamp seen from a still head collapses to a few rays,
// so consensus only counts parallax
Solver::Rays Solver::Thin(const Rays &r) {
  std::unordered_map<CellKey, size_t, CellHash> first;
  std::vector<size_t> keep;
  for (size_t n = 0; n < r.size(); n++)
    if (first.emplace(KeyOf(r.O[n], r.D[n]), n).second) keep.push_back(n);
  Rays o;
  for (size_t n : keep) { o.T.push_back(r.T[n]); o.O.push_back(r.O[n]); o.D.push_back(r.D[n]); }
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
  Rays r = GetRays(std::max(now - ACQ_WIN, since_));
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
  Rays t = Thin(GetRays(std::max(now - ACQ_WIN, since_)));
  if (!t.size()) return 0;
  std::vector<int> c;
  Support(x, S, Z, t, INLIER, c);
  return Score(c);
}

// the HMD pose stream broke: the Quest's space may have changed under the older sightings (boundary reset)
void Solver::PoseBreak(double t) {
  brk_ = since_ = std::max(since_, t);
  last_acq_ = -1e18;
  has_acq_x_ = false;
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
          if (k[n] == (int)kk && ak[n] < GATE) (r.T[n] > now - 4 ? rec : old).push_back(ak[n]);
        if (rec.size() >= 20 && old.size() >= 20 && Median(rec) - Median(old) > JUMP) {
          since_ = now - 4;
          st.jump = true;
          r = GetRays(since_);
          break;
        }
      }
      FitR f = Fit(x_, S, Z, r, now, GATE, has_anchor_ ? &anchor_ : nullptr);
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
      if ((sa >= ACQ || tight >= TIGHT_N) && sa > 2 * cur + 5) {
        log_(Fmt("acquired: yaw %+.2f deg t [%.3f %.3f %.3f] support %d (%d within %.1f deg, was %d)", xa[0] * kDeg, xa[1],
                 xa[2], xa[3], sa, tight, TIGHT_DEG, cur));
        Reset(xa);
        since_ = brk_;
      }
    }
  }
  stat_ = st; has_stat_ = true;
  return st;
}

// ================================================================ timing
void Timing::Record(double tg, V3 od, V3 dd) {
  rec_.push_back({tg, od, dd});
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
  n = (int)fast.size();
  // the first (wide) estimate wants plenty of turning: from 1500 fast-head sightings on, replays land within 0.4 ms
  // of each other from any start
  if (n < (wide ? 1500 : 300)) return false;
  X4 xs = x;
  if (!RefitSlow(xs, S, Z, so, sd)) return false;
  std::vector<V3> P, Zq;
  Solver::Predict(xs, S, Z, P, Zq);
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
  std::vector<double> T;
  std::vector<V3> O, D;
  for (auto &r : rec) {
    double t = r.tg - new_e, w;
    M3 R;
    V3 p;
    if (!poses_.At(t, R, p) || !poses_.Speed(t, w) || poses_.Still(t) || w > kWmax) continue;
    T.push_back(t);
    O.push_back(p + R * r.od);
    D.push_back(R * r.dd);
  }
  solver_.Replace(rec.front().tg - old_e, T, O, D);
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
  if (sfile_.loaded)
    log_(Fmt("reference frame: anchor %s%s, %d measured station(s)", sfile_.anchor.c_str(),
             sfile_.autogen ? " (automatic)" : "", (int)sfile_.fix.size()));
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
  miss = 0;
  for (size_t i = 0; i < S.size(); i++) miss = std::max(miss, norm(Ry(yaw) * S[i] + t - Q[i]));
  x = {yaw, t.x, t.y, t.z};
  return true;
}

bool Sync::SetCalibration(const std::string &json, std::string *err) {
  std::lock_guard<std::mutex> g(net_);
  if (!optics_.Load(json, err)) return false;
  have_optics_ = true;
  return true;
}

void Sync::HeadsetReset() {
  std::lock_guard<std::mutex> g(net_);
  clock_.Clear();
  grid_ = FrameGrid();
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
  bool have_lag = nb >= 0 && grid_.Lag(cam, hs, lag);
  if (nb <= 0 || !have_optics_) return;
  double grid_pc;
  if (!clock_.Map(hs - (have_lag ? lag : 0.0), grid_pc)) return;
  double expo = expo_;
  double t = grid_pc - expo - (have_lag ? 0.0 : kLagFallback);
  M3 R; V3 p;
  double w;
  if (!poses_.At(t, R, p) || !poses_.Speed(t, w) || poses_.Still(t)) return;
  double wmax = (cfg_.learn_timing && !timing_learned_) ? kWmaxUntimed : kWmax;
  const char *s = line + off;
  for (int i = 0; i < nb; i++) {
    int x10, y10, npx, peak, used = 0;
    if (sscanf(s, " %d %d %d %d%n", &x10, &y10, &npx, &peak, &used) < 4) break;
    s += used;
    if (npx > kMaxPx) continue;
    V3 o, d;
    if (!optics_.Ray(cam, x10 / 10.0 - 0.5, y10 / 10.0 - 0.5, o, d)) continue;
    if (have_lag && cfg_.learn_timing) timing_.Record(grid_pc, o, d);
    if (g_ray_dump) {
      V3 O = p + R * o, D = R * d;
      fprintf(g_ray_dump, "R %.6f %d %.6f %.6f %.6f %.7f %.7f %.7f %.6f %.6f %.6f %.1f\n", t, cam, O.x, O.y, O.z, D.x, D.y,
              D.z, p.x, p.y, p.z, w);
    }
    if (w > wmax) continue;
    solver_.Add(t, p + R * o, R * d);
    nsight_++;
  }
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

void Sync::Reacquire() {
  // like a pose break: older sightings no longer count, acquisition looks again at once
  V3 p;
  double t;
  if (poses_.Latest(p, &t)) solver_.PoseBreak(t);
  log_("re-acquire asked for: older sightings dropped");
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
    {
      std::lock_guard<std::mutex> g(raw_m_);
      raw = raw_;
    }
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
    } else if (sfile_.autogen && raw.count(sfile_.anchor)) {  // an automatic reference follows a moved anchor
      auto &a = raw[sfile_.anchor];
      if (RotDeg(sfile_.anchor_R * T(a.second)) > kMaxRot || norm(a.first - sfile_.anchor_p) > kMoved) {
        sfile_.anchor_p = a.first;
        sfile_.anchor_R = a.second;
        sfile_.SaveAuto(Join(cfg_.dir, "stations.json"));
        log_("anchor " + sfile_.anchor + " moved: reference frame reset");
      }
    }
    frame_.Update(raw);
    std::map<std::string, V3> cs;
    for (auto &kv : raw) {
      V3 p; M3 R;
      frame_.Apply(kv.second.first, kv.second.second, p, R);
      solver_.SetStation(kv.first, p, R);
      if (now - last_s_ >= 10) {
        Quat q = ToQuat(R);
        Rec(now, "S %s %.5f %.5f %.5f %.5f %.5f %.5f %.5f", kv.first.c_str(), p.x, p.y, p.z, q.w, q.x, q.y, q.z);
      }
    }
    if (now - last_s_ >= 10 && !raw.empty()) last_s_ = now;
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
    X4 x = solver_.x();
    if (recording_) {
      std::string per;
      for (auto &kv : st.per) per += Fmt("%s'%s': %d", per.empty() ? "{" : ", ", kv.first.c_str(), kv.second);
      per = st.has_per ? per + "}" : "None";
      Rec(now, "A %s | %s | %d %s %s%s%s", have_applied_ ? Fmt("%+.4f %.4f %.4f %.4f", applied_[0] * kDeg, applied_[1], applied_[2], applied_[3]).c_str() : "-",
          solver_.has_x() ? Fmt("%+.4f %.4f %.4f %.4f", x[0] * kDeg, x[1], x[2], x[3]).c_str() : "-", st.n,
          st.has_med ? Fmt("%.4f", st.med).c_str() : "None", per.c_str(), st.cond ? " cond" : "", st.jump ? " JUMP" : "");
    }
    if (solver_.has_x() && st.cond && st.has_med && st.med < 0.3 && now - last_save_ > 10) {
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
        // the step trusts the estimate by how much turning it saw (all of it from 2000 fast-head sightings on)
        double e = timing_learned_ ? cur + std::min(1.0, n / 2000.0) * (best - cur) : best;
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
  if (now - rate_t_ >= 2.0) {
    long f = nframes_, g = nsight_;
    if (rate_t_ > 0) { cam_fps_ = (f - rate_f_) / (now - rate_t_); sight_rate_ = (g - rate_s_) / (now - rate_t_); }
    rate_t_ = now; rate_f_ = f; rate_s_ = g;
  }
  s.cam_fps = cam_fps_;
  s.sight_rate = sight_rate_;
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
  if (s.has_x && st.has_per) {
    std::vector<int> c;
    for (auto &kv : st.per) c.push_back(kv.second);
    std::sort(c.begin(), c.end());
    s.locked = c.size() >= 2 && c[c.size() - 2] >= 10;
  }
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
