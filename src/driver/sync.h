// QuestLHSync core: the lighthouse universe -> headset space alignment (4 DOF: yaw + translation, both spaces are
// gravity-aligned) from base station laser flashes seen by the headset's tracking cameras (a Quest's or a Steam
// Frame's). Shared by the SteamVR driver and the offline tools (qlhs_replay, qlhs_nettest).
//   Optics    pixel -> ray in the HMD pose's frame (the headset's own calibration: Meta's Fisheye62, or the Frame's KB4)
//   FrameGrid the cameras' exact frame grid: a detection's poll lag comes off its timestamp (Quest; the Frame's
//             timestamps are the frames' own)
//   Clock     headset CLOCK_MONOTONIC -> PC clock (QPC seconds): round trips (live) or arrival envelope (old logs)
//   PoseHist  HMD pose history (SteamVR raw = the Quest's STAGE space), interpolated at exposure time
//   Frame     SteamVR's lighthouse frame of this run -> the reference frame (stations.json)
//   Solver    rays + base station poses -> x = (yaw, t): p_quest = Ry(yaw) p_reference + t
//   Timing    learns the frame grid -> HMD pose time offset per streamer (they report poses differently)
//   Sync      ties them together: seeding, slewing, state files, status
#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "mathx.h"

using LogFn = std::function<void(const std::string &)>;
using X4 = std::array<double, 4>;  // yaw, tx, ty, tz

constexpr double kIR = 1 / 1.00434;  // side cameras (OV7251): the near-IR laser images 0.434% further out than the
                                     // visible-light calibration predicts (lateral colour); measured on 4 Quest Pro
                                     // sessions, and assumed for the Quest 3's (the same sensor)
constexpr int kMaxCam = 16;             // camera ids 0..15
constexpr double kLagFallback = 0.005;  // s, typical poll lag while the frame grid isn't known yet
constexpr double kExpoArrival = 0.020;  // s, grid -> exposure with the arrival-envelope clock (logs without round trips)
constexpr double kExpoDefault = 0.018;  // s, grid -> pose time with the round-trip clock, before it's learned
                                        // (CreoleCast learns 18.5, a replayed streamer 20.8)
constexpr double kWmax = 60.0;          // deg/s: sightings while the head turns faster are skipped
constexpr double kWmaxUntimed = 20.0;   // ... until the timing is learned
constexpr double kNearDeg = 5.0;        // deg: a fast-head sighting this close to a base station counts for timing
constexpr int kTimingFirstN = 200;      // such sightings before the first (wide) timing estimate
constexpr int kTimingNextN = 40;        // ... before each later one
constexpr int kTimingFullN = 270;       // a later estimate moves the timing all the way from this many on
constexpr int kMaxPx = 400;             // bigger blobs are lamps/windows, not a laser dot
// blobs this bright: a base station's dot, or a lamp. Dimmer ones (module 1.6 on) are mostly other lights, a few
// percent station dots: they count only in a room too light for the dots to saturate (Solver::starved), and then in
// the running fit only within Solver::INLIER of a station
constexpr int kBright = 250;
constexpr int kLampPx = 20;             // blobs this big are steady lights (the frozen-pose check)
constexpr double kFrozen = 10.0;        // s: SteamVR's headset this still while the cameras see the room move
constexpr double kKeep = 600.0;         // s of rays kept
constexpr double kMoved = 0.25;         // m: SteamVR has a measured station this far off: it was moved
constexpr double kMaxRot = 3.0;         // deg: the anchor turned this much against the reference
// an automatic frame's layout fit: a station kMoved off against the others for kLayoutMoved s takes its new place, and
// no fit within kMoved for that long starts the frame over; stations within kLine of one line pin no tilt
constexpr double kLayoutMoved = 30.0, kLine = 0.3;  // s, m
// an automatic frame of two stations: SteamVR places the second from what single devices saw, centimetres off and
// moved as it goes, and with two stations its height and distance set where along the line between them the headset
// sits. The cameras measure both (Solver::Triangulate) every kPlaceEvery s; a measurement counts when the sightings,
// from kPlaceCells head cells per station or more, pin them within kPlaceSdD and kPlaceSdH, and one kPlaceMoved off
// the kept place (and well outside both) means the station moved
constexpr double kPlaceEvery = 30.0, kPlaceSdD = 0.015, kPlaceSdH = 0.010, kPlaceMoved = 0.05;  // s, m, m, m
constexpr int kPlaceCells = 30;
// one session's place is never taken as known better than kPlaceSessD / kPlaceSessH (recordings of one room: 1.4 and
// 0.9 cm apart from session to session, more than each one's sightings say), so the kept place is the sessions'
// average; it's never known better than kPlaceFloor, and each session loosens it by kPlaceAge
constexpr double kPlaceSessD = 0.012, kPlaceSessH = 0.008, kPlaceFloor = 0.003, kPlaceAge = 0.003;  // m
// corrections move lighthouse devices at the head by at most kSlewStill m/s while the head is still, plus
// kSlewTurn m per degree it turns and kSlewWalk of the distance it moves: a shift is hard to notice while the view
// itself moves, and plain to see on a controller held in front of a still head
constexpr double kSlewStill = 0.005, kSlewTurn = 0.0005, kSlewWalk = 0.05;
constexpr double kJumpApply = 0.25;     // m: bigger corrections apply at once
// levelling (Sync::LevelStep) waits this long after an acquisition, and wants the worn or held devices this close
constexpr double kLevelSettle = 5.0, kLevelBody = 1.0;  // s, m
// channel check (Solver::CheckIdentity): every kIdEvery s; a pair is swapped when its sightings follow each other's
// rotor periods better by kSwapGain, and switched if the swapped alignment keeps kSwapFit of the support
constexpr double kIdEvery = 10.0, kSwapGain = 0.10, kSwapFit = 0.7;
// the lighthouse devices worn or held tell two alignments apart (Solver::BodyFavours) when one puts them within
// kBodyNear of the head and the other kBodyGap further
constexpr double kBodyNear = 1.0, kBodyGap = 1.0;  // m
// ... or their motion does (Sync::BodyMotion): when the head went kBodyMoved, and they follow it one way by kBodyCorr
// and the other way not (a fit and its mirror move them opposite ways)
constexpr double kBodyMoved = 2.0, kBodyCorr = 0.2;  // m, correlation

double RotorPeriod(int channel);  // s, a 2.0 base station on channel 1..16, else 0

// ---------------------------------------------------------------- optics
struct CamCal {
  bool valid = false;
  int w = 0, h = 0;
  double ir = 1.0;  // near-IR laser image scale about the centre: kIR for the Quest side cameras
  M3 R;  // device <- camera
  V3 t;
  double f = 0, fy = 0, cx = 0, cy = 0, k[6] = {}, p[2] = {};
  void Dist(double a, double b, double &u, double &v) const;
  bool Unproject(double px, double py, V3 &d) const;  // unit ray, camera frame
};

struct JVal;

class Optics {
 public:
  bool Load(const std::string &json, std::string *err);
  bool Ray(int cam, double x, double y, V3 &o, V3 &d) const;  // device frame
  bool Pose(int cam, M3 &R, V3 &t) const;                     // device <- camera
  bool InImage(int cam, V3 dc, double margin) const;          // a camera-frame direction lands on the image
  const std::string &device() const { return device_; }       // "Seacliff" (Quest Pro), "Eureka" (Quest 3), ...
  bool exact_time() const { return exact_time_; }  // frame times are the frames' own, no poll lag to take off

 private:
  CamCal cams_[kMaxCam];
  std::string device_;
  bool exact_time_ = false;
  bool LoadFrame(const JVal &root, std::string *err);
};

// ---------------------------------------------------------------- timing helpers
class FrameGrid {
 public:
  // s: a Quest's short frames, every third at 37.5 fps, or at 45 fps with its power line frequency set to 60 Hz
  static constexpr double kQuest50 = 3 / 37.5, kQuest60 = 3 / 45.0;
  static constexpr double WIN = 10.0, LEARN_WIN = 30.0;  // s: the grid's window, the period's
  // quest: the period is whichever of the two the frames fold at; otherwise it's learned (the Frame)
  explicit FrameGrid(bool quest = true) : quest_(quest) {}
  bool Lag(int cam, double t, double &lag);  // t: a short frame's detection time (headset s), call for all
  double period(int cam) const;              // s, 0 while unknown

 private:
  struct Cam { std::deque<double> h, longer; double p = 0, next = 0; bool learn = false; };
  bool quest_;
  std::map<int, Cam> c_;
  static double Learn(const std::deque<double> &h, double prev);
};

class Clock {
 public:
  void AddArrival(double pc, double hs);                      // arrival low envelope (old logs)
  void AddPing(double pc_send, double pc_recv, double hs);   // round trip: offset at its middle, min RTT wins
  bool Map(double hs, double &pc);
  double rtt() const { return rtt_; }
  void Clear() { env_.clear(); has_ = false; dirty_ = false; rtt_ = 0; }

 private:
  struct E { double hs, d, rtt; };
  std::map<long long, E> env_;
  double a_ = 0, b_ = 0, x0_ = 0, rtt_ = 0;
  bool has_ = false, dirty_ = false;
  void Fit();
};

struct PoseS { double t; Quat q; V3 p; };

class PoseHist {
 public:
  static constexpr int N = 1 << 17;
  PoseHist() : ring_(N) {}
  void Add(double t, const Quat &q, V3 p);
  bool At(double t, M3 &R, V3 &p) const;
  bool AtQ(double t, Quat &q, V3 &p) const;
  bool Still(double t, double span = 2.0, double rot = 0.1, double pos = 0.002) const;
  bool Speed(double t, double &w, double span = 0.08) const;  // deg/s
  bool Latest(V3 &p, double *t = nullptr) const;
  int brk_id() const { return brk_id_; }
  double brk_t() const { return brk_t_; }
  std::string brk_what() const { std::lock_guard<std::mutex> g(m_); return brk_what_; }
  void Clear() { std::lock_guard<std::mutex> g(m_); i_ = 0; pend_ = 0; }

 private:
  std::vector<PoseS> ring_;
  uint64_t i_ = 0;
  mutable std::mutex m_;
  std::atomic<int> brk_id_{0};
  double brk_t_ = 0;
  std::string brk_what_;
  uint64_t pend_ = 0;  // a jump waiting for the poses after it: the index of the first pose past it
  bool Find(double t, uint64_t &i) const;  // newest index with time <= t (caller holds m_)
  bool Back(const PoseS &s) const;
  void Settle();
  void Break(const PoseS &a, const PoseS &b);
};

// ---------------------------------------------------------------- reference frame
struct StationsFile {
  bool loaded = false, autogen = false;
  std::string anchor;
  V3 anchor_p;
  M3 anchor_R;
  M3 level;  // automatic frames: turns the frame level with the headset's gravity (Solver::Level)
  std::map<std::string, V3> fix;  // measured positions, reference frame
  // automatic frames of three or more stations: each one's pose in the reference frame, before the level
  std::map<std::string, std::pair<V3, M3>> ref;
  // automatic frames of two stations: the other's distance (horizontal) and height from the anchor as the cameras
  // measured them, and how well (m)
  struct Place { double d = 0, h = 0, sd_d = 1, sd_h = 1; };
  std::map<std::string, Place> place;
  bool Load(const std::string &path);
  bool SaveAuto(const std::string &path) const;
};

// SteamVR's lighthouse frame -> the reference frame: the anchor's pose pins it (an automatic frame of three or more
// stations: the best fit of all their places, Layout), then the level (three stations' sightings) or gravity's tilt
// (gravity.h)
class Frame {
 public:
  explicit Frame(const StationsFile &f, LogFn log) : f_(f), log_(std::move(log)) {}
  void Update(const std::map<std::string, std::pair<V3, M3>> &raw);
  bool layout() const { return f_.autogen && f_.ref.size() >= 3; }
  const std::string &left() const { return left_; }  // the station the layout fit leaves out: moved against the rest
  double miss() const { return miss_; }  // m: the layout fit's worst miss of the stations it keeps
  void Apply(V3 p, const M3 &R, V3 &po, M3 &Ro) const { po = oref_ + C_ * (p - o_); Ro = C_ * R; }
  // before the level: what recordings hold, so a replay with the same stations.json levels them once
  void Unlevelled(V3 p, const M3 &R, V3 &po, M3 &Ro) const { po = oref_ + Cs_ * (p - o_); Ro = Cs_ * R; }
  void Rotation(M3 &C, V3 &t) const { C = C_; t = oref_ - C_ * o_; }  // p_ref = C p_raw + t
  // levelled by gravity (gravity.h): tilt after the level; from the next Update
  void SetGravity(bool on, const M3 &tilt) { grav_ = on; G_ = tilt; }
  const M3 &gravity() const { return GC_; }  // the tilt in C_ (none: identity)
  M3 Anchored() const { return f_.level * Cs_; }  // SteamVR's frame -> the reference frame, before gravity's tilt
  int ok() const { return ok_; }
  double deg() const { return deg_; }

 private:
  const StationsFile &f_;
  LogFn log_;
  M3 C_, Cs_;  // SteamVR's frame -> the reference frame, with (and gravity's tilt) and without the level
  M3 G_, GC_;
  bool grav_ = false;
  V3 o_, oref_;
  int ok_ = -1;
  double deg_ = 0;
  std::string left_;
  double miss_ = 0;
  bool Layout(const std::map<std::string, std::pair<V3, M3>> &raw, M3 &Cs, V3 &o, std::string &left, double &off,
              double &miss) const;
};

// ---------------------------------------------------------------- solver
struct StepStat {
  int n = 0;
  bool has_med = false, has_per = false, jump = false, has_acq = false, cond = false;
  double med = 0;
  std::map<std::string, int> per;
  std::map<std::string, double> last;  // newest inlier sighting per station
  int acq_sa = 0, acq_cur = 0, acq_tight = 0;
};

extern FILE *g_ray_dump;  // tools only (qlhs_replay --rays): every sighting's ray and head pose

class Solver {
 public:
  static constexpr double GATE = 2.0, INLIER = 0.5, TAU = 60.0, WIN = 300.0, JUMP = 1.0, ACQ_WIN = 60.0;
  static constexpr int ACQ = 12, TIGHT_N = 6;
  static constexpr double TIGHT_DEG = 0.2, FSCALE = 0.003;
  static constexpr double SIG_YAW = 0.5 / kDeg, SIG_T = 0.05, SIG_R = 0.1 / kDeg, COND_YAW = 0.1 / kDeg, COND_T = 0.01;
  // the level: a tilt step of the reference frame is fitted with a weak prior (SIG_LEVEL) and kept only when the
  // sightings pin it (COND_LEVEL, like COND_YAW) and at least LEVEL_STATIONS stations hold LEVEL_CELLS inlier cells
  static constexpr double SIG_LEVEL = 1.0 / kDeg, COND_LEVEL = 0.1 / kDeg, LEVEL_MED = 0.3;
  static constexpr int LEVEL_STATIONS = 3, LEVEL_CELLS = 3;
  struct LevelR { X4 x{}; M3 tilt; int stations = 0; double med = 0; };  // tilt about the pivot, then x
  // a tilt about one horizontal axis (rad), how well its time slices agree on it (rad), the sightings' median error
  struct AxisR { double theta = 0, sd = 0, med = 0; int slices = 0; };

  Solver(LogFn log, uint64_t seed, std::map<std::string, V3> fix);
  // how far (m, horizontally, median) the lighthouse devices worn or held stay from the head if x were the alignment;
  // NaN without enough of them
  using BodyFn = std::function<double(const X4 &)>;
  void SetBody(BodyFn f) { body_ = std::move(f); }
  // how their horizontal motion follows the head's if x were the alignment (+1 along, -1 against, ~0 left lying), and
  // how far the head went meanwhile (m); NaN without motion
  using MotionFn = std::function<double(const X4 &, double &)>;
  void SetMotion(MotionFn f) { motion_ = std::move(f); }
  void Reset(const X4 &x);
  // g: the frame's time on the headset's clock (NaN until the frame grid is known)
  void Add(double t, V3 o, V3 d, double g, int cam, bool bright);
  void Replace(double from, const std::vector<double> &T, const std::vector<V3> &O, const std::vector<V3> &D,
               const std::vector<double> &G, const std::vector<int> &C, const std::vector<char> &B);  // rays >= from
  void SetStation(const std::string &serial, V3 p, const M3 &R);
  void SetChannel(const std::string &serial, int channel);
  void SetFramePeriod(double p) { frame_p_ = p; }  // s between short frames
  void AddFrame(double t, double g, int cam, V3 o, const M3 &R);  // every short frame; camera pose in Quest space
  void SetInImage(std::function<bool(int cam, V3 d)> f) { in_image_ = std::move(f); }
  void PoseBreak(double t);
  StepStat Step(double now);
  bool has_x() const { return has_x_; }
  bool starved() const { return starved_; }
  int resets() const { return resets_; }
  X4 x() const { return x_; }
  // stations (sorted by serial) as the solver uses them
  void Stations(std::vector<std::string> &keys, std::vector<V3> &S, std::vector<V3> &Z) const;
  bool measured(const std::string &serial) const { return fix_.count(serial) && !stale_.count(serial); }
  // a measured place from now on (Sync's automatic two-station frames: it follows SteamVR's), or none
  void SetFix(const std::string &serial, V3 p) { fix_[serial] = p; }
  void DropFix(const std::string &serial) { fix_.erase(serial); stale_.erase(serial); }
  // each station's place as the cameras see it (headset space), from the sightings since the last pose break, its
  // covariance (m^2), the inliers, the 5 cm head cells they came from, and the oldest one's time
  struct Seen { V3 q; M3 C; int n = 0, cells = 0; double t0 = 0; };
  bool Triangulate(std::map<std::string, Seen> &out);
  // Both spaces share gravity only if SteamVR's lighthouse frame is level: a tilt of it moves lighthouse devices far
  // below the base stations sideways, and with two stations in view the 4-DOF fit can't see it. The third station's
  // sightings can: refit with the stations tilted about pivot (reference frame). True if the sightings pin the tilt.
  bool Level(V3 pivot, LevelR &out);
  bool LevelAxis(V3 pivot, V3 axis, AxisR &out);  // two stations: the tilt across them (Sync::LevelCheck)
  void Relevel(const X4 &x);  // the alignment for the stations as just levelled

  // geometry, also used by Timing
  static void Predict(const X4 &x, const std::vector<V3> &S, const std::vector<V3> &Z, std::vector<V3> &P, std::vector<V3> &Zq);
  static void Nearest(const std::vector<V3> &P, const std::vector<V3> &Zq, V3 o, V3 d, int &k, double &a);

 private:
  struct Rays {
    std::vector<double> T, G;
    std::vector<V3> O, D;
    std::vector<int> C;
    std::vector<char> B;  // 1: peak >= kBright
    size_t size() const { return T.size(); }
  };
  struct FitR { bool ok = false; X4 x{}; int n = 0; bool cond = false; M3 tilt; double theta = 0; };

  mutable std::mutex m_;
  std::vector<double> rt_, rg_;
  std::vector<V3> ro_, rd_;
  std::vector<int> rc_;
  std::vector<char> rb_;
  std::map<std::string, std::pair<V3, M3>> S_;
  std::map<std::string, int> ch_;
  std::map<std::string, V3> fix_;
  std::set<std::string> stale_;
  bool has_x_ = false, has_anchor_ = false, has_acq_x_ = false;
  X4 x_{}, anchor_{}, acq_x_{};
  double since_ = -1e18, brk_ = -1e18, last_acq_ = -1e18, seen_ = -1e18, last_id_ = -1e18;
  std::atomic<double> frame_p_{0};
  struct CamFrame { double t, g; int cam; V3 o; M3 R; };
  std::deque<CamFrame> frames_;
  std::function<bool(int, V3)> in_image_;
  std::string id_said_;
  LogFn log_;
  std::mt19937_64 rng_;
  StepStat stat_;
  bool has_stat_ = false;
  int resets_ = 0;
  BodyFn body_;
  MotionFn motion_;
  bool acq_forced_ = false;  // the last acquisition overruled the current fit by the mirror check
  bool acq_dim_ = false;     // ... was found on dim rays too (Bright): it needs twice the support
  std::atomic<bool> starved_{false};  // the recent rays are under 5% bright, of 200 or more (see Bright)
  static bool Starved(const Rays &r);
  double DimGate() const { return starved_ ? INLIER : -1; }
  int amb_state_ = 0;        // the mirror check's last outcome, logged when it changes
  double kept_said_ = -1e18;  // the last log of a far acquisition the worn devices turned down

  Rays GetRays(double t0);
  void Support(const X4 &x, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r, double gate, std::vector<int> &cnt) const;
  static int Score(const std::vector<int> &c);
  static void Cells(const Rays &r, const std::vector<char> &use, std::vector<int> &cnt);
  // pivot: also fit a tilt of the stations about it (6 DOF), returned in FitR::tilt; with axis (horizontal, unit),
  // only about that axis (5 DOF, FitR::theta too). dim_gate: the gate for rays dimmer than kBright, when narrower
  FitR Fit(const X4 &x0, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r, double now, double gate,
           const X4 *xa, const V3 *pivot = nullptr, double dim_gate = 180, const V3 *axis = nullptr);
  void Hypotheses(const Rays &r, const std::vector<V3> &S, std::vector<X4> &H, int M = 4000);
  void BatchSupport(const std::vector<X4> &H, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r,
                    const std::vector<int> &idx, double gate, std::vector<int> &out) const;
  static Rays Thin(const Rays &r);
  // r's bright rays (kBright): dim ones are mostly other lights, and can make a wrong alignment look supported. All
  // of r when bright ones are under 5% of 200 or more: a room too light for the dots to saturate
  static Rays Bright(const Rays &r);
  bool Acquire(double now, X4 &best, int &bs, int &tight);
  int ThinnedScore(const X4 &x, double now);
  // 1: the lighthouse devices worn or held say a, 2: b, 0: they don't tell (da, db: their distance from the head, m;
  // how: what told)
  int BodyFavours(const X4 &a, const X4 &b, double &da, double &db, std::string &how) const;
  void CheckMirror(X4 &best, int &bs, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r, const Rays &t,
                   double now);
  void CheckIdentity(double now, const std::vector<std::string> &keys, const std::vector<V3> &S, const std::vector<V3> &Z);
};

// ---------------------------------------------------------------- timing
class Timing {
 public:
  struct Rec { double tg; V3 od, dd; double hg; int cam; bool bright; };  // grid time (PC s), device-frame ray,
                                                                          // headset grid time, peak >= kBright
  void Record(double tg, V3 od, V3 dd, double hg, int cam, bool bright);
  const std::deque<Rec> &recs() const { return rec_; }
  // grid search of the offset on fast-head sightings against the current alignment: true if it found one
  bool Estimate(const PoseHist &poses, const X4 &x, const std::vector<V3> &S, const std::vector<V3> &Z, double cur,
                bool wide, double &best, int &n);

 private:
  std::deque<Rec> rec_;
};

// ---------------------------------------------------------------- the app
struct SyncConfig {
  std::string dir;         // state files (stations.json, state.json)
  bool arrival_clock = false;  // old logs: no round trips, fixed EXPO
  bool learn_timing = true;
  bool learn_grid = false;     // tools: learn the frame period even on a Quest Pro
};

struct Transform { bool active = false; Quat q; V3 t; };  // raw lighthouse -> Quest

class Sync {
 public:
  Sync(SyncConfig cfg, LogFn log);
  ~Sync();
  // headset
  bool SetCalibration(const std::string &json, std::string *err);
  void OnLine(double pc, const char *line);                         // lhsight lines
  void OnPing(double pc_send, double pc_recv, double hs) { std::lock_guard<std::mutex> g(net_); clock_.AddPing(pc_send, pc_recv, hs); }
  void HeadsetReset();                                              // new connection: new clock, grid
  // SteamVR
  void OnHmdPose(double t, const Quat &q, V3 p);
  void OnBodyPose(int dev, double t, V3 raw);  // lighthouse controllers and trackers (raw lighthouse universe)
  void SetStationsRaw(const std::map<std::string, std::pair<V3, M3>> &raw);
  void SetChannels(const std::map<std::string, int> &ch);
  void SetStreamer(const std::string &system);
  // 20 Hz; returns the transform for the lighthouse devices
  Transform Tick(double now);
  bool WillStep(double now) const { return now - last_step_ >= 1.0; }
  void ForceExpo(double e) { expo_ = e; }
  // commands
  // a reference frame to level by gravity (gravity.h): SteamVR's frame -> the reference frame before gravity's tilt
  // (rotation), the frame's turn on top of that now, and which reference frame (key). False when the cameras level it
  bool GravityFrame(M3 &C, M3 &turn, std::string &key);
  void SetGravity(bool on, const M3 &tilt) { frame_.SetGravity(on, tilt); }  // the driver's worker thread, as Tick
  void SetPaused(bool p) { paused_ = p; }
  bool paused() const { return paused_; }
  // status
  struct Status {
    bool has_x = false, locked = false, cond = false, timing_learned = false;
    X4 x{};
    double med = -1, expo = 0, rtt = 0, cam_fps = 0, spot_rate = 0, sight_rate = 0, lag_cm = 0, locked_for = -1;
    int head_still = 0;  // 1: still for 2 s; 2: still for kFrozen s while the cameras see the room move
    int n = 0, nstations = 0;
    struct St { std::string serial; int support = 0; bool anchor = false, measured = false; double last_seen = -1, dist = 0; };
    std::vector<St> st;
  };
  Status GetStatus(double now);
  // the cameras' bright spots so far, and what became of them
  struct Spots { long frames = 0, seen = 0, used = 0, still = 0, fast = 0, big = 0, other = 0, dim = 0; };
  Spots spots() { std::lock_guard<std::mutex> g(net_); return spots_; }
  static std::string Describe(const Spots &from, const Spots &to);
  void StationsForDump(std::vector<std::string> &keys, std::vector<V3> &S) const {  // tools
    std::vector<V3> Z;
    solver_.Stations(keys, S, Z);
  }
  // recording: every input as log records (qlhs_replay reads them)
  void SetRecord(FILE *f);
  void Rec(double t, const char *fmt, ...);
  bool recording() const { return recording_; }
  const PoseHist &poses() const { return poses_; }

 private:
  SyncConfig cfg_;
  LogFn log_;
  std::mutex net_;  // headset-side state (optics, clock, grid, timing records)
  Optics optics_;
  bool have_optics_ = false;
  Clock clock_;
  FrameGrid grid_;
  bool grid_quest_ = true;             // a Quest's frame grid (otherwise learned)
  std::map<int, double> grid_logged_;  // frame periods as last logged, per camera
  PoseHist poses_;
  StationsFile sfile_;
  Frame frame_;
  Solver solver_;
  Timing timing_;
  std::atomic<double> expo_{kExpoDefault};
  std::atomic<bool> timing_learned_{false};
  std::string streamer_;
  std::map<std::string, double> timings_;  // learned per streamer (state.json)

  std::mutex raw_m_;
  std::map<std::string, std::pair<V3, M3>> raw_;
  std::map<std::string, int> channels_;
  std::map<std::string, V3> quest_;  // saved station positions in Quest space
  bool seeded_ = false, have_applied_ = false, paused_ = false;
  X4 applied_{};
  double last_step_ = 0, last_save_ = 0, last_s_ = 0, last_timing_ = 0, last_level_ = 0, lock_time_ = -1;
  int brk_seen_ = 0, resets_seen_ = 0;
  double last_prec_ = -1e18;
  StepStat last_st_;
  bool has_last_st_ = false;
  std::atomic<long> nframes_{0};
  Spots spots_;  // under net_
  std::map<int, std::vector<std::pair<double, double>>> prev_big_;  // under net_: each camera's last steady lights
  long img_moved_ = 0, img_still_ = 0;  // under net_: of those, while SteamVR's headset stands still
  double still_since_ = -1;
  bool frozen_said_ = false;
  double rate_t_ = 0, cam_fps_ = 0, spot_rate_ = 0, sight_rate_ = 0;
  long rate_f_ = 0;
  Spots rate_sp_;
  double sum_t_ = -1;  // not acquired: since then (< 0: acquired), and the spots then
  Spots sum_sp_;
  int sum_n_ = 0;
  std::mutex rec_m_;
  FILE *rec_ = nullptr;
  std::atomic<bool> recording_{false};
  X4 saved_x_{};  // what state.json holds: rewritten only when the alignment moves
  std::map<std::string, V3> saved_cs_;
  std::map<std::string, double> saved_timings_;
  bool has_saved_ = false;
  std::mutex st_m_;
  struct BodyS { double t; int dev; V3 p; };  // raw lighthouse universe
  std::mutex body_m_;
  std::deque<BodyS> body_, body_new_;  // the last ACQ_WIN s; new ones, for the recording
  std::map<int, double> body_last_;
  double BodyDist(const X4 &x);
  double BodyMotion(const X4 &x, double &moved);

  void LoadState();
  void Retime(double old_e, double new_e);
  void LevelStep(const std::map<std::string, std::pair<V3, M3>> &raw, double now);
  void LayoutStep(const std::map<std::string, std::pair<V3, M3>> &raw, double now);
  std::string left_seen_;  // the station the layout fit leaves out, since left_since_
  double left_since_ = 0, bad_since_ = -1;  // bad_since_: since when no fit holds (-1: one does)
  void LevelCheck(double now);
  double last_check_ = 0, check_said_ = -1e18;
  // automatic frames of two stations: the other station's place (StationsFile::place) from the cameras
  bool TwoStations(const std::map<std::string, std::pair<V3, M3>> &raw, std::string &other) const;
  void PlaceStep(const std::map<std::string, std::pair<V3, M3>> &raw, double now);
  struct PlaceM { double t0, t1, d, h, sd_d, sd_h; };
  std::map<std::string, std::vector<PlaceM>> place_sess_;  // this session's: overlapping ones keep the better
  std::map<std::string, StationsFile::Place> place_prior_;  // kept by earlier sessions
  std::map<std::string, int> place_off_;  // measurements in a row far off the kept place
  double last_place_ = 0, place_saved_ = -1e18;
  void SaveState(const X4 &x, const std::map<std::string, V3> &cs);
  bool Seed(const std::map<std::string, V3> &raw, X4 &x, double &miss);
};
