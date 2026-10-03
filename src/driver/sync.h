// QuestLHSync core: the lighthouse universe -> headset space alignment (4 DOF: yaw + translation, both spaces are
// gravity-aligned) from base station laser flashes seen by the headset's tracking cameras (a Quest Pro's or a Steam
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

constexpr double kIR = 1 / 1.00434;  // Quest side cameras: the near-IR laser images 0.434% further out than the
                                     // visible-light calibration predicts (lateral colour); measured on 4 sessions
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
constexpr double kKeep = 600.0;         // s of rays kept
constexpr double kMoved = 0.25;         // m: SteamVR has a measured station this far off: it was moved
constexpr double kMaxRot = 3.0;         // deg: the anchor turned this much against the reference
// corrections move lighthouse devices at the head by at most kSlewStill m/s while the head is still, plus
// kSlewTurn m per degree it turns and kSlewWalk of the distance it moves: a shift is hard to notice while the view
// itself moves, and plain to see on a controller held in front of a still head
constexpr double kSlewStill = 0.005, kSlewTurn = 0.0005, kSlewWalk = 0.05;
constexpr double kJumpApply = 0.25;     // m: bigger corrections apply at once

// ---------------------------------------------------------------- optics
struct CamCal {
  bool valid = false;
  int w = 0, h = 0;
  M3 R;  // device <- camera
  V3 t;
  double f = 0, fy = 0, cx = 0, cy = 0, k[6] = {}, p[2] = {};
  double ir = 1;  // near-IR laser image scale about the centre
  void Dist(double a, double b, double &u, double &v) const;
  bool Unproject(double px, double py, V3 &d) const;  // unit ray, camera frame
};

struct JVal;

class Optics {
 public:
  bool Load(const std::string &json, std::string *err);
  bool Ray(int cam, double x, double y, V3 &o, V3 &d) const;  // device frame
  bool ok() const { return cams_[0].valid || cams_[1].valid || cams_[2].valid || cams_[3].valid; }
  bool exact_time() const { return exact_time_; }  // frame times are the frames' own, no poll lag to take off

 private:
  CamCal cams_[4];
  bool exact_time_ = false;
  bool LoadFrame(const JVal &root, std::string *err);
};

// ---------------------------------------------------------------- timing helpers
class FrameGrid {
 public:
  static constexpr double P = 3 / 37.5, WIN = 10.0;
  bool Lag(int cam, double t, double &lag);  // t: a short frame's detection time (headset s), call for all

 private:
  std::map<int, std::deque<double>> h_;
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
  void Clear() { std::lock_guard<std::mutex> g(m_); i_ = 0; }

 private:
  std::vector<PoseS> ring_;
  uint64_t i_ = 0;
  mutable std::mutex m_;
  std::atomic<int> brk_id_{0};
  double brk_t_ = 0;
  std::string brk_what_;
  bool Find(double t, uint64_t &i) const;  // newest index with time <= t (caller holds m_)
};

// ---------------------------------------------------------------- reference frame
struct StationsFile {
  bool loaded = false, autogen = false;
  std::string anchor;
  V3 anchor_p;
  M3 anchor_R;
  M3 level;  // automatic frames: turns the frame level with the headset's gravity (Solver::Level)
  std::map<std::string, V3> fix;  // measured positions, reference frame
  bool Load(const std::string &path);
  bool SaveAuto(const std::string &path) const;
};

class Frame {
 public:
  explicit Frame(const StationsFile &f, LogFn log) : f_(f), log_(std::move(log)) {}
  void Update(const std::map<std::string, std::pair<V3, M3>> &raw);
  void Apply(V3 p, const M3 &R, V3 &po, M3 &Ro) const { po = oref_ + C_ * (p - o_); Ro = C_ * R; }
  // before the level: what recordings hold, so a replay with the same stations.json levels them once
  void Unlevelled(V3 p, const M3 &R, V3 &po, M3 &Ro) const { po = oref_ + Cs_ * (p - o_); Ro = Cs_ * R; }
  void Rotation(M3 &C, V3 &t) const { C = C_; t = oref_ - C_ * o_; }  // p_ref = C p_raw + t
  int ok() const { return ok_; }
  double deg() const { return deg_; }

 private:
  const StationsFile &f_;
  LogFn log_;
  M3 C_, Cs_;  // SteamVR's frame -> the reference frame, with and without the level
  V3 o_, oref_;
  int ok_ = -1;
  double deg_ = 0;
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

  Solver(LogFn log, uint64_t seed, std::map<std::string, V3> fix);
  // how far (m, horizontally, median) the lighthouse devices worn or held stay from the head if x were the alignment;
  // NaN without enough of them
  using BodyFn = std::function<double(const X4 &)>;
  void SetBody(BodyFn f) { body_ = std::move(f); }
  void Reset(const X4 &x);
  void Add(double t, V3 o, V3 d);
  void Replace(double from, const std::vector<double> &T, const std::vector<V3> &O, const std::vector<V3> &D);  // rays >= from
  void SetStation(const std::string &serial, V3 p, const M3 &R);
  void PoseBreak(double t);
  StepStat Step(double now);
  bool has_x() const { return has_x_; }
  int resets() const { return resets_; }
  X4 x() const { return x_; }
  // stations (sorted by serial) as the solver uses them
  void Stations(std::vector<std::string> &keys, std::vector<V3> &S, std::vector<V3> &Z) const;
  bool measured(const std::string &serial) const { return fix_.count(serial) && !stale_.count(serial); }
  // Both spaces share gravity only if SteamVR's lighthouse frame is level: a tilt of it moves lighthouse devices far
  // below the base stations sideways, and with two stations in view the 4-DOF fit can't see it. The third station's
  // sightings can: refit with the stations tilted about pivot (reference frame). True if the sightings pin the tilt.
  bool Level(V3 pivot, LevelR &out);
  void Relevel(const X4 &x);  // the alignment for the stations as just levelled

  // geometry, also used by Timing
  static void Predict(const X4 &x, const std::vector<V3> &S, const std::vector<V3> &Z, std::vector<V3> &P, std::vector<V3> &Zq);
  static void Nearest(const std::vector<V3> &P, const std::vector<V3> &Zq, V3 o, V3 d, int &k, double &a);

 private:
  struct Rays { std::vector<double> T; std::vector<V3> O, D; size_t size() const { return T.size(); } };
  struct FitR { bool ok = false; X4 x{}; int n = 0; bool cond = false; M3 tilt; };

  mutable std::mutex m_;
  std::vector<double> rt_;
  std::vector<V3> ro_, rd_;
  std::map<std::string, std::pair<V3, M3>> S_;
  std::map<std::string, V3> fix_;
  std::set<std::string> stale_;
  bool has_x_ = false, has_anchor_ = false, has_acq_x_ = false;
  X4 x_{}, anchor_{}, acq_x_{};
  double since_ = -1e18, brk_ = -1e18, last_acq_ = -1e18, seen_ = -1e18;
  LogFn log_;
  std::mt19937_64 rng_;
  StepStat stat_;
  bool has_stat_ = false;
  int resets_ = 0;
  BodyFn body_;
  bool acq_forced_ = false;  // the last acquisition overruled the current fit by the mirror check
  int amb_state_ = 0;        // the mirror check's last outcome, logged when it changes

  Rays GetRays(double t0);
  void Support(const X4 &x, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r, double gate, std::vector<int> &cnt) const;
  static int Score(const std::vector<int> &c);
  static void Cells(const Rays &r, const std::vector<char> &use, std::vector<int> &cnt);
  // pivot: also fit a tilt of the stations about it (6 DOF), returned in FitR::tilt
  FitR Fit(const X4 &x0, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r, double now, double gate,
           const X4 *xa, const V3 *pivot = nullptr);
  void Hypotheses(const Rays &r, const std::vector<V3> &S, std::vector<X4> &H, int M = 4000);
  void BatchSupport(const std::vector<X4> &H, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r,
                    const std::vector<int> &idx, double gate, std::vector<int> &out) const;
  static Rays Thin(const Rays &r);
  bool Acquire(double now, X4 &best, int &bs, int &tight);
  int ThinnedScore(const X4 &x, double now);
  void CheckMirror(X4 &best, int &bs, const std::vector<V3> &S, const std::vector<V3> &Z, const Rays &r, const Rays &t,
                   double now);
};

// ---------------------------------------------------------------- timing
class Timing {
 public:
  struct Rec { double tg; V3 od, dd; };  // grid time (PC s), device-frame origin and ray
  void Record(double tg, V3 od, V3 dd);
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
  void SetStreamer(const std::string &system);
  // 20 Hz; returns the transform for the lighthouse devices
  Transform Tick(double now);
  bool WillStep(double now) const { return now - last_step_ >= 1.0; }
  void ForceExpo(double e) { expo_ = e; }
  // commands
  void Reacquire();
  void SetPaused(bool p) { paused_ = p; }
  bool paused() const { return paused_; }
  // status
  struct Status {
    bool has_x = false, locked = false, cond = false, timing_learned = false;
    X4 x{};
    double med = -1, expo = 0, rtt = 0, cam_fps = 0, sight_rate = 0, lag_cm = 0, locked_for = -1;
    int n = 0, nstations = 0;
    struct St { std::string serial; int support = 0; bool anchor = false, measured = false; double last_seen = -1, dist = 0; };
    std::vector<St> st;
  };
  Status GetStatus(double now);
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
  std::map<std::string, V3> quest_;  // saved station positions in Quest space
  bool seeded_ = false, have_applied_ = false, paused_ = false;
  X4 applied_{};
  double last_step_ = 0, last_save_ = 0, last_s_ = 0, last_timing_ = 0, last_level_ = 0, lock_time_ = -1;
  int brk_seen_ = 0, resets_seen_ = 0;
  double last_prec_ = -1e18;
  StepStat last_st_;
  bool has_last_st_ = false;
  std::atomic<long> nframes_{0}, nsight_{0};
  double rate_t_ = 0, cam_fps_ = 0, sight_rate_ = 0;
  long rate_f_ = 0, rate_s_ = 0;
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

  void LoadState();
  void Retime(double old_e, double new_e);
  void LevelStep(const std::map<std::string, std::pair<V3, M3>> &raw);
  void SaveState(const X4 &x, const std::map<std::string, V3> &cs);
  bool Seed(const std::map<std::string, V3> &raw, X4 &x, double &miss);
};
