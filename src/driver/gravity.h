// Gravity levelling, for one or two base stations. SteamVR levels its lighthouse frame by the base stations'
// accelerometers: half a degree or more off, and differently at every start. The cameras can't fix that with two
// stations (the room can turn about the line through them without either moving), and SteamVR re-places the second
// station by centimetres as it goes, so its place is no level either. A third station pins the level
// (Solver::Level); without one, gravity can: the lighthouse controllers' and trackers' accelerometers, through their
// poses, say which way up is in the room.
//
// The driver listens to each device's receiver (HID input reports, read only: nothing is sent, SteamVR's own
// connection is untouched), decodes the IMU samples (Watchman v2, as libsurvive does) and applies the device's
// factory calibration from SteamVR's config folder. Over each second, with R the IMU's turn from its pose, a its
// accelerometer and v the IMU's velocity (both from the poses):
//     sum R a dt - (v(t1) - v(t0)) = G T + (sum R dt) o + T b
// G is gravity, o the offset the factory calibration left (it moves as the device warms up) and b the sideways error
// of SteamVR's orientation of the device (it moves as SteamVR recalibrates the device's IMU). That's linear, and
// gravity stays put in the room while the offset turns with the device: devices that move and turn, as they do in
// use, tell them apart, and a device at rest is the same with v = 0. A Kalman filter follows G per SteamVR session
// (it re-tilts at every start), in the reference frame as the anchor alone places it: G is what the devices agree
// on. Once settled, the level is held for the session. The offsets are kept (gravity.json) for the next session. The
// IMU's clock is mapped onto the poses' by the samples' arrival times and the gyro's turning against the orientations'.
#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "mathx.h"
#include "sync.h"

class Gravity {
 public:
  // live: listens to the receivers on its own thread; else (tests) OnImu and Process are called by hand
  Gravity(std::string dir, LogFn log, bool live = true);
  ~Gravity() { Stop(); }
  void Stop();  // ends the reads (SteamVR closing); the rest stays callable
  void SetEnabled(bool on) { enabled_ = on; }
  // recordings: each window and fit as a line (Sync::Rec), for replaying the fit offline
  using RecFn = std::function<void(double, const std::string &)>;
  void SetRecord(RecFn f) { rec_ = std::move(f); }
  // a lighthouse controller or tracker (any thread): its serial and receiver (Prop_ConnectedWirelessDongle_String)
  void SetDevice(int id, const std::string &serial, const std::string &receiver);
  // its pose in SteamVR's raw lighthouse universe (its config's head frame), from the pose hook, every update
  void OnPose(int id, double t, V3 p, const Quat &q);
  // the frame to level (Sync::GravityFrame): SteamVR's frame -> the reference frame as the anchor alone places it
  // (rotation), the frame's turn on top of that now, and which reference frame (key). ok false: none to level by
  // gravity (three stations level it)
  struct Ref { bool ok = false; M3 C, turn; std::string key; };
  // the driver's worker thread, once a second. True when what to apply changed: on, and the tilt levelling the frame
  bool Step(const Ref &ref, bool &on, M3 &tilt);

  // a receiver's IMU sample: arrival (QPC s), the device's 48 MHz time, accelerometer and gyro (counts)
  void OnImu(const std::string &receiver, double arrival, uint32_t tick, const int16_t a[3], const int16_t g[3]);
  void Process(double now);  // the worker's round: windows, the device clocks, the fit

 private:
  struct ImuS { double arr; int64_t tick; float a[3], g[3]; };
  struct PoseS { double t; V3 p; Quat q; };
  struct Bin { double dev, off; int n; };  // a 2 s bin of the device clock: arrival - device time, its least
  struct Rx {                              // a receiver (worker's)
    std::deque<ImuS> s;                    // the last kKeepS s
    int64_t last = -1;                     // the newest sample's time, unwrapped
    std::deque<Bin> bins;
    double a = 0, b = 0, ref = 0;          // device time s -> QPC: s + a + b (s - ref)
    bool mapped = false;
    double rate = 0;                       // samples/s
    int epoch = 0;                         // counts the clock's restarts
  };
  struct Dev {  // a device (worker's)
    std::string serial, receiver;
    std::deque<PoseS> poses;  // the last kKeepS s
    int cfg = 0;              // 0 not loaded, 1 loaded, -1 none
    M3 head_imu;              // the config's head frame <- its IMU frame
    V3 r_head;                // the IMU's place in the head frame
    V3 bias, scale{1, 1, 1};  // the config's accelerometer calibration (m/s^2): (a - bias) * scale
    double cpg = 0;           // counts per g, once known
    double lag = 0, next_lag = 0;
    bool has_lag = false, said = false, shared = false;
    double next = -1;         // the next window's start (device time mapped, pose clock)
    double redo = -1;         // still windows before this were taken before its clock was known
    int epoch = -1;           // its receiver's clock's, when lag and next were found
  };
  struct Win { std::string dev; V3 y; double T; M3 M; double spin, t; };  // a second, reference frame
  struct Off { V3 o; double sd; std::string when; };                     // a device's offset (its IMU axes)
  struct Fit {
    bool ok = false;
    V3 G;
    double tx = 0, tz = 0, sd = 1e9, moving = 0;
    int devs = 0;
    std::map<std::string, Off> off;
    std::string key;  // the reference frame it's in
  };
  struct Reader {
    std::thread th;
    std::atomic<bool> done{false}, stop{false};
    std::atomic<long long> n{0};  // samples read
  };

  std::string dir_;
  LogFn log_;
  RecFn rec_;
  bool live_;
  std::atomic<bool> enabled_{true}, run_{true};
  std::thread worker_;
  std::string config_dir_;
  // shared: handed over to the worker under m_
  std::mutex m_;
  std::map<int, std::pair<std::string, std::string>> dev_in_;  // id -> serial, receiver
  std::map<int, std::vector<PoseS>> pose_in_;
  std::map<int, double> pose_last_;
  std::map<std::string, std::vector<ImuS>> imu_in_;
  Ref ref_;
  Fit fit_;  // the newest fit, for Step
  int fit_seq_ = 0;
  // the worker's
  std::map<int, Dev> dev_;
  std::map<std::string, Rx> rx_;
  std::map<std::string, std::unique_ptr<Reader>> readers_;
  std::map<std::string, double> retry_;
  std::map<std::string, int> fails_;
  // the filter: gravity, then each device's offset and tilt error (slot_: where); kP_ its covariance; kt_ its
  // newest window
  std::vector<double> kx_, kP_;
  std::map<std::string, int> slot_;
  std::map<std::string, double> born_;  // a device's first window this session (it warms up from about then)
  double kt_ = -1, noise0_ = 1, noise1_ = 1, t_first_ = 1e300, t_last_ = -1e300;
  int nwin_ = 0;
  std::set<long long> mv_;  // seconds any device moved in
  std::map<std::string, Off> prior_, saved_;  // offsets from earlier sessions (gravity.json); and to keep
  std::string key_;
  double next_fit_ = 0, last_save_ = 0, last_log_ = -1e9;
  bool new_wins_ = false, dirty_ = false, applying_ = false;
  int said_state_ = -1;
  double said_tx_ = 0, said_tz_ = 0;
  // the level once settled (kHoldAfter): held for the session
  bool held_ = false;
  double settle_since_ = -1, held_tx_ = 0, held_tz_ = 0;
  // Step's
  int seen_seq_ = 0;
  bool on_ = false, good_ = false;
  double tx_ = 0, tz_ = 0;
  std::string fit_key_;
  Quat applied_;

  void Worker();
  void ReadLoop(std::string receiver, Reader *r);
#ifdef _WIN32
  bool FindPath(const std::string &receiver, std::wstring &path);
#else
  bool FindPath(const std::string &receiver, std::string &path);
#endif
  bool LoadConfig(Dev &d);
  void Ingest(Rx &r, const ImuS &s);
  void MapClock(Rx &r);
  double ImuTime(const Rx &r, const ImuS &s) const { double d = s.tick / 48e6; return d + r.a + r.b * (d - r.ref); }
  bool PoseAt(const Dev &d, double t, Quat &q, V3 &p) const;
  bool VelAt(const Dev &d, double t, V3 &v) const;
  void FindLag(Dev &d, const Rx &r, double now);
  void Windows(Dev &d, const Rx &r, const Ref &ref);
  int Slot(const std::string &dev, double t);
  void Update(const Win &w);
  void Report(double now);
  void Load();
  void Save();
};
