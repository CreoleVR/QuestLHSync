// Gravity levelling, for one or two base stations. SteamVR levels its lighthouse frame by the base stations'
// accelerometers: half a degree or more off, and differently at every start. The cameras can't fix that with two
// stations (the room can turn about the line through them without either moving), and SteamVR re-places the second
// station by centimetres as it goes, so its place is no level either. A third station pins the level
// (Solver::Level); without one, gravity can: a lighthouse controller's or tracker's accelerometer, while it rests,
// through its pose, says which way up is in the room.
//
// The driver listens to the device's receiver (HID input reports, read only: nothing is sent, SteamVR's own
// connection is untouched), decodes the IMU samples (Watchman v2, as libsurvive does) and applies the device's
// factory calibration from SteamVR's config folder. Rests are kept (gravity.json) in the reference frame as the
// anchor alone places it, per SteamVR session, since it re-tilts at every start. The fit takes each session's tilt
// and each device's leftover accelerometer offset, which rests in different orientations tell apart, and the frame
// is levelled by this session's tilt (Frame::SetGravity) once it is known well enough.
#pragma once
#include <atomic>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mathx.h"
#include "sync.h"

class Gravity {
 public:
  Gravity(std::string dir, LogFn log);
  ~Gravity() { Stop(); }
  void Stop();  // ends the reads (SteamVR closing); the rest stays callable
  void SetEnabled(bool on) { enabled_ = on; }
  // a lighthouse controller or tracker (any thread): its serial and receiver (Prop_ConnectedWirelessDongle_String)
  void SetDevice(int id, const std::string &serial, const std::string &receiver);
  // its pose in SteamVR's raw lighthouse universe (its config's head frame), from the pose hook
  void OnPose(int id, double t, V3 p, const M3 &R);
  // the frame to level (Sync::GravityFrame): SteamVR's frame -> the reference frame as the anchor alone places it
  // (rotation), the frame's turn on top of that now, and which reference frame (key). ok false: none to level by
  // gravity (three stations level it)
  struct Ref { bool ok = false; M3 C, turn; std::string key; };
  // the driver's worker thread, once a second. True when what to apply changed: on, and the tilt levelling the frame
  bool Step(const Ref &ref, bool &on, M3 &tilt);

 private:
  struct Sample { double t; V3 p; M3 R; };
  struct Device {
    std::string serial, receiver;
    std::deque<Sample> hist;  // 10 Hz, the last kHist s
    double last = -1, rest_since = -1;
    V3 rest_p;                // where this rest began
    M3 rest_R;
    bool read = false;        // this rest has been read
    int cfg = 0;              // 0 not loaded, 1 loaded, -1 none
    M3 head_imu;              // the config's head frame <- its IMU frame
    V3 bias, scale{1, 1, 1};  // the config's accelerometer calibration (m/s^2): (a - bias) * scale
    std::vector<M3> taken;    // this session's rests (raw <- IMU): the same orientation isn't read twice
  };
  // a rest: accelerometer (calibrated, m/s^2, its axes); reference frame (anchor alone) <- its axes; SteamVR session
  struct Rest { std::string dev, session, when; V3 a; M3 Q; };
  struct Raw { std::string dev; V3 a; M3 R; int n; };  // a read, before the frame (raw <- IMU)

  std::string dir_, session_;
  LogFn log_;
  std::atomic<bool> enabled_{true}, run_{true};
  std::mutex m_;
  std::map<int, Device> dev_;
  std::vector<Raw> pending_;
  std::map<std::string, std::wstring> paths_;  // receiver -> its HID interface
  std::thread worker_;
  std::string config_dir_;
  std::map<std::string, int> fails_;  // receivers that gave no IMU samples, logged once
  // Step's (worker thread)
  std::vector<Rest> rests_;
  std::string key_;
  bool fitted_ = false, have_ = false, on_ = false, good_ = false;
  double tx_ = 0, tz_ = 0;  // this session's tilt (Tilt(x, z), rad)
  Quat applied_;            // the turn applied

  void Worker();
  static bool Still(const Device &d, double from, double to);
  bool Listen(const std::string &receiver, double secs, std::vector<V3> &acc);
  bool FindPath(const std::string &receiver, std::wstring &path);
  bool LoadConfig(Device &d);
  void Load();
  void Save() const;
  void Fit();
};
