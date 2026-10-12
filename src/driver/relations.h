// SteamVR places every base station but the first from a "relationship": a device that has just found its pose sees
// two stations, which says where they sit relative to each other. Each new one replaces the last, and one device's
// view is off by centimetres and up to a degree or more, so SteamVR moves the station whenever a device finds its
// tracking again (vrserver.txt: "Moving base ... because of relationship with ..."), and everything it tracks through
// that station moves with it. This hooks that step in SteamVR's lighthouse driver and hands it the average of the
// pair's relationships instead (kept across sessions in relations.json).
#pragma once
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "mathx.h"
#include "sync.h"

class Relations {
 public:
  Relations(std::string dir, LogFn log);
  void SetEnabled(bool on) { enabled_ = on; }
  // hooks driver_lighthouse.dll's relationship step (MinHook initialised). True once there's nothing more to try
  // (hooked, or code this doesn't know), false while that DLL isn't loaded yet
  bool Hook();
  void Unhook();
  // SteamVR's relationship of stations a and b (qx qy qz qw tx ty tz; a = rel * b) -> out, the one it gets instead;
  // false: it keeps its own. Runs on SteamVR's tracking thread
  bool Average(uint32_t a, uint32_t b, const float rel[7], float out[7]);
  // logs what Average changed since the last call and writes relations.json, at most every 10 s unless now (the
  // driver's worker thread)
  void Flush(bool now = false);

  struct Pose { Quat q; V3 t; };
  static Pose Inv(const Pose &p);
  static Pose Mean(const std::vector<Pose> &v);
  // the relationship step in a loaded driver_lighthouse.dll, or null and why
  static void *Find(void *module, std::string &why);

 private:
  std::string dir_;
  LogFn log_;
  std::atomic<bool> enabled_{true};
  std::mutex m_;
  std::map<std::pair<uint32_t, uint32_t>, std::vector<Pose>> pairs_;  // lower serial first: a = rel * b
  std::map<std::pair<uint32_t, uint32_t>, Pose> pending_;              // one far from the average, waiting for another
  bool dirty_ = false;                                                 // pairs_ changed since relations.json
  std::vector<std::string> said_;                                      // log lines for Flush
  uint64_t saved_t_ = 0, say_t0_ = 0;                                  // GetTickCount64 ms
  int say_n_ = 0, say_skipped_ = 0;                                    // lines this minute, and dropped
  bool odd_said_ = false;                                              // a relationship that isn't a pose logged
  void *target_ = nullptr;

  void Load();
  void Say(const std::string &s);  // a log line for Flush, at most 10 a minute (m_ held)
  void Roll(uint64_t now);         // Say's next minute, once it's due: how many it dropped (m_ held)
  std::string Json() const;  // relations.json's contents, m_ held
};
