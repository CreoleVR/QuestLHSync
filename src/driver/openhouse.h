// openhouse (an open replacement for SteamVR's lighthouse driver) publishes its room in shared memory
// (src/common/openhouse_room.h): the base stations as it has them now. While openhouse runs, the stations come from
// its room rather than from the runtime's devices, and the solver waits while they settle; without it (SteamVR's own
// lighthouse driver), nothing changes.
#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>

#include "mathx.h"

class Sync;

class OpenhouseRoom {
 public:
  // state: the file the room's id is kept in, so a room reset while we weren't running counts too
  OpenhouseRoom(std::string state, std::function<void(const std::string &)> log)
      : state_(std::move(state)), log_(std::move(log)) {}
  // raw: the runtime's stations, replaced by openhouse's while its room is up. Tells sync where they came from.
  void Update(Sync &sync, std::map<std::string, std::pair<V3, M3>> &raw, double now);

 private:
  std::string state_;
  std::function<void(const std::string &)> log_;
  bool ours_ = false;
  std::string id_;                                 // the room the alignment belongs to
  std::map<std::string, std::pair<V3, M3>> ref_;   // the used stations' places the sightings are against
  uint64_t epoch_ = 0;
  double still_since_ = 0;
};
