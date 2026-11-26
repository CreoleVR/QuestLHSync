#include "openhouse.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <cstdio>
#include <cstring>
#include <fstream>

#include "../common/openhouse_room.h"
#include "net.h"
#include "sync.h"

// openhouse's room from its shared memory, if it's there and alive. A seqlock: the copy is kept only if no write
// began or ended during it.
static bool ReadRoom(openhouse_room &out, double now) {
#ifdef _WIN32
  HANDLE h = OpenFileMappingA(FILE_MAP_READ, FALSE, OPENHOUSE_ROOM_WIN32_NAME);
  if (!h) return false;
  const openhouse_room *r = (const openhouse_room *)MapViewOfFile(h, FILE_MAP_READ, 0, 0, sizeof(openhouse_room));
  CloseHandle(h);  // the view keeps the mapping
  if (!r) return false;
  volatile LONG *seq = (volatile LONG *)&r->seq;
  bool ok = false;
  for (int tries = 0; tries < 8 && !ok; tries++) {
    // a read-only view: an interlocked exchange would write, so the counter is read plainly between full fences
    MemoryBarrier();
    const LONG s = *seq;
    MemoryBarrier();
    if (s & 1) { Sleep(0); continue; }
    memcpy(&out, (const void *)r, sizeof out);
    MemoryBarrier();
    ok = *seq == s;
  }
  UnmapViewOfFile(r);
  if (!ok || out.magic != OPENHOUSE_ROOM_MAGIC || out.version != OPENHOUSE_ROOM_VERSION) return false;
  out.room[sizeof out.room - 1] = 0;
  // beat: QueryPerformanceCounter seconds, as QpcNow
  if (out.beat == 0 || now - out.beat >= 2.0 || out.pid <= 0) return false;
  // openhouse runs in vrserver like us; a pid of another process must still be alive
  if ((DWORD)out.pid == GetCurrentProcessId()) return true;
  HANDLE p = OpenProcess(SYNCHRONIZE, FALSE, (DWORD)out.pid);
  if (!p) return false;
  const bool alive = WaitForSingleObject(p, 0) == WAIT_TIMEOUT;
  CloseHandle(p);
  return alive;
#else
  (void)out;
  (void)now;
  return false;
#endif
}

void OpenhouseRoom::Update(Sync &sync, std::map<std::string, std::pair<V3, M3>> &raw, double now) {
  char line[256];
  openhouse_room r;
  const bool ours = ReadRoom(r, now);
  if (ours != ours_) {
    ours_ = ours;
    if (ours) snprintf(line, sizeof line, "base stations: openhouse's room %s", r.room);
    log_(ours ? line : "base stations: the runtime's (no openhouse room)");
  }
  if (!ours) {
    sync.SetRoomSource(false, true, 0);
    return;
  }
  // Only the well-placed stations: one openhouse is relearning (bumped, or barely seen) is placed with a large sigma
  // and moves while it learns; with it in, its sightings were against a place still changing. openhouse's own
  // settled/epoch cover every station, so one relearning station would pause the alignment and drop every sighting;
  // here they're judged on the stations used only.
  constexpr double kGoodSigma = 0.025;               // m
  constexpr double kMovedM = 0.01, kMovedDeg = 0.3;  // a station this far off moved: older sightings are wrong
  constexpr double kStillFor = 3.0;                  // s the used stations must hold still
  std::map<std::string, std::pair<V3, M3>> own;
  for (int i = 0; i < r.count && i < OPENHOUSE_ROOM_MAX_BASES; i++) {
    const openhouse_room_base &b = r.bases[i];
    if (!b.visible || !b.placed || !(b.sigma <= kGoodSigma)) continue;
    own[std::string(b.serial, strnlen(b.serial, sizeof b.serial))] = {V3{b.p[0], b.p[1], b.p[2]},
                                                                      ToM3(Quat{b.q[0], b.q[1], b.q[2], b.q[3]})};
  }
  raw = own;
  bool moved = false;
  for (auto &kv : own) {
    auto ref = ref_.find(kv.first);
    if (ref == ref_.end()) continue;  // new to the alignment: nothing seen of it yet against another place
    if (norm(kv.second.first - ref->second.first) > kMovedM ||
        RotDeg(ref->second.second * T(kv.second.second)) > kMovedDeg)
      moved = true;
  }
  for (auto &kv : own) ref_[kv.first] = kv.second;  // kept while a station drops out, to compare on return
  if (moved) {
    ++epoch_;
    still_since_ = now;
    snprintf(line, sizeof line, "openhouse station moved: epoch %llu", (unsigned long long)epoch_);
    log_(line);
  }
  if (own.size() < 2) still_since_ = now;
  if (epoch_ == 0 && own.size() >= 2) epoch_ = 1;
  const bool settled = own.size() >= 2 && now - still_since_ >= kStillFor;
  sync.SetRoomSource(true, settled, epoch_);
  // A different room (reset from openhouse's settings, its room file deleted, or the stations moved enough to start
  // over): the alignment and the station places learned against the old one are wrong, so start over.
  const std::string id(r.room, strnlen(r.room, sizeof r.room));
  if (!id.empty() && id != id_) {
    if (id_.empty()) {
      std::ifstream in(state_);
      std::getline(in, id_);
    }
    if (!id_.empty() && id != id_) {
      snprintf(line, sizeof line, "openhouse room changed (%s -> %s): recalibrating", id_.c_str(), id.c_str());
      log_(line);
      sync.ForceRecalibrate();
    }
    id_ = id;
    std::ofstream(state_) << id << "\n";
  }
}
