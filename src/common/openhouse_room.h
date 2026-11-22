// The driver's room, published for tools that align the lighthouse frame to something
// else (QuestLHSync): shared memory, the same layout on both systems.
//  - Linux: POSIX shared memory /openhouse-room-<uid> (shm_open; /dev/shm/openhouse-room-<uid>).
//  - Windows: the named file mapping "Local\\openhouse-room" (OpenFileMappingA(FILE_MAP_READ,
//    FALSE, OPENHOUSE_ROOM_WIN32_NAME) + MapViewOfFile), in the user's logon session.
//
// Poses are in the host's raw tracking frame (what the base stations and devices are
// reported in: WorldFromDriver * pose). "settled" says the room has stopped moving: no
// base has moved for kSettleTime and every visible base is placed. "epoch" changes when
// a settled room differs from the last settled one (a base moved, or was added): what
// was measured against the old poses is stale. A reader takes the room only while
// "beat" is fresh (the driver writes it every 0.25 s) and the pid is alive. beat's clock:
// CLOCK_MONOTONIC on Linux; on Windows QueryPerformanceCounter / QueryPerformanceFrequency,
// in seconds (system-wide, so any process can compare it with its own reading).
//
// Plain C layout, so other projects can keep a copy of this header.
//
// A copy of openhouse include/lh/room_shm.h: keep the two the same.
#pragma once
#include <stdint.h>

#define OPENHOUSE_ROOM_MAGIC 0x4D52484Cu  // "LHRM"
#define OPENHOUSE_ROOM_VERSION 1
#define OPENHOUSE_ROOM_MAX_BASES 16

typedef struct {
  char serial[32];       // "LHB-XXXXXXXX"
  int32_t visible;       // seen this session
  int32_t placed;        // pose known
  int32_t origin;        // the room's origin base
  int32_t pad;
  double p[3], q[4];     // raw frame, q: w x y z
  double sigma;          // m, position uncertainty
} openhouse_room_base;

typedef struct {
  uint32_t magic, version;
  uint32_t seq;          // seqlock: odd while writing (use atomics on both sides)
  int32_t pid;
  double beat;           // monotonic s (see above); 0: the driver closed the room
  uint64_t epoch;        // 0: never settled
  int32_t settled;
  int32_t count;
  char room[64];
  openhouse_room_base bases[OPENHOUSE_ROOM_MAX_BASES];
} openhouse_room;

#define OPENHOUSE_ROOM_PATH_PREFIX "/openhouse-room-"
#define OPENHOUSE_ROOM_WIN32_NAME "Local\\openhouse-room"
