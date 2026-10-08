// QuestLHSync driver <-> dashboard overlay: shared memory "Local\QuestLHSync".
// The driver publishes its status under a seqlock (seq odd while writing); the overlay sends commands by
// writing cmd and then incrementing cmd_seq.
#pragma once
#include <stdint.h>

#define QLHS_SHM_NAME L"Local\\QuestLHSync"
#define QLHS_OVERLAY_MUTEX L"Local\\QuestLHSyncOverlay"
#define QLHS_MAGIC 0x53484C51u  // "QLHS"
#define QLHS_VERSION 2
#define QLHS_RELEASE "1.16"  // magisk/build_module.py reads it

enum QlhsState : int32_t {
  QLHS_STARTING = 0,
  QLHS_NO_HMD,       // SteamVR's headset isn't a Quest Pro, 3, 3S or Steam Frame (or there's none yet)
  QLHS_SEARCHING,    // no QuestLHSync headset answers on the network
  QLHS_CONNECTING,
  QLHS_NO_CAMERAS,   // connected, but no camera frames come in
  QLHS_NO_STATIONS,  // fewer than 2 base stations in SteamVR
  QLHS_ACQUIRING,
  QLHS_LOCKED,
  QLHS_DISABLED,     // driver_questlhsync.enable is false
  QLHS_WIRED,        // SteamVR's headset is wired, not streamed: the driver is off and the dashboard closes
};

enum QlhsCmd : int32_t {
  QLHS_CMD_NONE = 0,
  QLHS_CMD_REACQUIRE = 1,  // retired in 1.7 (it only threw the sightings away): ignored
  QLHS_CMD_PAUSE = 2,
  QLHS_CMD_RESUME = 3,
  QLHS_CMD_RECORD_ON = 4,
  QLHS_CMD_RECORD_OFF = 5,
};

#pragma pack(push, 8)
struct QlhsStation {
  char serial[32];
  int32_t support;    // sightings that fit it now
  int32_t anchor;     // 1: the reference frame's anchor station
  int32_t measured;   // 1: sits where stations.json measured it
  int32_t pad;
  double last_seen;   // s since the cameras last saw it, < 0: not yet
  double dist;        // m from the headset
};

struct QlhsStatus {
  uint32_t magic, version;
  volatile int32_t seq;
  int32_t state;
  double updated;       // driver's clock (s); the overlay compares two reads for staleness
  char hmd[64];         // SteamVR's headset model
  char hmd_system[32];  // its tracking system (streamer)
  char headset[48];     // the headset's model ("Quest Pro", "Steam Frame"), never its serial
  char headset_addr[48];
  char headset_fw[48];
  double cam_fps;       // camera frames/s: the Quest's side cameras (all four from module v1.0), the Frame's four
  double sight_rate;    // bright spots/s the alignment uses
  double spot_rate;     // bright spots/s the cameras see
  int32_t head_still, pad2;  // 1: the headset hasn't moved for 2 s, its frames wait; 2: for 10 s while its cameras
                             // see the room move (SteamVR isn't getting the head's motion)
  double rtt_ms;        // network round trip (clock sync)
  double expo_ms;       // frame grid -> pose time
  int32_t expo_learned, pad0;
  int32_t nst, pad1;
  QlhsStation st[8];
  int32_t locked, cond;
  double yaw_deg, t[3];
  double med_deg;       // median sighting error
  int32_t nfit, paused;
  double locked_for;    // s since the last acquisition, < 0: never
  double lag_cm;        // applied vs solved at the head (slewing)
  int32_t recording;
  uint32_t nlog;        // messages written so far; log[i % 8]
  char log[8][120];
  volatile int32_t cmd_seq;  // overlay -> driver
  int32_t cmd;
};
#pragma pack(pop)
