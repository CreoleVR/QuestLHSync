// qlhs_stat: the running driver's status (the dashboard's shared memory, qlhs_status.h) as text, and its buttons.
//   qlhs_stat [seconds [every_s]]   a line per sample: state, headset and why it was taken, the fit, each station's
//                                   support and when the cameras last saw it
//   qlhs_stat cmd <n>               a dashboard command (QlhsCmd): 2 pause, 3 resume, 4 record on, 5 record off,
//                                   6 force recalibrate, 7 switch headset
#include <windows.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../common/qlhs_status.h"

static QlhsStatus *g;

static bool Read(QlhsStatus &o) {
  for (int i = 0; i < 100; i++) {
    int32_t s = g->seq;
    if (s & 1) { Sleep(0); continue; }
    MemoryBarrier();
    memcpy(&o, (const void *)g, sizeof o);
    MemoryBarrier();
    if (g->seq == s) return o.magic == QLHS_MAGIC && o.version == QLHS_VERSION;
  }
  return false;
}

int main(int argc, char **argv) {
  HANDLE m = OpenFileMappingW(FILE_MAP_ALL_ACCESS, FALSE, QLHS_SHM_NAME);
  if (!m) { printf("no driver status: is SteamVR running with QuestLHSync?\n"); return 1; }
  g = (QlhsStatus *)MapViewOfFile(m, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(QlhsStatus));
  if (!g) return 1;
  if (argc >= 3 && !strcmp(argv[1], "cmd")) {
    g->cmd = atoi(argv[2]);
    MemoryBarrier();
    InterlockedIncrement((volatile LONG *)&g->cmd_seq);
    printf("sent command %d\n", atoi(argv[2]));
    return 0;
  }
  double dur = argc > 1 ? atof(argv[1]) : 0, every = argc > 2 ? atof(argv[2]) : 5;
  static const char *st[] = {"starting", "no_hmd", "searching", "connecting", "no_cameras", "no_stations",
                             "acquiring", "locked", "disabled", "wired"};
  DWORD t0 = GetTickCount();
  for (;;) {
    QlhsStatus s;
    if (!Read(s)) { printf("status unreadable: the driver is another version (%u, this tool %u)\n", g->version, QLHS_VERSION); return 1; }
    printf("%5.0fs %-10s %s %s why=\"%s\" headsets %d | cams %.0f/s spots %.0f/s used %.1f/s rtt %.1fms expo %.1fms%s "
           "still %d | locked %d cond %d med %.3f deg n %d yaw %+.2f t [%.2f %.2f %.2f] for %.0fs lag %.1fcm rec %d |",
           (GetTickCount() - t0) / 1000.0, s.state >= 0 && s.state < 10 ? st[s.state] : "?", s.headset, s.headset_addr,
           s.headset_why, s.nheadsets, s.cam_fps, s.spot_rate, s.sight_rate, s.rtt_ms, s.expo_ms,
           s.expo_learned ? " learned" : "", s.head_still, s.locked, s.cond, s.med_deg, s.nfit, s.yaw_deg, s.t[0], s.t[1],
           s.t[2], s.locked_for, s.lag_cm, s.recording);
    for (int i = 0; i < s.nst; i++)
      printf(" %s%s:%d seen %.0fs %.1fm", s.st[i].serial + 4, s.st[i].anchor ? "*" : "", s.st[i].support,
             s.st[i].last_seen, s.st[i].dist);
    printf("\n");
    fflush(stdout);
    if ((GetTickCount() - t0) / 1000.0 >= dur) break;
    Sleep((DWORD)(every * 1000));
  }
  return 0;
}
