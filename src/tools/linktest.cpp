// qlhs_linktest: the driver's HeadsetLink alone against the real network, without SteamVR: which headset it takes,
// why (the page's "why"), and Switch headset. It connects for real, so the headset's lhsight runs meanwhile.
//   qlhs_linktest <state dir> <headset.txt copy> <streamer driver> <seconds> [switch_at_s]
//   e.g. qlhs_linktest scratch\state scratch\headset.txt oculus_virtualdesktop 60 30
// Run it on the PC that streams the headset, with the streamer connected: "streams with this PC" should name it.
#include <winsock2.h>
#include <windows.h>

#include <cstdio>
#include <cstdlib>
#include <string>

#include "../driver/net.h"
#include "../driver/sync.h"

int main(int argc, char **argv) {
  if (argc < 5) {
    fprintf(stderr, "usage: qlhs_linktest <state dir> <headset.txt copy> <streamer driver> <seconds> [switch_at_s]\n");
    return 2;
  }
  std::string dir = argv[1], mem = argv[2], streamer = argv[3];
  double dur = atof(argv[4]), sw = argc > 5 ? atof(argv[5]) : -1;
  double t0 = QpcNow();
  auto logfn = [&](const std::string &s) { printf("%6.1f  %s\n", QpcNow() - t0, s.c_str()); fflush(stdout); };
  SyncConfig cfg;
  cfg.dir = dir;
  Sync sync(cfg, logfn);
  HeadsetLink link(&sync, logfn, [](double, const std::string &) {});
  link.SetMemory(mem);
  link.SetFamily("");  // any kind, as with Virtual Desktop's "Meta Quest 3"
  link.SetStreamer(streamer);
  link.SetWanted(true);
  link.Start();
  double next = 2;
  bool switched = false;
  while (QpcNow() - t0 < dur) {
    double t = QpcNow() - t0;
    sync.Tick(QpcNow());
    if (t >= next) {
      next += 5;
      printf("%6.1f  [%s] %s %s  why=\"%s\"  headsets=%d\n", t,
             link.state() == HeadsetLink::kConnected ? "connected" : "searching", link.model().c_str(),
             link.addr().c_str(), link.why().c_str(), link.headsets());
      fflush(stdout);
    }
    if (sw > 0 && !switched && t >= sw) {
      switched = true;
      printf("%6.1f  -- pressing Switch headset\n", t);
      link.NextHeadset();
    }
    Sleep(50);
  }
  link.Stop();
  return 0;
}
