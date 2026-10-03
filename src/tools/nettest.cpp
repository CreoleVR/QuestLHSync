// qlhs_nettest: the driver's HeadsetLink + Sync against fake_lhsyncd.py, in real time. HMD poses come from the same
// log, placed on this PC's clock where the fake headset's clock puts the camera frames.
//   qlhs_nettest <log> --dir <state dir> [--start s] [--dur s] [--mem <headset.txt>] [--family "Quest 3"]
// --family: SteamVR's HMD kind (default Quest Pro), to test that headsets of another kind are skipped
#include <winsock2.h>
#include <windows.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "../driver/net.h"
#include "../driver/sync.h"

int main(int argc, char **argv) {
  std::string log, dir, mem, family = "Quest Pro";
  double start = 0, dur = 240;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--dir" && i + 1 < argc) dir = argv[++i];
    else if (a == "--mem" && i + 1 < argc) mem = argv[++i];  // no manual host: only the remembered address
    else if (a == "--start" && i + 1 < argc) start = atof(argv[++i]);
    else if (a == "--dur" && i + 1 < argc) dur = atof(argv[++i]);
    else if (a == "--family" && i + 1 < argc) family = argv[++i];
    else log = a;
  }
  struct P { double t; Quat q; V3 p; };
  std::vector<P> poses;
  std::map<std::string, std::pair<V3, M3>> raw;
  double pc0 = 0;
  {
    FILE *f = fopen(log.c_str(), "rb");
    if (!f) return 1;
    static char line[1 << 16];
    while (fgets(line, sizeof line, f)) {
      char *sp = strchr(line, ' ');
      if (!sp || !sp[1] || sp[2] != ' ') continue;
      double pc = atoll(line) / 1e9;
      const char *rest = sp + 3;
      if (sp[1] == 'F' && !pc0) pc0 = pc + start;
      if (sp[1] == 'P') {
        int idx;
        double m[12];
        if (sscanf(rest, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &idx, &m[0], &m[1], &m[2], &m[3], &m[4],
                   &m[5], &m[6], &m[7], &m[8], &m[9], &m[10], &m[11]) == 13 && idx == 0) {
          M3 R;
          for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) R.m[r][c] = m[r * 4 + c];
          poses.push_back({pc, ToQuat(R), V3{m[3], m[7], m[11]}});
        }
      } else if (sp[1] == 'S' && (!pc0 || pc < pc0 + dur)) {
        char serial[64];
        double v[7];
        if (sscanf(rest, "%63s %lf %lf %lf %lf %lf %lf %lf", serial, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) == 8)
          raw[serial] = {V3{v[0], v[1], v[2]}, ToM3(Quat{v[3], v[4], v[5], v[6]})};
      }
    }
    fclose(f);
  }
  printf("%zu HMD poses, %zu stations\n", poses.size(), raw.size());
  fflush(stdout);
  std::atomic<double> qpc0{0};
  auto logfn = [&](const std::string &s) {
    double q;
    if (sscanf(s.c_str(), "headset: fake qpc0 %lf", &q) == 1) qpc0 = q;
    printf("%8.1f  %s\n", qpc0 > 0 ? QpcNow() - qpc0 : 0.0, s.c_str());
    fflush(stdout);
  };
  SyncConfig cfg;
  cfg.dir = dir;
  Sync sync(cfg, logfn);
  sync.SetStreamer("nettest");
  sync.SetStationsRaw(raw);
  HeadsetLink link(&sync, logfn, [](double, const std::string &) {});
  link.SetPreferred("FAKE0001");
  link.SetFamily(family);  // a real headset on the network answers discovery too
  if (mem.empty()) link.SetHosts({"127.0.0.1"});
  else link.SetMemory(mem);
  link.SetWanted(true);
  link.Start();
  while (qpc0 == 0) {
    if (link.state() == HeadsetLink::kConnected && !link.serial().empty() && link.serial() != "FAKE0001") {
      printf("connected to a real headset (%s), not the fake: stopping\n", link.serial().c_str());
      link.Stop();
      return 3;
    }
    Sleep(10);
  }
  std::atomic<bool> run{true};
  std::thread feeder([&] {  // HMD poses at their time on this clock, like the driver's hook sees them
    size_t i = 0;
    while (i < poses.size() && poses[i].t < pc0 - 3) i++;
    for (; i < poses.size() && run; i++) {
      double at = qpc0 + (poses[i].t - pc0);
      double w = at - QpcNow();
      if (w > 0.002) Sleep((DWORD)(w * 1000));
      sync.OnHmdPose(at, poses[i].q, poses[i].p);
    }
  });
  double next_print = QpcNow() + 5;
  while (QpcNow() - qpc0 < dur) {
    double now = QpcNow();
    sync.Tick(now);
    if (now >= next_print) {
      next_print += 10;
      auto st = sync.GetStatus(now);
      std::string sts;
      for (auto &e : st.st) sts += " " + e.serial + ":" + std::to_string(e.support);
      printf("%8.1f  [%s] rtt %.1f ms cams %.0f/s sightings %.1f/s expo %.1f ms%s | %s yaw %+.3f t [%.1f %.1f %.1f] n %d med %.3f%s\n",
             now - qpc0, link.state() == HeadsetLink::kConnected ? "connected" : "not connected", st.rtt * 1000, st.cam_fps,
             st.sight_rate, st.expo * 1000, st.timing_learned ? " learned" : "", st.has_x ? "x" : "-", st.x[0] * kDeg,
             st.x[1] * 100, st.x[2] * 100, st.x[3] * 100, st.n, st.med, sts.c_str());
      fflush(stdout);
    }
    Sleep(50);
  }
  run = false;
  feeder.join();
  link.Stop();
  return 0;
}
