// qlhs_replay: run a QuestLHSync recording through the driver's Sync, as if live.
//   qlhs_replay <log> --calib <camera calibration json> --dir <state dir> [--out file] [--learn] [--expo s]
//               [--learn-grid] [--pings]
// Logs without clock round trips use the arrival envelope clock and EXPO 0.020. --pings: the round trips alone, like the
// live driver (QuestLHSync recordings have them).
// --learn: round-trip-less logs still learn the timing (starting from --expo), to test the estimator.
// --learn-grid: learn the cameras' frame period as for headsets other than the Quest Pro, to test that.
// --channel SERIAL=N: a base station's channel, for older logs.
// --xf file: the transform applied to the lighthouse devices (raw -> Quest: pc_ns qw qx qy qz tx ty tz), every 0.25 s
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "../driver/sync.h"

int main(int argc, char **argv) {
  std::string log, calib, dir, outp, xfp;
  bool learn = false, learn_grid = false, pings = false;
  double expo = -1;
  std::map<std::string, int> chans;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--channel" && i + 1 < argc) {
      std::string v = argv[++i];
      size_t eq = v.find('=');
      if (eq != std::string::npos) chans[v.substr(0, eq)] = atoi(v.c_str() + eq + 1);
    } else if (a == "--calib" && i + 1 < argc) calib = argv[++i];
    else if (a == "--dir" && i + 1 < argc) dir = argv[++i];
    else if (a == "--out" && i + 1 < argc) outp = argv[++i];
    else if (a == "--xf" && i + 1 < argc) xfp = argv[++i];
    else if (a == "--rays" && i + 1 < argc) g_ray_dump = fopen(argv[++i], "w");  // every sighting + the stations
    else if (a == "--expo" && i + 1 < argc) expo = atof(argv[++i]);
    else if (a == "--learn") learn = true;
    else if (a == "--learn-grid") learn_grid = true;
    else if (a == "--pings") pings = true;
    else log = a;
  }
  if (log.empty() || calib.empty()) { fprintf(stderr, "usage: qlhs_replay <log> --calib file --dir statedir [--out f] [--learn] [--expo s]\n"); return 2; }
  std::ifstream cf(calib, std::ios::binary);
  std::stringstream ss;
  ss << cf.rdbuf();
  double t0 = 0, tnow = 0;
  auto logfn = [&](const std::string &s) { printf("%6.0fs    %s\n", tnow - t0, s.c_str()); };
  SyncConfig cfg;
  cfg.dir = dir;
  cfg.arrival_clock = !pings;
  cfg.learn_timing = learn;
  cfg.learn_grid = learn_grid;
  Sync sync(cfg, logfn);
  if (expo >= 0) sync.ForceExpo(expo);
  sync.SetStreamer("replay");
  sync.SetChannels(chans);
  std::string err;
  if (!sync.SetCalibration(ss.str(), &err)) { fprintf(stderr, "calibration: %s\n", err.c_str()); return 1; }
  FILE *out = outp.empty() ? nullptr : fopen(outp.c_str(), "w");
  FILE *xf = xfp.empty() ? nullptr : fopen(xfp.c_str(), "w");
  long ticks = 0;
  FILE *f = fopen(log.c_str(), "rb");
  if (!f) { fprintf(stderr, "can't open %s\n", log.c_str()); return 1; }
  static char line[1 << 16];
  std::map<std::string, std::pair<V3, M3>> raw;
  double next_tick = 0, next_print = 0;
  long nl = 0;
  auto wall = std::chrono::steady_clock::now();
  while (fgets(line, sizeof line, f)) {
    size_t L = strlen(line);
    while (L && (line[L - 1] == '\n' || line[L - 1] == '\r')) line[--L] = 0;
    char *sp = strchr(line, ' ');
    if (!sp || sp[1] == 0 || sp[2] != ' ') continue;
    double pc = atoll(line) / 1e9;
    char kind = sp[1];
    const char *rest = sp + 3;
    if (!t0) { t0 = pc; next_tick = pc; next_print = pc + 10; }
    tnow = pc;
    nl++;
    if (kind == 'P') {
      int idx;
      double m[12];
      if (sscanf(rest, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &idx, &m[0], &m[1], &m[2], &m[3], &m[4], &m[5],
                 &m[6], &m[7], &m[8], &m[9], &m[10], &m[11]) == 13 && idx == 0) {
        M3 R;
        for (int r = 0; r < 3; r++)
          for (int c = 0; c < 3; c++) R.m[r][c] = m[r * 4 + c];
        sync.OnHmdPose(pc, ToQuat(R), V3{m[3], m[7], m[11]});
      }
    } else if (kind == 'S') {
      char serial[64];
      double v[7];
      int ch = 0;
      int got = sscanf(rest, "%63s %lf %lf %lf %lf %lf %lf %lf %d", serial, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &ch);
      if (got >= 8) {
        raw[serial] = {V3{v[0], v[1], v[2]}, ToM3(Quat{v[3], v[4], v[5], v[6]})};
        sync.SetStationsRaw(raw);
        if (got == 9 && ch > 0 && !chans.count(serial)) {  // recorded channels; --channel wins
          chans[serial] = ch;
          sync.SetChannels(chans);
        }
      }
    } else if (kind == 'F') {
      sync.OnLine(pc, sp + 1);
    } else if (kind == 'B') {  // worn and held lighthouse devices (reference frame)
      int dev;
      double v[3];
      if (sscanf(rest, "%d %lf %lf %lf", &dev, &v[0], &v[1], &v[2]) == 4) sync.OnBodyPose(dev, pc, V3{v[0], v[1], v[2]});
    } else if (kind == 'Q') {  // QuestLHSync recordings: clock round trips "Q pc_send pc_recv hs"
      double a, b, h;
      if (sscanf(rest, "%lf %lf %lf", &a, &b, &h) == 3) sync.OnPing(a, b, h);
    }
    while (pc >= next_tick) {
      bool step = sync.WillStep(next_tick);
      Transform x = sync.Tick(next_tick);
      if (xf && x.active && ticks++ % 5 == 0)
        fprintf(xf, "%lld %.7f %.7f %.7f %.7f %.5f %.5f %.5f\n", (long long)(next_tick * 1e9), x.q.w, x.q.x, x.q.y, x.q.z,
                x.t.x, x.t.y, x.t.z);
      if (step && out) {
        auto st = sync.GetStatus(next_tick);
        if (st.has_x)
          fprintf(out, "%lld %.5f %.5f %.5f %.5f\n", (long long)(next_tick * 1e9), st.x[0] * kDeg, st.x[1], st.x[2], st.x[3]);
      }
      next_tick += 0.05;
    }
    if (pc >= next_print) {
      next_print += 10;
      auto st = sync.GetStatus(pc);
      std::string sts;
      for (auto &e : st.st) sts += " " + e.serial + ":" + std::to_string(e.support);
      if (st.has_x)
        printf("%6.0fs yaw %+.3f t [%.1f %.1f %.1f] cm n %d med %.3f%s%s expo %.1f ms\n", pc - t0, st.x[0] * kDeg, st.x[1] * 100,
               st.x[2] * 100, st.x[3] * 100, st.n, st.med, sts.c_str(), st.cond ? " cond" : "", st.expo * 1000);
      else
        printf("%6.0fs unlocked%s\n", pc - t0, sts.c_str());
    }
  }
  auto st = sync.GetStatus(tnow);
  double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count();
  printf("replayed %ld lines (%.0f s of log) in %.1f s; expo %.1f ms%s\n", nl, tnow - t0, secs, st.expo * 1000,
         st.timing_learned ? " (learned)" : "");
  printf("%s\n", Sync::Describe({}, sync.spots()).c_str());
  if (out) fclose(out);
  if (xf) fclose(xf);
  if (g_ray_dump) {
    std::vector<std::string> keys;
    std::vector<V3> S, Z;
    sync.StationsForDump(keys, S);
    for (size_t i = 0; i < keys.size(); i++) fprintf(g_ray_dump, "S %s %.6f %.6f %.6f\n", keys[i].c_str(), S[i].x, S[i].y, S[i].z);
    fprintf(g_ray_dump, "X %.6f %.6f %.6f %.6f %.5f\n", st.x[0] * kDeg, st.x[1], st.x[2], st.x[3], st.expo);
    fclose(g_ray_dump);
  }
  return 0;
}
