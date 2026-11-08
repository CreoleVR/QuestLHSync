// lhsyncd's Quest side (src/headset/headset.h): Android properties, Meta's camera calibration in /persist/calibration,
// and lhsight as a Frida script (frida-inject + lhsight.js, the headset's cameras filled in) inside the sensors HAL: it
// reads the camera rings and writes nothing.
#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <sys/types.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#include <android/log.h>
#include <android/looper.h>
#include <android/sensor.h>

#include "headset.h"

#define HAL "vendor.oculus.hardware.sensors@1.0-service"
#define RUNDIR "/dev/.questlhsync"

const char *const hs_capture_name = "frida";
static const char *moddir = "/data/adb/modules/questlhsync";

void hs_log(const char *line) { __android_log_print(ANDROID_LOG_INFO, "QuestLHSync", "%s", line); }

void hs_device_info(char *serial, char *model, char *fw, size_t n) {
  (void)n;  // the core's fields hold PROP_VALUE_MAX
  __system_property_get("ro.serialno", serial);
  __system_property_get("ro.product.model", model);
  __system_property_get("ro.build.version.incremental", fw);
  if (!model[0]) strcpy(model, "Quest");
}

// the camera calibration: the newest online-refined file, else the factory one
static int calibration(char *path, size_t n) {
  const char *dir = "/persist/calibration/online";
  DIR *d = opendir(dir);
  time_t best = 0;
  path[0] = 0;
  if (d) {
    struct dirent *e;
    while ((e = readdir(d))) {
      if (e->d_name[0] == '.') continue;
      char p[512];
      struct stat st;
      snprintf(p, sizeof p, "%s/%s", dir, e->d_name);
      if (stat(p, &st) || !S_ISREG(st.st_mode) || st.st_size < 1000) continue;
      if (st.st_mtime >= best) { best = st.st_mtime; snprintf(path, n, "%s", p); }
    }
    closedir(d);
  }
  if (!path[0]) snprintf(path, n, "/persist/calibration/camera_calibration.json");
  return access(path, R_OK) == 0;
}

// a whole file, NUL-terminated (malloc'd), or NULL
static char *read_file(const char *path, size_t *len) {
  FILE *f = fopen(path, "rb");
  if (!f) return NULL;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  char *buf = malloc(n > 0 ? n + 1 : 1);
  size_t got = n > 0 ? fread(buf, 1, n, f) : 0;
  fclose(f);
  buf[got] = 0;
  *len = got;
  return buf;
}

int hs_calibration(char **data, size_t *len, char *name, size_t n) {
  char path[512];
  *data = calibration(path, sizeof path) ? read_file(path, len) : NULL;
  if (!*data) { snprintf(name, n, "no camera calibration in /persist/calibration"); return -1; }
  snprintf(name, n, "%s", strstr(path, "/calibration/") ? strstr(path, "/calibration/") + 13 : path);
  return 0;
}

// ---------------------------------------------------------------- the cameras lhsight reads, from the calibration
// "CameraCalibration": [{"Id": "2", "SensorType": "OV7251", "Shutter": {"Type": "Global"}, "ImageSize": [640, 480],
// ...}, ...] -> "id:w:h:scan,..." for the global-shutter (tracking) cameras. scan 1 for the side cameras (OV7251):
// the Quest Pro's front pair (OG01A) only ever saw other lights, and the Quest 3 has the same four sensors. A headset
// without OV7251s scans every tracking camera.
static const char *skip_ws(const char *p, const char *e) {
  while (p < e && (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ',')) p++;
  return p;
}

static const char *jkey(const char *b, const char *e, const char *key) {  // just past "key": inside [b, e), or NULL
  char k[64];
  int n = snprintf(k, sizeof k, "\"%s\"", key);
  const char *p = memmem(b, e - b, k, n);
  if (!p) return NULL;
  for (p += n; p < e && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r'); p++) {}
  if (p >= e || *p != ':') return NULL;
  for (p++; p < e && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r'); p++) {}
  return p < e ? p : NULL;
}

static const char *jclose(const char *p, const char *e) {  // past the object/array starting at p, or NULL
  int depth = 0;
  for (; p < e; p++) {
    if (*p == '"') {
      for (p++; p < e && *p != '"'; p++)
        if (*p == '\\') p++;
      continue;
    }
    if (*p == '{' || *p == '[') depth++;
    else if ((*p == '}' || *p == ']') && --depth == 0) return p + 1;
  }
  return NULL;
}

static int camera_list(const char *js, size_t len, char *out, size_t cap) {
  const char *e = js + len, *p = jkey(js, e, "CameraCalibration"), *end;
  struct { int id, w, h, side; } c[16];
  int n = 0, nside = 0;
  out[0] = 0;
  if (!p || *p != '[' || !(end = jclose(p, e))) return 0;
  for (p++; (p = skip_ws(p, end)) < end && *p == '{' && n < 16;) {
    const char *oe = jclose(p, end);
    if (!oe) break;
    const char *id = jkey(p, oe, "Id"), *sz = jkey(p, oe, "ImageSize"), *st = jkey(p, oe, "SensorType");
    if (id && sz && *sz == '[' && !memmem(p, oe - p, "\"Rolling\"", 9)) {
      char *q;
      long w = strtol(sz + 1, &q, 10), h = strtol(skip_ws(q, oe), NULL, 10);
      if (w > 0 && h > 0 && w <= 4096 && h <= 4096) {
        c[n].id = atoi(*id == '"' ? id + 1 : id);
        c[n].w = (int)w;
        c[n].h = (int)h;
        c[n].side = st && !strncmp(st, "\"OV7251\"", 8);
        nside += c[n].side;
        n++;
      }
    }
    p = oe;
  }
  size_t o = 0;
  for (int i = 0; i < n && o + 32 < cap; i++)
    o += snprintf(out + o, cap - o, "%s%d:%d:%d:%d", i ? "," : "", c[i].id, c[i].w, c[i].h, nside ? c[i].side : 1);
  return n;
}

// the sensors HAL: its Quest Pro name, else any vendor.oculus.hardware.sensors* (another version or OS build)
static pid_t hal_pid(char *name, size_t cap) {
  DIR *d = opendir("/proc");
  if (!d) return 0;
  struct dirent *e;
  pid_t found = 0, other = 0;
  snprintf(name, cap, "%s", HAL);
  while (!found && (e = readdir(d))) {
    if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
    char p[64], cmd[256];
    snprintf(p, sizeof p, "/proc/%s/cmdline", e->d_name);
    int fd = open(p, O_RDONLY);
    if (fd < 0) continue;
    ssize_t n = read(fd, cmd, sizeof cmd - 1);
    close(fd);
    if (n <= 0) continue;
    cmd[n] = 0;
    const char *base = strrchr(cmd, '/') ? strrchr(cmd, '/') + 1 : cmd;
    if (!strcmp(base, HAL)) found = atoi(e->d_name);
    else if (!other && !strncmp(base, "vendor.oculus.hardware.sensors", 30)) {
      other = atoi(e->d_name);
      snprintf(name, cap, "%s", base);
    }
  }
  closedir(d);
  if (found) snprintf(name, cap, "%s", HAL);
  return found ? found : other;
}

// lhsight.js with this headset's cameras filled in, in the run dir: its path in out, the cameras (or why not) in note;
// out "" on failure, with the reason in note
static void prepare_script(char *out, size_t cap, char *note, size_t ncap) {
  char src[512], path[512], cams[512] = "";
  size_t len, cal_len;
  out[0] = 0;
  snprintf(src, sizeof src, "%s/lhsight.js", moddir);
  char *js = read_file(src, &len);
  if (!js) { snprintf(note, ncap, "can't read %s", src); return; }
  char *cal = calibration(path, sizeof path) ? read_file(path, &cal_len) : NULL;
  int n = cal ? camera_list(cal, cal_len, cams, sizeof cams) : 0;
  free(cal);
  if (n) snprintf(note, ncap, "tracking cameras (id:width:height:scan) %s", cams);
  else snprintf(note, ncap, "no tracking cameras in the camera calibration: trying the Quest Pro's");
  char *at = strstr(js, "@CAMS@");
  snprintf(out, cap, "%s/lhsight.js", RUNDIR);
  FILE *f = fopen(out, "wb");
  if (!f) { snprintf(note, ncap, "can't write %s", out); out[0] = 0; free(js); return; }
  if (at && n) {
    fwrite(js, 1, at - js, f);
    fputs(cams, f);
    fputs(at + 6, f);
  } else {
    fwrite(js, 1, len, f);
  }
  fclose(f);
  chmod(out, 0644);
  free(js);
}

// a frida-inject still running a lhsight.js (this daemon SIGKILLed, an old adb session) would make the next start a
// second injection: stop those, and only those
static int is_lhsight(const char *pid) {
  char p[64], cmd[1024];
  snprintf(p, sizeof p, "/proc/%s/cmdline", pid);
  int fd = open(p, O_RDONLY);
  if (fd < 0) return 0;
  ssize_t len = read(fd, cmd, sizeof cmd - 1);
  close(fd);
  if (len <= 0) return 0;
  cmd[len] = 0;
  const char *base = strrchr(cmd, '/') ? strrchr(cmd, '/') + 1 : cmd;
  if (strcmp(base, "frida-inject")) return 0;
  for (ssize_t i = strlen(cmd) + 1; i < len; i += strlen(cmd + i) + 1) {
    size_t l = strlen(cmd + i);
    if (l >= 10 && !strcmp(cmd + i + l - 10, "lhsight.js")) return 1;
  }
  return 0;
}

static void kill_strays(void) {
  DIR *d = opendir("/proc");
  if (!d) return;
  struct dirent *e;
  char found[16][16];
  int n = 0;
  while (n < 16 && (e = readdir(d)))
    if (e->d_name[0] >= '0' && e->d_name[0] <= '9' && is_lhsight(e->d_name)) snprintf(found[n++], 16, "%s", e->d_name);
  closedir(d);
  for (int i = 0; i < n; i++) {
    char m[96];
    snprintf(m, sizeof m, "stopping a stray lhsight (frida-inject pid %s)", found[i]);
    hs_log(m);
    kill(atoi(found[i]), SIGTERM);
  }
  for (int t = 0; n && t < 30; t++) {
    usleep(100000);
    int left = 0;
    for (int i = 0; i < n; i++) left += is_lhsight(found[i]);
    if (!left) return;
  }
  for (int i = 0; i < n; i++)
    if (is_lhsight(found[i])) kill(atoi(found[i]), SIGKILL);
}

void hs_init(int argc, char **argv) {
  if (argc > 1) moddir = argv[1];
  kill_strays();
  char m[600];
  snprintf(m, sizeof m, "module %s", moddir);
  hs_log(m);
}

pid_t hs_capture_start(int out_fd, char *msg, size_t n) {
  char hal_name[256];
  pid_t hal = hal_pid(hal_name, sizeof hal_name);
  if (!hal) { snprintf(msg, n, "sensors HAL not running"); return 0; }
  char frida[512], script[512], pidstr[16], note[600];
  mkdir(RUNDIR, 0755);
  mkdir(RUNDIR "/tmp", 0755);
  prepare_script(script, sizeof script, note, sizeof note);
  if (!script[0]) { snprintf(msg, n, "%s", note); return 0; }
  snprintf(frida, sizeof frida, "%s/frida-inject", moddir);
  snprintf(pidstr, sizeof pidstr, "%d", hal);
  pid_t pid = fork();
  if (pid == 0) {
    setsid();
    dup2(out_fd, 1);
    dup2(out_fd, 2);
    int nul = open("/dev/null", O_RDONLY);
    dup2(nul, 0);
    for (int fd = 3; fd < 1024; fd++) close(fd);
    setenv("TMPDIR", RUNDIR "/tmp", 1);  // frida's scratch files: tmpfs, never /data/local/tmp
    chdir(RUNDIR);
    execl(frida, "frida-inject", "-p", pidstr, "-s", script, (char *)NULL);
    _exit(127);
  }
  if (pid < 0) return -1;
  snprintf(msg, n, "lhsight started in %s (pid %d); %s", hal_name, hal, note);
  return pid;
}

// ---------------------------------------------------------------- the accelerometer (headset.h: hs_gravity)
// Meta's HMD IMU, as Android's sensor service hands it out (oculus.sensor.hmd_imu.1, type 65537, no permission needed):
// data[0] the IMU's temperature, data[1..3] the accelerometer (m/s^2, the IMU's axes: the calibration's DeviceFromImu
// turns them into the device's), data[4..6] the gyro, about 1000 events a second. The timestamps aren't Android's
// usual CLOCK_BOOTTIME ns but the IMU's own clock (microseconds on a Quest Pro): their unit is told from how fast they
// run against CLOCK_MONOTONIC over the first 2 s, and their offset is the smallest delay seen from a sample to its
// reading (over 5 s windows, so the two clocks may drift)
#define HMD_IMU 65537
static ASensorManager *gsm;
static ASensorEventQueue *gq;
static int gnone;
static double gunit;                     // timestamp units a second (0: not known yet)
static double gu_ts0, gu_m0 = -1;        // the first sample's timestamp and when it was read, to tell the unit
static double goff, gmin, gmin_since;   // CLOCK_MONOTONIC - the sample's time (s): in use, this window's least

static double clk(clockid_t c) {
  struct timespec t;
  clock_gettime(c, &t);
  return t.tv_sec + t.tv_nsec / 1e9;
}

int hs_gravity(double sum[3], double *tsum) {
  if (!gq) {
    if (gnone) return 0;
    gsm = ASensorManager_getInstanceForPackage("com.questlhsync");
    const ASensor *imu = NULL;
    ASensorList list;
    int n = gsm ? ASensorManager_getSensorList(gsm, &list) : 0;
    for (int i = 0; i < n && !imu; i++)
      if (ASensor_getType(list[i]) == HMD_IMU) imu = list[i];
    ALooper *lp = ALooper_forThread();
    if (!lp) lp = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    if (imu) gq = ASensorManager_createEventQueue(gsm, lp, 1, NULL, NULL);
    if (!gq || ASensorEventQueue_registerSensor(gq, imu, 10000, 0) < 0) {
      if (gq) ASensorManager_destroyEventQueue(gsm, gq);
      gq = NULL;
      gnone = 1;
      hs_log("no headset IMU to read (oculus.sensor.hmd_imu): no G lines");
      return 0;
    }
    gunit = 0;
    gu_m0 = -1;
    goff = 0;
    gmin = 1e30;
    gmin_since = clk(CLOCK_MONOTONIC);
    hs_log("headset IMU on: G lines");
  }
  ASensorEvent ev[64];
  ssize_t k;
  int got = 0;
  double now = clk(CLOCK_MONOTONIC);
  while ((k = ASensorEventQueue_getEvents(gq, ev, 64)) > 0)
    for (ssize_t i = 0; i < k; i++) {
      if (ev[i].type != HMD_IMU) continue;
      double ts = (double)ev[i].timestamp;
      if (gu_m0 < 0) { gu_ts0 = ts; gu_m0 = now; }
      if (!gunit) {
        if (now - gu_m0 < 2) continue;  // the unit isn't known yet: these go
        double r = (ts - gu_ts0) / (now - gu_m0), best = 1e3;  // ms, us or ns, whichever it runs nearest to
        for (double u = 1e6; u <= 1e9; u *= 1e3)
          if (fabs(log10(r / u)) < fabs(log10(r / best))) best = u;
        gunit = best;
        goff = now - ts / gunit;
        hs_log(gunit == 1e3 ? "headset IMU clock: ms" : gunit == 1e6 ? "headset IMU clock: us" : "headset IMU clock: ns");
      }
      double d = now - ts / gunit;  // how late it's read, plus the clocks' offset: the least is the offset
      if (d < gmin) gmin = d;
      if (d < goff) goff = d;
      for (int j = 0; j < 3; j++) sum[j] += ev[i].data[1 + j];
      *tsum += ts / gunit + goff;
      got++;
    }
  if (now - gmin_since >= 5 && gmin < 1e29) {  // the clocks may drift apart: the offset is the last window's least
    goff = gmin;
    gmin = 1e30;
    gmin_since = now;
  }
  return got;
}

void hs_gravity_stop(void) {
  if (!gq) return;
  ASensorManager_destroyEventQueue(gsm, gq);
  gq = NULL;
}
