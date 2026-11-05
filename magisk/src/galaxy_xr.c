// lhsyncd's Galaxy XR side (src/headset/headset.h): Android properties, Google's OpenPX camera calibration, and
// lhsight-gxr (gxrsight.c), the camera reader: a QVR camera client of its own, as Samsung's controller service is one.
// The Galaxy XR uses Samsung S5K931 global-shutter tracking cameras (640x640 Y8), managed by Google's Perception stack
// through android.hardware.perceptionhost instead of Meta's sensors HAL.
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <android/log.h>

#include "headset.h"


// Google's OpenPX calibration: this unit's factory profile, and its records of some cameras' extrinsics (trackingA/B,
// the same as the factory's so far). The profile under /vendor is the one every headset gets, and it isn't this unit:
// compared to the factory's on one headset, principal points up to 12 px off, trackingC/D up to 1.2 deg, so it is only
// the fallback. (The QVR XML under /data/vendor/qvr is an unconfigured default: its rig is 15 deg off what the cameras see.)
#define OPX_PROFILE "/mnt/vendor/efs/device_profile.textproto"
#define OPX_PROFILE_GENERIC "/vendor/etc/calibration_service/device_profile.textproto"
#define OPX_RECORDS "/data/vendor/openpx/calibration/records"
// the firmware the calibration client's ABI is known on (gxrsight.c, latest_transforms)
#define CAL_FIRMWARE "I610UEU2AZF3"
#define RUN_EXE "/data/local/tmp/questlhsync/lhsight-gxr"

const char *const hs_capture_name = "lhsight-gxr";
static const char *moddir = "/data/adb/modules/questlhsync";

void hs_log(const char *line) { __android_log_print(ANDROID_LOG_INFO, "QuestLHSync", "%s", line); }

void hs_device_info(char *serial, char *model, char *fw, size_t n) {
  (void)n;
  __system_property_get("ro.serialno", serial);
  __system_property_get("ro.product.model", model);
  __system_property_get("ro.build.version.incremental", fw);
  if (!model[0]) strcpy(model, "Galaxy_XR");
}

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

// ---------------------------------------------------------------- calibration (OpenPX textproto)
// The tracking cameras lhsight reads, by the ids it sends them as. trackingA/B sit low and look 25 deg down,
// trackingC/D sit high and look level; each pair 23 deg out to either side. This unit runs the _curved profiles (its
// own records refine those).
static const char *const kCam[] = {"trackingA_curved", "trackingB_curved", "trackingC_curved", "trackingD_curved"};
#define NCAM ((int)(sizeof kCam / sizeof kCam[0]))
// The HMD pose's origin in IMU_0 (OpenGL axes: x right, y up, z back), when the live calibration can't say (head_origin).
// First guessed from the eye cameras' places (between the eyes, 2.5 cm of eye relief behind the lenses: 0.0625, -0.035,
// 0.050); measured with a tracker on a headset mount (5.8 min, 680 station sightings): 5.6 cm further back, 1.6 cm
// lower, 1 cm to the right (+-2 cm), which took the solve from 13 cm off the tracker's truth to 4
static const double kEye[3] = {0.0725, -0.051, 0.106};

// the body of a "key {" block at or after s (before e), braces matched
static int opx_sub(const char *s, const char *e, const char *key, const char **bs, const char **be) {
  size_t kl = strlen(key);
  for (const char *p = s; p && p < e; p++) {
    p = strstr(p, key);
    if (!p || p + kl > e) return 0;
    const char *q = p + kl;
    while (q < e && (*q == ' ' || *q == '\t')) q++;
    if (q >= e || *q != '{') continue;
    int depth = 0;
    for (const char *r = q; r < e; r++) {
      if (*r == '{') depth++;
      else if (*r == '}' && --depth == 0) { *bs = q + 1; *be = r; return 1; }
    }
    return 0;
  }
  return 0;
}

// a top-level "name {" block naming camera id
static int opx_block(const char *t, const char *e, const char *name, const char *id, const char **bs, const char **be) {
  char want[96];
  snprintf(want, sizeof want, "camera_id: \"%s\"", id);
  size_t nl = strlen(name);
  for (const char *p = t; p < e; p++) {
    if (p != t && p[-1] != '\n') continue;
    if (strncmp(p, name, nl) || p[nl] != ' ') continue;
    const char *s0, *e0;
    if (!opx_sub(p, e, name, &s0, &e0)) return 0;
    const char *c = strstr(s0, want);
    if (c && c < e0) { *bs = s0; *be = e0; return 1; }
    p = e0;
  }
  return 0;
}

// "field: value" directly in [s, e) (protobuf leaves zeros out: missing reads as 0)
static double opx_num(const char *s, const char *e, const char *field) {
  char want[64];
  snprintf(want, sizeof want, "%s:", field);
  size_t wl = strlen(want);
  for (const char *p = s; p && p < e; p++) {
    p = strstr(p, want);
    if (!p || p >= e) break;
    if (p != s && (p[-1] == '_' || (p[-1] >= 'a' && p[-1] <= 'z'))) continue;  // "x:" inside "max:"
    return strtod(p + wl, NULL);
  }
  return 0;
}

static int opx_vec(const char *s, const char *e, const char *key, const char *const *f, int n, double *out) {
  const char *bs, *be;
  if (!opx_sub(s, e, key, &bs, &be)) return 0;
  for (int i = 0; i < n; i++) out[i] = opx_num(bs, be, f[i]);
  return 1;
}

// a pose block (frame_t_camera, a_t_b): p, and the rotation camera -> IMU_0. OpenPX writes JPL quaternions: as
// Hamilton ones they turn the other way (the cameras' measured relative rotations say so), hence the transpose.
static int opx_pose(const char *s, const char *e, const char *key, double t[3], double R[9]) {
  static const char *const xyz[] = {"x", "y", "z"}, *const wxyz[] = {"w", "x", "y", "z"};
  const char *bs, *be;
  double q[4];
  if (!opx_sub(s, e, key, &bs, &be) || !opx_vec(bs, be, "p", xyz, 3, t) || !opx_vec(bs, be, "q", wxyz, 4, q)) return 0;
  double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!(n > 0.5)) return 0;
  double w = q[0] / n, x = q[1] / n, y = q[2] / n, z = q[3] / n;
  double H[9] = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) R[r * 3 + c] = H[c * 3 + r];
  return 1;
}

// ---------------------------------------------------------------- the head's frame, live
// The HMD pose's frame in IMU_0, asked of Google's calibration service at each connection (in recordings the cameras
// turned against the HMD pose by up to 1.6 deg between sessions, and a fixed calibration can't follow that; whether this
// transform is what moves is to be seen in the logs). It takes a vendor library, so lhsight-gxr's copy asks (RUN_EXE).
// The cameras' own live transforms go to the log only, for comparing with the profile: their names are a guess from the
// records' file names.
static int install_reader(char *msg, size_t n);
static const char *const kCalAsk[] = {
    "lhsight-gxr", "--transform", "FRAME_IMU_0", "SENSOR_THIRD_EYE", "FRAME_IMU_0", "CAMERA_trackingA_curved", "FRAME_IMU_0",
    "CAMERA_trackingB_curved", "FRAME_IMU_0", "CAMERA_trackingC_curved", "FRAME_IMU_0", "CAMERA_trackingD_curved", NULL};

// head_R, head_t: head -> IMU_0, rv the rotation as a vector (rad). Returns 1 when live, else leaves them as they were
static int head_origin(double head_R[9], double head_t[3], double rv[3], char *why, size_t n) {
  char fw[PROP_VALUE_MAX] = "";
  __system_property_get("ro.build.version.incremental", fw);
  if (strcmp(fw, CAL_FIRMWARE)) { snprintf(why, n, "firmware %s", fw); return 0; }
  if (install_reader(why, n)) return 0;
  int p[2];
  if (pipe(p)) { snprintf(why, n, "no pipe"); return 0; }
  pid_t pid = fork();
  if (pid < 0) { close(p[0]); close(p[1]); snprintf(why, n, "no fork"); return 0; }
  if (pid == 0) {
    dup2(p[1], 1);
    dup2(p[1], 2);
    int nul = open("/dev/null", O_RDONLY);
    dup2(nul, 0);
    for (int fd = 3; fd < 1024; fd++) close(fd);
    setenv("LD_LIBRARY_PATH", "/vendor/lib64:/system/lib64", 1);
    execv(RUN_EXE, (char *const *)kCalAsk);
    _exit(127);
  }
  close(p[1]);
  // bounded: the asker gives up by itself (alarm), which ends this read
  char out[2048];
  size_t got = 0;
  for (;;) {
    ssize_t r = read(p[0], out + got, sizeof out - 1 - got);
    if (r < 0 && errno == EINTR) continue;
    if (r <= 0) break;
    got += r;
    if (got == sizeof out - 1) break;
  }
  out[got] = 0;
  close(p[0]);
  waitpid(pid, NULL, 0);
  int live = 0;
  snprintf(why, n, "no answer");
  for (char *ln = strtok(out, "\n"); ln; ln = strtok(NULL, "\n")) {
    char m[300];
    snprintf(m, sizeof m, "live calibration: %s", ln);
    hs_log(m);
    double v[6];
    if (sscanf(ln, "FRAME_IMU_0 SENSOR_THIRD_EYE %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 6) {
      double a = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]), s = a > 1e-12 ? sin(a) / a : 1, c = a > 1e-12 ? (1 - cos(a)) / (a * a) : .5;
      double x = v[0], y = v[1], z = v[2];
      double R[9] = {1 - c * (y * y + z * z), c * x * y - s * z, c * x * z + s * y,
                     c * x * y + s * z, 1 - c * (x * x + z * z), c * y * z - s * x,
                     c * x * z - s * y, c * y * z + s * x, 1 - c * (x * x + y * y)};
      memcpy(head_R, R, sizeof R);
      for (int i = 0; i < 3; i++) { rv[i] = v[i]; head_t[i] = v[i + 3]; }
      live = 1;
    } else if (!live) {
      snprintf(why, n, "%.100s", ln);
    }
  }
  return live;
}

int hs_calibration(char **data, size_t *len, char *name, size_t n) {
  double hR[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, ht[3] = {kEye[0], kEye[1], kEye[2]}, hv[3] = {0, 0, 0};
  char why[160] = "", head[160];
  // asked even when an override goes out: the log is what tells how it moves between sessions
  // the driver logs its name up to the first space or '/', so none in what the log is for
  if (head_origin(hR, ht, hv, why, sizeof why))
    snprintf(head, sizeof head, "head-live:%.3f,%.3f,%.3f(deg),%.4f,%.4f,%.4f(m)", hv[0] * 180 / M_PI, hv[1] * 180 / M_PI,
             hv[2] * 180 / M_PI, ht[0], ht[1], ht[2]);
  else {
    snprintf(head, sizeof head, "head-measured");
    char m[200];
    snprintf(m, sizeof m, "no live head origin: %s", why);
    hs_log(m);
  }
  hs_log(head);
  // a calibration refined elsewhere (fitted to recorded base station sightings) wins over the profile's
  char op[512];
  snprintf(op, sizeof op, "%s/calibration_override.json", moddir);
  size_t olen;
  char *ov = read_file(op, &olen);
  if (ov && olen) {
    *data = ov;
    *len = olen;
    snprintf(name, n, "override,not-applied:%s", head);
    return 0;
  }
  free(ov);
  size_t plen;
  const char *src = OPX_PROFILE, *srcname = "factory";
  char *prof = read_file(src, &plen);
  if (!prof || !plen) {
    free(prof);
    src = OPX_PROFILE_GENERIC, srcname = "generic";
    prof = read_file(src, &plen);
  }
  if (!prof || !plen) {
    free(prof);
    snprintf(name, n, "can't read %s or %s", OPX_PROFILE, OPX_PROFILE_GENERIC);
    return -1;
  }
  const char *pe = prof + plen;
  size_t jcap = 8192, jo = 0;
  char *json = malloc(jcap);
  #define JP(...) do { jo += snprintf(json + jo, jcap - jo, __VA_ARGS__); } while (0)
  // galaxy_xr_sensor: lhsight's frame times are the sensors' own (mid-exposure, CLOCK_MONOTONIC), not when it found them
  JP("{\"kind\": \"galaxy_xr_sensor\", \"GxrCameraCalibration\": [");
  int ncam = 0, nrec = 0;
  char missing[256] = "";
  for (int i = 0; i < NCAM; i++) {
    const char *cb, *ce, *xb, *xe;
    double t[3], R[9];
    if (!opx_block(prof, pe, "cameras", kCam[i], &cb, &ce) || !opx_block(prof, pe, "camera_extrinsics", kCam[i], &xb, &xe) ||
        !opx_pose(xb, xe, "frame_t_camera", t, R)) {
      snprintf(missing + strlen(missing), sizeof missing - strlen(missing), " %s", kCam[i]);
      continue;
    }
    char rp[256];
    snprintf(rp, sizeof rp, "%s/FRAME_IMU_0_t_CAMERA_%s_.textproto", OPX_RECORDS, kCam[i]);
    size_t rlen;
    char *rec = read_file(rp, &rlen);
    if (rec && rlen && opx_pose(rec, rec + rlen, "a_t_b", t, R)) nrec++;
    free(rec);
    static const char *const xy[] = {"x", "y"};
    double f[2], c[2];
    if (!opx_vec(cb, ce, "calibrated_focal_length", xy, 2, f) || !opx_vec(cb, ce, "calibrated_principal_point", xy, 2, c) ||
        !(f[0] > 0 && f[1] > 0)) {
      snprintf(missing + strlen(missing), sizeof missing - strlen(missing), " %s(intrinsics)", kCam[i]);
      continue;
    }
    double w = opx_num(cb, ce, "calibrated_width"), h = opx_num(cb, ce, "calibrated_height");
    if (ncam) JP(",");
    JP("\n  {\"Id\": %d, \"Name\": \"%s\", \"ImageSize\": [%.0f, %.0f],", i, kCam[i], w > 0 ? w : 640, h > 0 ? h : 640);
    // DeviceFromCamera: 4x4 row-major [R | t; 0 0 0 1], device = the HMD pose's frame: inv(IMU_0 from head) * IMU_0 from camera
    double D[9], d[3];
    for (int r = 0; r < 3; r++) {
      d[r] = 0;
      for (int k = 0; k < 3; k++) d[r] += hR[k * 3 + r] * (t[k] - ht[k]);
      for (int c0 = 0; c0 < 3; c0++) {
        D[r * 3 + c0] = 0;
        for (int k = 0; k < 3; k++) D[r * 3 + c0] += hR[k * 3 + r] * R[k * 3 + c0];
      }
    }
    JP(" \"DeviceFromCamera\": [%.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, %.10g, 0, 0, 0, 1],",
       D[0], D[1], D[2], d[0], D[3], D[4], D[5], d[1], D[6], D[7], D[8], d[2]);
    JP(" \"Projection\": {\"Model\": \"PinholeXY\", \"Coefficients\": [%.10g, %.10g, %.10g, %.10g]},", f[0], f[1], c[0], c[1]);
    // distortion_fisheye_spline3 isn't a KB polynomial (read as one it runs away at the rim); the lenses are near
    // ideal equidistant fisheyes, which is what all-zero coefficients give
    JP(" \"Distortion\": {\"Coefficients\": [0, 0, 0, 0, 0, 0, 0, 0]},");
    JP(" \"SensorType\": \"S5K931\"}");
    ncam++;
  }
  JP("\n]}\n");
  #undef JP
  free(prof);
  if (!ncam) {
    free(json);
    snprintf(name, n, "no tracking cameras in %s:%s", src, missing);
    return -1;
  }
  *data = json;
  *len = jo;
  if (missing[0]) {
    char m[300];
    snprintf(m, sizeof m, "calibration: missing%s", missing);
    hs_log(m);
  }
  snprintf(name, n, "openpx-%s,%d-cameras,%d-records,%s%s", srcname, ncam, nrec, head, missing[0] ? ",cameras-missing" : "");
  return 0;
}

// ---------------------------------------------------------------- lhsight-gxr
// a stray reader (this daemon SIGKILLed, an old adb session) would keep its camera clients and the frame pattern:
// lhsight-gxr, or the Frida script inside perceptionhost that earlier modules ran
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
  if (!strcmp(base, "lhsight-gxr")) return 1;
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
    snprintf(m, sizeof m, "stopping a stray lhsight (pid %s)", found[i]);
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
  snprintf(m, sizeof m, "module %s (Galaxy XR)", moddir);
  hs_log(m);
}

// The linker lets an executable under /data (the module) load no vendor library: /linkerconfig/ld.config.txt files
// /data under "system", whose namespace leaves /vendor/lib64 out, and libqvrcamera_client.qti.so is one. /data/local/tmp
// is "unrestricted": lhsight-gxr runs from a copy there
static int copy_file(const char *src, const char *dst) {
  size_t len;
  char *data = read_file(src, &len);
  if (!data) return -1;
  char tmp[512];
  snprintf(tmp, sizeof tmp, "%s.new", dst);
  FILE *f = fopen(tmp, "wb");
  int ok = f && fwrite(data, 1, len, f) == len;
  if (f) fclose(f);
  free(data);
  if (!ok || chmod(tmp, 0755) || rename(tmp, dst)) { unlink(tmp); return -1; }
  return 0;
}

static int install_reader(char *msg, size_t n) {
  char src[512];
  snprintf(src, sizeof src, "%s/lhsight-gxr", moddir);
  if (access(src, R_OK)) { snprintf(msg, n, "no %s", src); return -1; }
  mkdir("/data/local/tmp/questlhsync", 0755);
  if (copy_file(src, RUN_EXE)) { snprintf(msg, n, "can't copy %s to %s", src, RUN_EXE); return -1; }
  return 0;
}

// lhsight-gxr with the ids hs_calibration gives trackingA..D
pid_t hs_capture_start(int out_fd, char *msg, size_t n) {
  char id[NCAM][8];
  const char *exe = RUN_EXE;
  if (install_reader(msg, n)) return -1;
  for (int i = 0; i < NCAM; i++) snprintf(id[i], sizeof id[i], "%d", i);
  pid_t pid = fork();
  if (pid == 0) {
    setsid();
    dup2(out_fd, 1);
    dup2(out_fd, 2);
    int nul = open("/dev/null", O_RDONLY);
    dup2(nul, 0);
    for (int fd = 3; fd < 1024; fd++) close(fd);
    execl(exe, "lhsight-gxr", id[0], id[1], id[2], id[3], (char *)NULL);
    _exit(127);
  }
  if (pid < 0) return -1;
  snprintf(msg, n, "lhsight-gxr started (pid %d): its own QVR camera clients, cameras %s %s %s %s as 0-3", pid, kCam[0],
           kCam[1], kCam[2], kCam[3]);
  return pid;
}

