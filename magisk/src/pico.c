// lhsyncd's PICO 4 side (src/headset/headset.h): Android properties, PICO's camera calibration in
// /mnt/vendor/persist/pvr/mav0, and picosight as a Frida script (frida-inject + picosight.js) inside
// pvrtrackingservice: it reads the camera frames the tracking service gets and writes nothing.
//
// The calibration the driver gets is the Quest's "CameraCalibration" form, the cameras' poses in the head frame (the
// one the HMD pose apps get is of), with "kind": "pico4". From PICO's own files:
//  - mav0/cam<i>/sensor.yaml (OpenCV FileStorage, Kalibr's fields as libpico_controller_tracking.pxr.so's
//    picosensor::CameraCalib reads them): camera_model pinhole | omni, intrinsics [fx, fy, cx, cy] (omni: [xi, fx, fy,
//    cx, cy]), distortion_model equidistant | radtan | equiDis62, distortion_coefficients, resolution, T_BS (4x4,
//    row-major: the IMU body <- camera)
//  - the head pose: libSlamTools.pxr.so getPredictResult turns the SLAM's body pose into the head pose apps get. Its
//    axes: x_head = -y_body, y_head = x_body, z_head = z_body (a quarter turn about z), and the head's origin is the
//    body's plus the head-frame offset o of a 6DoF pose: line 1 of config_head.txt (/apex/com.pico.xrsystem/etc/pvr/
//    config/, else /system/etc/pvr/config/), built in (0.05412, 0.01361, 0.0537) m when there's none. So a camera's
//    pose in the head frame: R = C R_BS, t = C t_BS - o.
#include <dirent.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <android/log.h>

#include "headset.h"

#define SERVICE "pvrtrackingservice"
#define RUNDIR "/dev/.questlhsync"
#define NCAM 4
#define UPWARD 0.1  // as the driver's kUpward: a camera whose axis climbs this much (of 1) looks up

const char *const hs_capture_name = "frida";
static const char *moddir = "/data/adb/modules/questlhsync";

void hs_log(const char *line) { __android_log_print(ANDROID_LOG_INFO, "QuestLHSync", "%s", line); }

void hs_device_info(char *serial, char *model, char *fw, size_t n) {
  (void)n;  // the core's fields hold PROP_VALUE_MAX
  char m[PROP_VALUE_MAX] = "";
  __system_property_get("ro.serialno", serial);
  __system_property_get("ro.product.model", m);
  __system_property_get("ro.build.display.id", fw);
  if (!strncmp(m, "A8110", 5) || !m[0]) strcpy(model, "PICO 4");  // the PICO 4 calls itself A8110
  else if (strstr(m, "PICO") || strstr(m, "Pico")) snprintf(model, PROP_VALUE_MAX, "%s", m);
  else snprintf(model, PROP_VALUE_MAX, "PICO 4 (%.60s)", m);
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
  if (len) *len = got;
  return buf;
}

// ---------------------------------------------------------------- sensor.yaml (OpenCV FileStorage YAML)
// key: at a line's start (after spaces), from p on; returns just past "key:", or NULL
static const char *ykey(const char *p, const char *key) {
  size_t n = strlen(key);
  for (const char *s = p; (s = strstr(s, key)); s += n) {
    const char *b = s;
    while (b > p && (b[-1] == ' ' || b[-1] == '\t')) b--;
    if (b > p && b[-1] != '\n') continue;
    const char *e = s + n;
    while (*e == ' ' || *e == '\t') e++;
    if (*e == ':') return e + 1;
  }
  return NULL;
}

// a scalar string value (quotes dropped) into out
static int ystr(const char *doc, const char *key, char *out, size_t cap) {
  const char *p = ykey(doc, key);
  out[0] = 0;
  if (!p) return 0;
  while (*p == ' ' || *p == '\t') p++;
  char q = (*p == '"' || *p == '\'') ? *p++ : 0;
  size_t o = 0;
  while (*p && *p != '\n' && *p != '\r' && *p != q && *p != '#' && o + 1 < cap) out[o++] = *p++;
  while (o && (out[o - 1] == ' ' || out[o - 1] == '\t')) o--;
  out[o] = 0;
  return o > 0;
}

// a list "[a, b, ...]" (it may span lines) after key, from p on; returns how many numbers
static int ylist_at(const char *p, double *v, int cap) {
  if (!p) return 0;
  while (*p && *p != '[' && *p != '\n') p++;
  if (*p != '[') return 0;
  int n = 0;
  for (p++; *p && *p != ']' && n < cap;) {
    char *e;
    double x = strtod(p, &e);
    if (e == p) { p++; continue; }
    v[n++] = x;
    p = e;
  }
  return n;
}
static int ylist(const char *doc, const char *key, double *v, int cap) { return ylist_at(ykey(doc, key), v, cap); }

struct cam {
  int ok, id, w, h, omni, dist;  // dist: 0 equidistant (KB4), 1 radtan, 2 equiDis62
  double xi, k[4], d[8];          // k: fx fy cx cy
  double R[3][3], t[3];           // head <- camera
  int nd;
};

static const char *const MAV0[] = {"/mnt/vendor/persist/pvr/mav0", "/mnt/vendor/factory/pvr/mav0"};

// camera i's sensor.yaml, from the first place that has one; why not in err
static int load_cam(int i, const double o[3], struct cam *c, char *err, size_t en) {
  char path[256], s[64];
  char *doc = NULL;
  memset(c, 0, sizeof *c);
  c->id = i;
  for (size_t m = 0; m < sizeof MAV0 / sizeof MAV0[0] && !doc; m++) {
    snprintf(path, sizeof path, "%s/cam%d/sensor.yaml", MAV0[m], i);
    doc = read_file(path, NULL);
  }
  if (!doc) { snprintf(err, en, "no cam%d/sensor.yaml in %s or %s", i, MAV0[0], MAV0[1]); return 0; }
  double in[8], res[4], T[20];
  int nin = ylist(doc, "intrinsics", in, 8), nres = ylist(doc, "resolution", res, 4);
  const char *tbs = ykey(doc, "T_BS");
  int nt = tbs ? ylist_at(ykey(tbs, "data"), T, 20) : 0;
  c->nd = ylist(doc, "distortion_coefficients", c->d, 8);
  ystr(doc, "camera_model", s, sizeof s);
  c->omni = !strcmp(s, "omni");
  if (strcmp(s, "omni") && strcmp(s, "pinhole")) {
    snprintf(err, en, "cam%d: camera_model \"%s\", not pinhole or omni", i, s);
    free(doc);
    return 0;
  }
  ystr(doc, "distortion_model", s, sizeof s);
  c->dist = !strcmp(s, "equidistant") ? 0 : !strcmp(s, "radtan") ? 1 : !strcmp(s, "equiDis62") ? 2 : -1;
  free(doc);
  int need = c->dist == 2 ? 8 : 4;
  if (c->dist < 0 || c->nd < need) {
    snprintf(err, en, "cam%d: distortion_model \"%s\" with %d coefficients", i, s, c->nd);
    return 0;
  }
  if (nin < (c->omni ? 5 : 4) || nres < 2 || nt < 16) {
    snprintf(err, en, "cam%d: intrinsics %d, resolution %d, T_BS %d values", i, nin, nres, nt);
    return 0;
  }
  const double *k = in + (c->omni ? 1 : 0);
  c->xi = c->omni ? in[0] : 0;
  for (int j = 0; j < 4; j++) c->k[j] = k[j];
  c->w = (int)res[0];
  c->h = (int)res[1];
  // head <- body: C (x_h = -y_b, y_h = x_b, z_h = z_b), then the offset
  static const double C[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
  double Rb[3][3], tb[3];
  for (int r = 0; r < 3; r++) {
    for (int q = 0; q < 3; q++) Rb[r][q] = T[r * 4 + q];
    tb[r] = T[r * 4 + 3];
  }
  for (int r = 0; r < 3; r++) {
    c->t[r] = -o[r];
    for (int q = 0; q < 3; q++) {
      c->R[r][q] = 0;
      for (int j = 0; j < 3; j++) c->R[r][q] += C[r][j] * Rb[j][q];
      c->t[r] += C[r][q] * tb[q];
    }
  }
  double det = c->R[0][0] * (c->R[1][1] * c->R[2][2] - c->R[1][2] * c->R[2][1]) -
               c->R[0][1] * (c->R[1][0] * c->R[2][2] - c->R[1][2] * c->R[2][0]) +
               c->R[0][2] * (c->R[1][0] * c->R[2][1] - c->R[1][1] * c->R[2][0]);
  if (!(fabs(det - 1) < 0.01) || !(c->k[0] > 0 && c->k[1] > 0) || c->w <= 0 || c->h <= 0) {
    snprintf(err, en, "cam%d: T_BS isn't a rotation (det %.3f) or the intrinsics are off", i, det);
    return 0;
  }
  c->ok = 1;
  return 1;
}

// the head-frame offset of a 6DoF pose (config_head.txt line 1); where it came from in src
static void head_offset(double o[3], char *src, size_t n) {
  static const char *const p[] = {"/apex/com.pico.xrsystem/etc/pvr/config/config_head.txt",
                                  "/system/etc/pvr/config/config_head.txt"};
  o[0] = 0.05412; o[1] = 0.01361; o[2] = 0.0537;
  snprintf(src, n, "built in");
  for (size_t i = 0; i < sizeof p / sizeof p[0]; i++) {
    FILE *f = fopen(p[i], "r");
    if (!f) continue;
    char line[256];
    double a, b, c;
    int ok = fgets(line, sizeof line, f) && sscanf(line, "%lf %lf %lf", &a, &b, &c) == 3;
    fclose(f);
    if (ok && isfinite(a + b + c) && fabs(a) < 0.5 && fabs(b) < 0.5 && fabs(c) < 0.5) {
      o[0] = a; o[1] = b; o[2] = c;
      snprintf(src, n, "%s", p[i]);
      return;
    }
  }
}

// all four cameras; how many loaded. The first failure in err
static int load_cams(struct cam *c, double o[3], char *osrc, size_t on, char *err, size_t en) {
  int n = 0;
  head_offset(o, osrc, on);
  err[0] = 0;
  for (int i = 0; i < NCAM; i++) {
    char e[200];
    if (load_cam(i, o, &c[i], e, sizeof e)) n++;
    else if (!err[0]) snprintf(err, en, "%s", e);
  }
  return n;
}

static int append(char **buf, size_t *len, size_t *cap, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
static int append(char **buf, size_t *len, size_t *cap, const char *fmt, ...) {
  for (;;) {
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(*buf + *len, *cap - *len, fmt, ap);
    va_end(ap);
    if (k < 0) return -1;
    if (*len + (size_t)k < *cap) { *len += k; return 0; }
    *cap = (*cap + k) * 2;
    *buf = realloc(*buf, *cap);
  }
}

int hs_calibration(char **data, size_t *len, char *name, size_t n) {
  struct cam c[NCAM];
  double o[3];
  char osrc[128], err[256];
  int got = load_cams(c, o, osrc, sizeof osrc, err, sizeof err);
  if (!got) { snprintf(name, n, "no camera calibration: %s", err); *data = NULL; return -1; }
  size_t cap = 8192, l = 0;
  char *b = malloc(cap);
  append(&b, &l, &cap, "{\"kind\": \"pico4\", \"Device\": {\"DeviceType\": \"PICO 4\"}, \"HeadOffset\": [%.6f, %.6f, %.6f], "
         "\"HeadOffsetFrom\": \"%s\", \"CameraCalibration\": [", o[0], o[1], o[2], osrc);
  int first = 1;
  for (int i = 0; i < NCAM; i++) {
    if (!c[i].ok) continue;
    const struct cam *k = &c[i];
    append(&b, &l, &cap, "%s{\"Id\": %d, \"SensorType\": \"PICO\", \"Shutter\": {\"Type\": \"Global\"}, "
           "\"ImageSize\": [%d, %d], \"DeviceFromCamera\": [", first ? "" : ", ", k->id, k->w, k->h);
    for (int r = 0; r < 4; r++)
      for (int q = 0; q < 4; q++) {
        double v = r == 3 ? (q == 3 ? 1 : 0) : q == 3 ? k->t[r] : k->R[r][q];
        append(&b, &l, &cap, "%s%.9f", r || q ? ", " : "", v);
      }
    append(&b, &l, &cap, "], \"Projection\": {\"Model\": \"PinholeXY\", \"Coefficients\": [%.9f, %.9f, %.9f, %.9f]}, ",
           k->k[0], k->k[1], k->k[2], k->k[3]);
    // the driver's distortion models (src/driver/sync.cpp CamCal):
    //  Fisheye62 [k1..k6, p0, p1]: equidistant (its k1..k4, the rest 0) and equiDis62 (taken as OpenCV's order k1..k6,
    //    p1, p2; the driver's p0 is OpenCV's p2)
    //  MeiRadTan [xi, k1, k2, p1, p2] (OpenCV's p1, p2): omni with radtan
    //  RadTan [k1, k2, p1, p2]: pinhole with radtan
    //  KB4 under omni (xi with an equidistant lens): no such lens in PICO's tools, left out
    const double *d = k->d;
    if (k->omni && k->dist == 1)
      append(&b, &l, &cap, "\"Distortion\": {\"Model\": \"MeiRadTan\", \"Coefficients\": [%.9f, %.9g, %.9g, %.9g, %.9g]}}",
             k->xi, d[0], d[1], d[2], d[3]);
    else if (!k->omni && k->dist == 1)
      append(&b, &l, &cap, "\"Distortion\": {\"Model\": \"RadTan\", \"Coefficients\": [%.9g, %.9g, %.9g, %.9g]}}",
             d[0], d[1], d[2], d[3]);
    else if (!k->omni && k->dist == 0)
      append(&b, &l, &cap, "\"Distortion\": {\"Model\": \"Fisheye62\", \"Coefficients\": [%.9g, %.9g, %.9g, %.9g, 0, 0, 0, 0]}}",
             d[0], d[1], d[2], d[3]);
    else if (!k->omni && k->dist == 2)
      append(&b, &l, &cap, "\"Distortion\": {\"Model\": \"Fisheye62\", \"Coefficients\": [%.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g]}}",
             d[0], d[1], d[2], d[3], d[4], d[5], d[7], d[6]);
    else {
      snprintf(name, n, "cam%d: an omni camera with a %s lens: not supported", i, k->dist == 0 ? "equidistant" : "equiDis62");
      free(b);
      *data = NULL;
      return -1;
    }
    first = 0;
  }
  append(&b, &l, &cap, "]}");
  *data = b;
  *len = l;
  snprintf(name, n, "pico4/mav0");
  return 0;
}

// ---------------------------------------------------------------- the cameras picosight scans
// the ones looking up (the base stations hang above: PICO's lower pair looks at the floor and the hands); in note, what
// the calibration says about each, and whether the head frame came out as the driver expects it (x right, y up, z back:
// the cameras look forward on the whole, the left ones are on the left)
static void scan_list(char *out, size_t cap, char *note, size_t ncap) {
  struct cam c[NCAM];
  double o[3];
  char osrc[128], err[256];
  out[0] = 0;
  if (!load_cams(c, o, osrc, sizeof osrc, err, sizeof err)) {
    snprintf(note, ncap, "no camera calibration (%s): scanning all cameras", err);
    return;
  }
  size_t ol = 0, nl = 0;
  double fz = 0, lr = 0;
  int up = 0, nok = 0;
  nl += snprintf(note + nl, ncap - nl, "head offset %.4f %.4f %.4f (%s); cameras (id: model, axis in the head frame)",
                 o[0], o[1], o[2], osrc);
  for (int i = 0; i < NCAM; i++) {
    if (!c[i].ok) continue;
    nok++;
    double ax = c[i].R[0][2], ay = c[i].R[1][2], az = c[i].R[2][2];
    fz += az;
    lr += c[i].t[0] * ax;  // a camera on the left looks left, on the right right
    if (nl < ncap) nl += snprintf(note + nl, ncap - nl, " %d: %s/%s %.2f %.2f %.2f%s", i, c[i].omni ? "omni" : "pinhole",
                                  c[i].dist == 0 ? "equidistant" : c[i].dist == 1 ? "radtan" : "equiDis62", ax, ay, az,
                                  ay > UPWARD ? " up" : "");
    if (ay > UPWARD) {
      up++;
      if (ol < cap) ol += snprintf(out + ol, cap - ol, "%s%d", ol ? "," : "", i);
    }
  }
  if (!up) {
    out[0] = 0;
    if (nl < ncap) nl += snprintf(note + nl, ncap - nl, "; none looks up: scanning all");
  }
  if (nl < ncap && (fz >= 0 || lr <= 0))
    snprintf(note + nl, ncap - nl, "; W the cameras don't look %s in this head frame: the PC's alignment will be off",
             fz >= 0 ? "forward" : "outward");
  (void)nok;
}

static pid_t service_pid(void) {
  DIR *d = opendir("/proc");
  if (!d) return 0;
  struct dirent *e;
  pid_t found = 0;
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
    if (!strcmp(base, SERVICE)) found = atoi(e->d_name);
  }
  closedir(d);
  return found;
}

// picosight.js with the cameras to scan filled in, in the run dir: its path in out, what was found in note; out ""
// on failure, with the reason in note
static void prepare_script(char *out, size_t cap, char *note, size_t ncap) {
  char src[512], cams[64];
  size_t len;
  out[0] = 0;
  snprintf(src, sizeof src, "%s/picosight.js", moddir);
  char *js = read_file(src, &len);
  if (!js) { snprintf(note, ncap, "can't read %s", src); return; }
  scan_list(cams, sizeof cams, note, ncap);
  char *at = strstr(js, "@CAMS@");
  snprintf(out, cap, "%s/picosight.js", RUNDIR);
  FILE *f = fopen(out, "wb");
  if (!f) { snprintf(note, ncap, "can't write %s", out); out[0] = 0; free(js); return; }
  if (at && cams[0]) {
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

// a frida-inject still running a picosight.js (this daemon SIGKILLed, an old adb session) would make the next start a
// second hook: stop those, and only those
static int is_ours(const char *pid) {
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
    if (l >= 12 && !strcmp(cmd + i + l - 12, "picosight.js")) return 1;
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
    if (e->d_name[0] >= '0' && e->d_name[0] <= '9' && is_ours(e->d_name)) snprintf(found[n++], 16, "%s", e->d_name);
  closedir(d);
  for (int i = 0; i < n; i++) {
    char m[96];
    snprintf(m, sizeof m, "stopping a stray picosight (frida-inject pid %s)", found[i]);
    hs_log(m);
    kill(atoi(found[i]), SIGTERM);
  }
  for (int t = 0; n && t < 30; t++) {
    usleep(100000);
    int left = 0;
    for (int i = 0; i < n; i++) left += is_ours(found[i]);
    if (!left) return;
  }
  for (int i = 0; i < n; i++)
    if (is_ours(found[i])) kill(atoi(found[i]), SIGKILL);
}

void hs_init(int argc, char **argv) {
  if (argc > 1) moddir = argv[1];
  kill_strays();
  char m[600];
  snprintf(m, sizeof m, "module %s (PICO 4)", moddir);
  hs_log(m);
}

pid_t hs_capture_start(int out_fd, char *msg, size_t n) {
  pid_t svc = service_pid();
  if (!svc) { snprintf(msg, n, SERVICE " not running"); return 0; }
  char frida[512], script[512], pidstr[16], note[900];
  mkdir(RUNDIR, 0755);
  mkdir(RUNDIR "/tmp", 0755);
  prepare_script(script, sizeof script, note, sizeof note);
  if (!script[0]) { snprintf(msg, n, "%s", note); return 0; }
  snprintf(frida, sizeof frida, "%s/frida-inject", moddir);
  snprintf(pidstr, sizeof pidstr, "%d", svc);
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
  snprintf(msg, n, "picosight started in " SERVICE " (pid %d); %s", svc, note);
  return pid;
}
