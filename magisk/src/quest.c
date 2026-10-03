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
#include <time.h>
#include <unistd.h>

#include <android/log.h>

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
