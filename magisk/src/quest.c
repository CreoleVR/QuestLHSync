// lhsyncd's Quest Pro side (src/headset/headset.h): Android properties, Meta's camera calibration in
// /persist/calibration, and lhsight as a Frida script (frida-inject + lhsight.js) inside the sensors HAL: it reads the
// camera rings and writes nothing.
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

int hs_calibration(char **data, size_t *len, char *name, size_t n) {
  char path[512];
  FILE *f = calibration(path, sizeof path) ? fopen(path, "rb") : NULL;
  if (!f) { snprintf(name, n, "no camera calibration in /persist/calibration"); return -1; }
  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  fseek(f, 0, SEEK_SET);
  *data = malloc(size > 0 ? size : 1);
  *len = size > 0 ? fread(*data, 1, size, f) : 0;
  fclose(f);
  snprintf(name, n, "%s", strstr(path, "/calibration/") ? strstr(path, "/calibration/") + 13 : path);
  return 0;
}

static pid_t hal_pid(void) {
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
    if (!strcmp(base, HAL)) found = atoi(e->d_name);
  }
  closedir(d);
  return found;
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
  pid_t hal = hal_pid();
  if (!hal) { snprintf(msg, n, "sensors HAL not running"); return 0; }
  char frida[512], script[512], pidstr[16];
  snprintf(frida, sizeof frida, "%s/frida-inject", moddir);
  snprintf(script, sizeof script, "%s/lhsight.js", moddir);
  snprintf(pidstr, sizeof pidstr, "%d", hal);
  mkdir(RUNDIR, 0755);
  mkdir(RUNDIR "/tmp", 0755);
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
  snprintf(msg, n, "lhsight started in the sensors HAL (pid %d)", hal);
  return pid;
}
