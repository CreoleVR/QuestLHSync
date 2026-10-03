// lhsyncd's Steam Frame side (src/headset/headset.h): the factory calibration in /persist, and lhsight (lhsight.c) in a
// child process. Runs as a systemd user service; the camera buffers come from the questlhsync_frame SteamVR driver, so
// nothing here needs root.
#define _GNU_SOURCE
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <unistd.h>

#include "headset.h"
#include "lhsight.h"

#define XRSERVICE_JSON "/persist/xrservice.json"     // cameras: intrinsics, extrinsics to camera 0
#define DEVICE_JSON "/persist/device_config.json"    // where the head (SteamVR's HMD pose) sits against the cameras

const char *const hs_capture_name = "lhsight";

void hs_log(const char *line) {
  fprintf(stderr, "%s\n", line);  // the journal
}

void hs_init(int argc, char **argv) { (void)argc, (void)argv; }

static char *slurp(const char *path, size_t *len) {
  FILE *f = fopen(path, "rb");
  if (!f) return NULL;
  size_t cap = 1 << 16, n = 0;
  char *b = malloc(cap);
  for (size_t r; b && (r = fread(b + n, 1, cap - n, f)) > 0;) {
    n += r;
    if (n == cap) b = realloc(b, cap *= 2);
  }
  fclose(f);
  if (b) b[n] = 0, *len = n;  // n < cap: room for the terminator
  return b;
}

// the string value of "key" in a flat JSON file, good enough for device_config.json's top level
static void json_str(const char *json, const char *key, char *out, size_t n) {
  char k[64];
  snprintf(k, sizeof k, "\"%s\"", key);
  const char *s = json ? strstr(json, k) : NULL;
  out[0] = 0;
  if (!s || !(s = strchr(s + strlen(k), ':')) || !(s = strchr(s, '"'))) return;
  const char *e = strchr(++s, '"');
  if (e) snprintf(out, n, "%.*s", (int)(e - s), s);
}

void hs_device_info(char *serial, char *model, char *fw, size_t n) {
  size_t len;
  char *dc = slurp(DEVICE_JSON, &len);
  json_str(dc, "device_serial_number", serial, n);
  free(dc);
  snprintf(model, n, "Steam Frame");
  fw[0] = 0;
  FILE *f = fopen("/etc/os-release", "r");
  char line[256];
  while (f && fgets(line, sizeof line, f))
    if (!strncmp(line, "BUILD_ID=", 9)) { line[strcspn(line, "\r\n")] = 0; snprintf(fw, n, "SteamOS_%s", line + 9); }
  if (f) fclose(f);
}

// both files as they are, in one JSON object: {"kind": "steam_frame", "xrservice": {...}, "device_config": {...}}
int hs_calibration(char **data, size_t *len, char *name, size_t n) {
  size_t la = 0, lb = 0;
  char *a = slurp(XRSERVICE_JSON, &la), *b = slurp(DEVICE_JSON, &lb);
  if (!a || !b || !la || !lb) {
    free(a), free(b);
    snprintf(name, n, "no camera calibration (%s, %s)", XRSERVICE_JSON, DEVICE_JSON);
    return -1;
  }
  static const char h[] = "{\"kind\": \"steam_frame\", \"xrservice\": ", m[] = ", \"device_config\": ", t[] = "}\n";
  *len = strlen(h) + la + strlen(m) + lb + strlen(t);
  char *o = *data = malloc(*len);
  memcpy(o, h, strlen(h)), o += strlen(h);
  memcpy(o, a, la), o += la;
  memcpy(o, m, strlen(m)), o += strlen(m);
  memcpy(o, b, lb), o += lb;
  memcpy(o, t, strlen(t));
  free(a), free(b);
  snprintf(name, n, "factory/xrservice.json");
  return 0;
}

pid_t hs_capture_start(int out_fd, char *msg, size_t n) {
  pid_t parent = getpid(), pid = fork();
  if (pid == 0) {
    prctl(PR_SET_PDEATHSIG, SIGTERM);  // never outlive lhsyncd: it holds on to XRService's buffers
    if (getppid() != parent) _exit(1);
    signal(SIGTERM, SIG_DFL);
    signal(SIGINT, SIG_DFL);
    signal(SIGPIPE, SIG_DFL);
    dup2(out_fd, 1);
    dup2(out_fd, 2);
    for (int fd = 3; fd < 1024; fd++) close(fd);
    _exit(lhsight_main());
  }
  if (pid < 0) return -1;
  snprintf(msg, n, "lhsight started");
  return pid;
}
