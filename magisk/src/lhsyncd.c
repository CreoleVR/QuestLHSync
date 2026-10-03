// lhsyncd: QuestLHSync's headset daemon (started by the Magisk module's service.sh). Serves the tracking
// cameras' view of lighthouse base station laser flashes to the QuestLHSync SteamVR driver on the LAN.
//   UDP 47281  discovery: "QLHS?" -> "QLHS 1 <serial> <tcp port> <model> <idle|capturing>"
//   TCP 47280  per client: "H QuestLHSync 1 serial=.. model=.. fw=.. module=..", "C <len> <name>" + the camera
//              calibration file (/persist/calibration, read-only), then lhsight's lines (F/T/I/W) as they come.
//              The client sends "P <seq> <pc_ns>"; the reply "Q <seq> <pc_ns> <mono_ns>" is its clock sync.
// lhsight (frida-inject + lhsight.js inside the sensors HAL; reads the camera rings, writes nothing) runs only
// while a client is connected, and is stopped a few seconds after the last one leaves.
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <android/log.h>

#define TCP_PORT 47280
#define UDP_PORT 47281
#define MAXC 4
#define OUTCAP (8 << 20)
#define HAL "vendor.oculus.hardware.sensors@1.0-service"
#define RUNDIR "/dev/.questlhsync"
#ifndef MODULE_VERSION
#define MODULE_VERSION "dev"
#endif

static const char *moddir = "/data/adb/modules/questlhsync";

static void logi(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  __android_log_vprint(ANDROID_LOG_INFO, "QuestLHSync", fmt, ap);
  va_end(ap);
}

static double mono(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec / 1e9;
}

static long long mono_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (long long)t.tv_sec * 1000000000ll + t.tv_nsec;
}

// ---------------------------------------------------------------- clients
struct client {
  int fd;
  char *out;
  size_t olen;
  char in[1024];
  size_t ilen;
  double last_rx;
};
static struct client cl[MAXC];
static int ncl;

static void cl_close(int i) {
  close(cl[i].fd);
  free(cl[i].out);
  cl[i] = cl[--ncl];
  memset(&cl[ncl], 0, sizeof cl[ncl]);
}

static int cl_queue(struct client *c, const char *d, size_t n) {
  if (c->olen + n > OUTCAP) return -1;  // a client this far behind is gone
  memcpy(c->out + c->olen, d, n);
  c->olen += n;
  return 0;
}

static void cl_flush(int i) {
  struct client *c = &cl[i];
  while (c->olen) {
    ssize_t n = send(c->fd, c->out, c->olen, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (n <= 0) break;
    memmove(c->out, c->out + n, c->olen - n);
    c->olen -= n;
  }
}

static void broadcast(const char *d, size_t n) {
  for (int i = 0; i < ncl; i++)
    if (cl_queue(&cl[i], d, n)) {
      logi("client %d fell behind, dropped", i);
      cl_close(i--);
    }
}

static void say(const char *fmt, ...) {  // a message line to the log and every client
  char b[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b - 2, fmt, ap);
  va_end(ap);
  logi("%s", b);
  size_t n = strlen(b);
  b[n++] = '\n';
  broadcast(b, n);
}

// ---------------------------------------------------------------- device info
static char serial[PROP_VALUE_MAX], model[PROP_VALUE_MAX], fw[PROP_VALUE_MAX];

static void underscores(char *s) {
  for (; *s; s++)
    if (*s == ' ' || *s == '\t') *s = '_';
}

static void device_info(void) {
  __system_property_get("ro.serialno", serial);
  __system_property_get("ro.product.model", model);
  __system_property_get("ro.build.version.incremental", fw);
  if (!serial[0]) strcpy(serial, "unknown");
  underscores(serial);
  underscores(model);
  underscores(fw);
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

static int send_hello(struct client *c) {
  char h[512];
  int n = snprintf(h, sizeof h, "H QuestLHSync 1 serial=%s model=%s fw=%s module=%s\n", serial, model, fw, MODULE_VERSION);
  if (cl_queue(c, h, n)) return -1;
  char path[512];
  if (!calibration(path, sizeof path)) {
    n = snprintf(h, sizeof h, "E no camera calibration in /persist/calibration\n");
    return cl_queue(c, h, n);
  }
  size_t got;
  char *buf = read_file(path, &got);
  if (!buf) return -1;
  const char *name = strstr(path, "/calibration/") ? strstr(path, "/calibration/") + 13 : path;
  n = snprintf(h, sizeof h, "C %zu %s\n", got, name);
  int r = cl_queue(c, h, n) || cl_queue(c, buf, got);
  free(buf);
  return r ? -1 : 0;
}

// ---------------------------------------------------------------- lhsight (frida-inject in the sensors HAL)
static pid_t child;
static int child_fd = -1;
static char cbuf[1 << 16];
static size_t clen;
static double stop_at, kill_at, restart_at;
static int failures;

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

// lhsight.js with this headset's cameras filled in, in the run dir: its path, or "" on failure
static void prepare_script(char *out, size_t cap) {
  char src[512], path[512], cams[512] = "";
  size_t len, cal_len;
  out[0] = 0;
  snprintf(src, sizeof src, "%s/lhsight.js", moddir);
  char *js = read_file(src, &len);
  if (!js) { say("E can't read %s", src); return; }
  char *cal = calibration(path, sizeof path) ? read_file(path, &cal_len) : NULL;
  int n = cal ? camera_list(cal, cal_len, cams, sizeof cams) : 0;
  free(cal);
  if (n) say("I tracking cameras (id:width:height:scan) %s", cams);
  else say("W no tracking cameras in the camera calibration: trying the Quest Pro's");
  char *at = strstr(js, "@CAMS@");
  snprintf(out, cap, "%s/lhsight.js", RUNDIR);
  FILE *f = fopen(out, "wb");
  if (!f) { say("E can't write %s", out); out[0] = 0; free(js); return; }
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
  for (int i = 0; i < n; i++) logi("stopping a stray lhsight (frida-inject pid %s)", found[i]), kill(atoi(found[i]), SIGTERM);
  for (int t = 0; n && t < 30; t++) {
    usleep(100000);
    int left = 0;
    for (int i = 0; i < n; i++) left += is_lhsight(found[i]);
    if (!left) return;
  }
  for (int i = 0; i < n; i++)
    if (is_lhsight(found[i])) kill(atoi(found[i]), SIGKILL);
}

static void start_capture(void) {
  char hal_name[256];
  pid_t hal = hal_pid(hal_name, sizeof hal_name);
  if (!hal) { say("E sensors HAL not running"); restart_at = mono() + 5; return; }
  if (strcmp(hal_name, HAL)) say("I sensors HAL: %s", hal_name);
  char frida[512], script[512], pidstr[16];
  mkdir(RUNDIR, 0755);
  mkdir(RUNDIR "/tmp", 0755);
  prepare_script(script, sizeof script);
  if (!script[0]) { restart_at = mono() + 30; return; }
  int p[2];
  if (pipe(p)) return;
  snprintf(frida, sizeof frida, "%s/frida-inject", moddir);
  snprintf(pidstr, sizeof pidstr, "%d", hal);
  pid_t pid = fork();
  if (pid == 0) {
    setsid();
    dup2(p[1], 1);
    dup2(p[1], 2);
    int nul = open("/dev/null", O_RDONLY);
    dup2(nul, 0);
    for (int fd = 3; fd < 1024; fd++) close(fd);
    setenv("TMPDIR", RUNDIR "/tmp", 1);  // frida's scratch files: tmpfs, never /data/local/tmp
    chdir(RUNDIR);
    execl(frida, "frida-inject", "-p", pidstr, "-s", script, (char *)NULL);
    _exit(127);
  }
  close(p[1]);
  if (pid < 0) { close(p[0]); return; }
  child = pid;
  child_fd = p[0];
  fcntl(child_fd, F_SETFL, O_NONBLOCK);
  clen = 0;
  stop_at = kill_at = 0;
  say("I lhsyncd: lhsight started in the sensors HAL (pid %d)", hal);
}

static void stop_capture(void) {
  if (child <= 0) return;
  kill(child, SIGTERM);  // frida-inject detaches; the script unloads from the HAL
  kill_at = mono() + 3;
}

static void reap(void) {
  if (child <= 0) return;
  int st;
  pid_t r = waitpid(child, &st, WNOHANG);
  if (r == child) {
    if (child_fd >= 0) { close(child_fd); child_fd = -1; }
    child = 0;
    kill_at = 0;
    if (ncl) {
      failures++;
      double wait = failures < 3 ? 2 : failures < 6 ? 10 : 30;
      say("W lhsight exited (status %d), restarting in %.0f s", st, wait);
      restart_at = mono() + wait;
    } else {
      logi("lhsight stopped");
    }
  } else if (kill_at && mono() > kill_at) {
    kill(child, SIGKILL);
    kill_at = 0;
  }
}

// lhsight's output -> clients, whole lines; anything that isn't one of its records is frida talking
static void pump(void) {
  for (;;) {
    ssize_t n = read(child_fd, cbuf + clen, sizeof cbuf - clen - 1);
    if (n <= 0) break;
    clen += n;
    size_t s = 0;
    for (size_t i = 0; i < clen; i++) {
      if (cbuf[i] != '\n') continue;
      char *ln = cbuf + s;
      size_t len = i - s + 1;
      if (len >= 2 && strchr("FTIWE", ln[0]) && ln[1] == ' ') {
        if (ln[0] == 'F') failures = 0;
        broadcast(ln, len);
      } else if (len > 1) {
        char m[300];
        int k = snprintf(m, sizeof m, "E frida: %.*s", (int)(len - 1 > 250 ? 250 : len - 1), ln);
        logi("%s", m + 2);
        m[k++] = '\n';
        broadcast(m, k);
      }
      s = i + 1;
    }
    memmove(cbuf, cbuf + s, clen - s);
    clen -= s;
    if (clen >= sizeof cbuf - 1) clen = 0;  // a line this long is garbage
  }
}

// ---------------------------------------------------------------- main
static volatile sig_atomic_t quit;
static void on_term(int s) { (void)s; quit = 1; }

static int listen_tcp(void) {
  int s = socket(AF_INET, SOCK_STREAM, 0), on = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
  struct sockaddr_in a = {0};
  a.sin_family = AF_INET;
  a.sin_port = htons(TCP_PORT);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (struct sockaddr *)&a, sizeof a) || listen(s, 4)) { close(s); return -1; }
  fcntl(s, F_SETFL, O_NONBLOCK);
  return s;
}

static int listen_udp(void) {
  int s = socket(AF_INET, SOCK_DGRAM, 0), on = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
  struct sockaddr_in a = {0};
  a.sin_family = AF_INET;
  a.sin_port = htons(UDP_PORT);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (struct sockaddr *)&a, sizeof a)) { close(s); return -1; }
  fcntl(s, F_SETFL, O_NONBLOCK);
  return s;
}

int main(int argc, char **argv) {
  if (argc > 1) moddir = argv[1];
  signal(SIGPIPE, SIG_IGN);
  signal(SIGTERM, on_term);
  signal(SIGINT, on_term);
  device_info();
  int ts = listen_tcp(), us = listen_udp();
  if (ts < 0 || us < 0) { logi("ports %d/%d busy: already running?", TCP_PORT, UDP_PORT); return 3; }
  kill_strays();
  logi("lhsyncd %s up: serial %s, TCP %d, discovery UDP %d, module %s", MODULE_VERSION, serial, TCP_PORT, UDP_PORT, moddir);
  double idle_since = 0;
  while (!quit) {
    struct pollfd pf[3 + MAXC];
    int np = 0;
    pf[np++] = (struct pollfd){ts, POLLIN, 0};
    pf[np++] = (struct pollfd){us, POLLIN, 0};
    int cpi = -1;
    if (child_fd >= 0) { cpi = np; pf[np++] = (struct pollfd){child_fd, POLLIN, 0}; }
    int c0 = np;
    for (int i = 0; i < ncl; i++) pf[np++] = (struct pollfd){cl[i].fd, POLLIN | (cl[i].olen ? POLLOUT : 0), 0};
    poll(pf, np, 100);
    double now = mono();

    if (pf[1].revents & POLLIN) {  // discovery
      char b[64];
      struct sockaddr_in from;
      socklen_t fl = sizeof from;
      ssize_t n = recvfrom(us, b, sizeof b - 1, 0, (struct sockaddr *)&from, &fl);
      if (n >= 5 && !memcmp(b, "QLHS?", 5)) {
        char r[256];
        int k = snprintf(r, sizeof r, "QLHS 1 %s %d %s %s", serial, TCP_PORT, model[0] ? model : "Quest", child > 0 ? "capturing" : "idle");
        sendto(us, r, k, 0, (struct sockaddr *)&from, fl);
      }
    }
    if (pf[0].revents & POLLIN) {  // a PC connects
      int fd = accept(ts, NULL, NULL);
      if (fd >= 0) {
        if (ncl == MAXC) close(fd);
        else {
          int on = 1, idle = 5, intvl = 2, cnt = 3;
          setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof on);
          setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof on);
          setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof idle);
          setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof intvl);
          setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof cnt);
          struct client *c = &cl[ncl++];
          memset(c, 0, sizeof *c);
          c->fd = fd;
          c->out = malloc(OUTCAP);
          c->last_rx = now;
          if (send_hello(c)) cl_close(ncl - 1);
          else {
            logi("client connected (%d)", ncl);
            if (child > 0 && kill_at) logi("client back while lhsight stops: restarting it after");
            stop_at = 0;
            if (child <= 0) { failures = 0; restart_at = now; }
          }
        }
      }
    }
    // client input: clock round trips
    for (int i = ncl - 1; i >= 0; i--) {
      if (c0 + i >= np) continue;  // accepted just now: no poll entry yet
      short re = pf[c0 + i].revents;
      if (re & (POLLERR | POLLHUP | POLLNVAL)) { cl_close(i); continue; }
      if (re & POLLIN) {
        struct client *c = &cl[i];
        ssize_t n = recv(c->fd, c->in + c->ilen, sizeof c->in - c->ilen - 1, MSG_DONTWAIT);
        if (n <= 0) { cl_close(i); continue; }
        long long t = mono_ns();
        c->ilen += n;
        c->last_rx = now;
        size_t s = 0;
        for (size_t j = 0; j < c->ilen; j++) {
          if (c->in[j] != '\n') continue;
          c->in[j] = 0;
          unsigned seq;
          long long pc;
          if (sscanf(c->in + s, "P %u %lld", &seq, &pc) == 2) {
            char r[96];
            int k = snprintf(r, sizeof r, "Q %u %lld %lld\n", seq, pc, t);
            cl_queue(c, r, k);
          }
          s = j + 1;
        }
        memmove(c->in, c->in + s, c->ilen - s);
        c->ilen -= s;
        if (c->ilen >= sizeof c->in - 1) c->ilen = 0;
      }
    }
    for (int i = ncl - 1; i >= 0; i--)
      if (now - cl[i].last_rx > 15) { logi("client silent for 15 s, dropped"); cl_close(i); }
    if (cpi >= 0 && (pf[cpi].revents & (POLLIN | POLLHUP))) pump();
    reap();
    for (int i = 0; i < ncl; i++) cl_flush(i);

    // lhsight runs while someone listens
    if (ncl) {
      idle_since = 0;
      if (child <= 0 && restart_at && now >= restart_at) { restart_at = 0; start_capture(); }
    } else {
      if (!idle_since) idle_since = now;
      restart_at = 0;
      if (child > 0 && !kill_at && now - idle_since > 5) { logi("no clients: stopping lhsight"); stop_capture(); }
    }
  }
  if (child > 0) {
    kill(child, SIGTERM);
    int i = 0;
    while (i < 30 && waitpid(child, NULL, WNOHANG) == 0) i++, usleep(100000);
    if (i == 30) kill(child, SIGKILL), waitpid(child, NULL, 0);
  }
  logi("lhsyncd exiting");
  return 0;
}
