// lhsyncd: QuestLHSync's headset daemon, shared by the Quest Pro (magisk/: the Magisk module's service.sh starts it)
// and the Steam Frame (frame/: a systemd user service). Serves the tracking cameras' view of lighthouse base station
// laser flashes to the QuestLHSync SteamVR driver on the LAN. The headset's own parts are behind headset.h.
//   UDP 47281  discovery: "QLHS?" -> "QLHS 1 <serial> <tcp port> <model> <idle|capturing>"
//   TCP 47280  per client: "H QuestLHSync 1 serial=.. model=.. fw=.. module=..", "C <len> <name>" + the camera
//              calibration (read-only), then lhsight's lines (F/T/I/W/E) as they come.
//              The client sends "P <seq> <pc_ns>"; the reply "Q <seq> <pc_ns> <mono_ns>" is its clock sync.
// lhsight (the headset's camera reader: reads the camera buffers, writes nothing) runs only while a client is connected,
// and is stopped a few seconds after the last one leaves.
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "headset.h"

#define TCP_PORT 47280
#define UDP_PORT 47281
#define MAXC 4
#define OUTCAP (8 << 20)

static void logi(const char *fmt, ...) {
  char b[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  hs_log(b);
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
  unsigned inputs;
};
static struct client cl[MAXC];
static int ncl;
#ifdef QLHS_FRAME
static int input_owner_fd = -1;
#endif

#ifdef QLHS_FRAME
static void input_status(int up);
#endif

static void cl_close(int i) {
#ifdef QLHS_FRAME
  if (cl[i].fd == input_owner_fd) {
    input_owner_fd = -1;
    logi("input owner disconnected");
  }
#endif
  close(cl[i].fd);
  free(cl[i].out);
  cl[i] = cl[--ncl];
  memset(&cl[ncl], 0, sizeof cl[ncl]);
#ifdef QLHS_FRAME
  if (input_owner_fd < 0 && ncl > 0) {
    input_owner_fd = cl[0].fd;
    logi("input owner client %d", input_owner_fd);
  }
  input_status(input_owner_fd >= 0);
#endif
}

static int cl_queue(struct client *c, const char *d, size_t n) {
  if (c->olen + n > OUTCAP) return -1;  // a client this far behind is gone
  memcpy(c->out + c->olen, d, n);
  c->olen += n;
  return 0;
}

#ifdef QLHS_FRAME
static void cl_send_owner(const char *d, size_t n) {
  if (input_owner_fd < 0) return;
  for (int i = 0; i < ncl; i++)
    if (cl[i].fd == input_owner_fd) {
      cl_queue(&cl[i], d, n);
      return;
    }
}
#endif

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
static char serial[128], model[128], fw[128];

static void underscores(char *s) {
  for (; *s; s++)
    if (*s == ' ' || *s == '\t') *s = '_';
}

static void device_info(void) {
  hs_device_info(serial, model, fw, sizeof serial);
  if (!serial[0]) strcpy(serial, "unknown");
  if (!model[0]) strcpy(model, "headset");
  underscores(serial);
  underscores(model);
  underscores(fw);
}

static int send_hello(struct client *c) {
  char h[512];
  const char *cap =
#ifdef QLHS_FRAME
      " cap=input1";
#else
      "";
#endif
  int n = snprintf(h, sizeof h, "H QuestLHSync 1 serial=%s model=%s fw=%s module=%s%s\n", serial, model, fw, MODULE_VERSION, cap);
  if (cl_queue(c, h, n)) return -1;
  char *buf = NULL, name[256] = "";
  size_t got = 0;
  if (hs_calibration(&buf, &got, name, sizeof name)) {
    n = snprintf(h, sizeof h, "E %s\n", name);
    return cl_queue(c, h, n);
  }
  n = snprintf(h, sizeof h, "C %zu %s\n", got, name);
  int r = cl_queue(c, h, n) || cl_queue(c, buf, got);
  free(buf);
  return r ? -1 : 0;
}

// ---------------------------------------------------------------- lhsight (the headset's camera reader)
static pid_t child;
static int child_fd = -1;
static char cbuf[1 << 16];
static size_t clen;
static double stop_at, kill_at, restart_at;
static int failures;

static void start_capture(void) {
  int p[2];
  if (pipe(p)) return;
  char msg[480] = "";
  pid_t pid = hs_capture_start(p[1], msg, sizeof msg);
  close(p[1]);
  if (pid <= 0) {
    close(p[0]);
    if (pid == 0) { say("E %s", msg); restart_at = mono() + 5; }
    return;
  }
  child = pid;
  child_fd = p[0];
  fcntl(child_fd, F_SETFL, O_NONBLOCK);
  clen = 0;
  stop_at = kill_at = 0;
  say("I lhsyncd: %s", msg);
}

static void stop_capture(void) {
  if (child <= 0) return;
  kill(child, SIGTERM);  // the reader lets go of the camera buffers
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

// lhsight's output -> clients, whole lines; anything that isn't one of its records is the reader's runtime talking
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
        int k = snprintf(m, sizeof m, "E %s: %.*s", hs_capture_name, (int)(len - 1 > 250 ? 250 : len - 1), ln);
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

// ---------------------------------------------------------------- controller input from the PC
// The host sends "X ..." lines; hand them to the Frame SteamVR driver over a same-user datagram
// socket. The driver creates the socket when SteamVR starts; sends before that simply fail.
#ifdef QLHS_FRAME
// SteamVR and the systemd user unit can disagree about XDG_RUNTIME_DIR (nested Plasma vs
// /run/user/<uid>). Use the stable per-user runtime dir when it exists so both ends meet.
static const char *runtime_dir(void) {
  static char dir[64];
  snprintf(dir, sizeof dir, "/run/user/%u", (unsigned)getuid());
  struct stat st;
  if (stat(dir, &st) == 0) return dir;
  const char *e = getenv("XDG_RUNTIME_DIR");
  return e && *e ? e : "";
}

static int input_fd = -1;
static struct sockaddr_un input_addr;

static void input_init(void) {
  const char *rt = runtime_dir();
  if (!rt || !*rt) return;
  char dir[256], path[256];
  snprintf(dir, sizeof dir, "%s/questlhsync", rt);
  if (mkdir(dir, 0700) != 0 && errno != EEXIST) logi("input dir %s: %s", dir, strerror(errno));
  int n = snprintf(path, sizeof path, "%s/input.sock", dir);
  if (n <= 0 || (size_t)n >= sizeof input_addr.sun_path) return;
  input_fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (input_fd < 0) return;
  memset(&input_addr, 0, sizeof input_addr);
  input_addr.sun_family = AF_UNIX;
  memcpy(input_addr.sun_path, path, (size_t)n + 1);
}

static void input_forward(const char *line, size_t len) {
  if (input_fd >= 0 && len)
    sendto(input_fd, line, len, MSG_DONTWAIT | MSG_NOSIGNAL, (struct sockaddr *)&input_addr,
           sizeof input_addr);
}

static int input_status_last = -1;
static void input_status(int up) {
  up = up != 0;
  if (up == input_status_last) return;
  input_status_last = up;
  input_forward(up ? "G 1\n" : "G 0\n", 4);
  logi("input link %s", up ? "up" : "down");
}

// The Frame driver writes "V ..." haptic events to this socket; relay them to every PC client.
static int haptic_fd = -1;
static char haptic_path[256];

static void haptic_init(void) {
  if (haptic_fd >= 0) return;
  const char *rt = runtime_dir();
  if (!rt || !*rt) {
    logi("haptic relay: no runtime directory, retrying");
    return;
  }
  char dir[256], path[256];
  snprintf(dir, sizeof dir, "%s/questlhsync", rt);
  if (mkdir(dir, 0700) != 0 && errno != EEXIST) logi("haptic dir %s: %s", dir, strerror(errno));
  int n = snprintf(path, sizeof path, "%s/haptic.sock", dir);
  struct sockaddr_un a = {0};
  a.sun_family = AF_UNIX;
  if (n <= 0 || (size_t)n >= sizeof a.sun_path) return;
  memcpy(a.sun_path, path, (size_t)n + 1);
  haptic_fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (haptic_fd < 0) return;
  unlink(path);
  if (bind(haptic_fd, (struct sockaddr *)&a, sizeof a) != 0) {
    logi("haptic relay bind %s: %s (retrying)", path, strerror(errno));
    close(haptic_fd);
    haptic_fd = -1;
    return;
  }
  if (chmod(path, 0600) != 0) logi("haptic chmod %s: %s", path, strerror(errno));
  fcntl(haptic_fd, F_SETFL, O_NONBLOCK);
  snprintf(haptic_path, sizeof haptic_path, "%s", path);
  logi("haptic relay up on %s", path);
}
#endif

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
  signal(SIGPIPE, SIG_IGN);
  signal(SIGTERM, on_term);
  signal(SIGINT, on_term);
  device_info();
#ifdef QLHS_FRAME
  input_init();
  haptic_init();
#endif
  int ts = listen_tcp(), us = listen_udp();
  if (ts < 0 || us < 0) { logi("ports %d/%d busy: already running?", TCP_PORT, UDP_PORT); return 3; }
  hs_init(argc, argv);
  logi("lhsyncd %s up: serial %s, TCP %d, discovery UDP %d", MODULE_VERSION, serial, TCP_PORT, UDP_PORT);
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
        char r[320];
        int k = snprintf(r, sizeof r, "QLHS 1 %s %d %s %s", serial, TCP_PORT, model, child > 0 ? "capturing" : "idle");
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
#ifdef QLHS_FRAME
            if (input_owner_fd < 0) {
              input_owner_fd = c->fd;
              logi("input owner client %d", c->fd);
            }
            input_status(input_owner_fd >= 0);
#endif
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
#ifdef QLHS_FRAME
          else if ((c->in[s] == 'X' || c->in[s] == 'D') && c->in[s + 1] == ' ') {
            if (c->fd != input_owner_fd) {
              if (c->inputs++ == 0) logi("controller input ignored from non-owner");
            } else {
              input_forward(c->in + s, j - s);
              if (c->inputs++ == 0) logi("controller input: %.100s", c->in + s);
            }
          }
#endif
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

#ifdef QLHS_FRAME
    static double next_haptic_try;
    if (haptic_fd < 0 && now >= next_haptic_try) {
      next_haptic_try = now + 2.0;
      haptic_init();
    }
    if (haptic_fd >= 0) {
      char b[256];
      for (;;) {
        ssize_t n = recv(haptic_fd, b, sizeof b, 0);
        if (n <= 0) break;
        cl_send_owner(b, (size_t)n);
      }
    }

    static double last_input_hb;
    if (now - last_input_hb >= 1.0) {
      last_input_hb = now;
      input_forward(input_owner_fd >= 0 ? "G 1\n" : "G 0\n", 4);
    }
#endif

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
#ifdef QLHS_FRAME
  if (haptic_fd >= 0) {
    close(haptic_fd);
    haptic_fd = -1;
  }
  if (haptic_path[0]) unlink(haptic_path);
#endif
  if (child > 0) {
    kill(child, SIGTERM);
    int i = 0;
    while (i < 30 && waitpid(child, NULL, WNOHANG) == 0) i++, usleep(100000);
    if (i == 30) kill(child, SIGKILL), waitpid(child, NULL, 0);
  }
  logi("lhsyncd exiting");
  return 0;
}
