// lhsight (Steam Frame): bright-spot detector for the four tracking cameras, run by lhsyncd in a child process. Reads
// XRService's camera buffers and writes nothing: the buffer list comes from VIDIOC_QUERYBUF on the cameras' own nodes
// (anyone may query a queue), the buffers themselves from the questlhsync_frame driver in the Frame's vrserver (see
// frame/driver/xrfds.cpp). Every new frame: its mean (exposure class), and on short-exposure frames (the controller
// LED frames, 8.4 ms after each long one, every 30 ms) the saturated blobs, then dimmer ones. Lines on stdout:
//   F cam seq t_us mean nblobs [x y npx peak]...   t: the frame's own timestamp on CLOCK_MONOTONIC, mean * 10,
//                                                  nblobs -1 on long frames, x y in pixels * 10 (pixel centres at .5)
//   T t_us scans n scan_us u torn n polls n lag_us l   once a second
//   I/W/E messages
// Exits when XRService or its buffers change (lhsyncd starts it again).
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "lhsight.h"

#define NCAM 4    // XRService's tracking cameras, its indices = the calibration's cameras
#define NBUF 16   // per camera ring (QUERYBUF stops there)
#define MAXB 48   // blobs per frame
#define THR 64    // the dim pass: 4x the frame's mean, at least THR (a dot that doesn't saturate, in a light room)
#define QUIET 0.006     // s: a camera's next frame comes 8.4 ms after a long one, 21.6 ms after a short one
#define STALLED 0.25    // s without a frame: the cameras are paused (headset off), poll slowly

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

struct cam {
  char dev[64];
  int vfd, w, h, stride, nb;
  int xfd[NBUF];  // XRService's fd numbers
  int fd[NBUF];   // ours
  u8 *map[NBUF];
  size_t len[NBUF];
  unsigned seq[NBUF];
  u32 k;          // frames seen
  int m1, m2;     // the previous two means
  double last, prev, prev2;  // when its last three frames were found (CLOCK_MONOTONIC s)
  double polled;             // when it was last polled
};
static struct cam C[NCAM];
static u8 *scratch;
static u16 *cells;
static int *lab;
static u8 *hot;  // cells the dim pass leaves to the saturated one
static u32 nscan, ntorn, npoll;
static u64 scan_ns;
static double lag_sum;
static u32 lag_n;
static clockid_t ts_clock = -1;  // what the V4L2 timestamps count

static void out(const char *fmt, ...) {
  char b[2600];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(b, sizeof b - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof b - 2) n = sizeof b - 2;
  b[n++] = '\n';
  for (int o = 0; o < n;) {
    ssize_t w = write(1, b + o, n - o);
    if (w <= 0) { if (errno == EINTR) continue; _exit(1); }  // lhsyncd is gone
    o += w;
  }
}

static double now_s(clockid_t c) {
  struct timespec t;
  clock_gettime(c, &t);
  return t.tv_sec + t.tv_nsec / 1e9;
}

static u64 now_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (u64)t.tv_sec * 1000000000ull + t.tv_nsec;
}

// ---------------------------------------------------------------- finding XRService's cameras
static pid_t xrservice_pid(void) {
  DIR *d = opendir("/proc");
  if (!d) return 0;
  struct dirent *e;
  pid_t found = 0;
  while (!found && (e = readdir(d))) {
    if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
    char p[300], cmd[512];
    snprintf(p, sizeof p, "/proc/%s/cmdline", e->d_name);
    int fd = open(p, O_RDONLY);
    if (fd < 0) continue;
    ssize_t n = read(fd, cmd, sizeof cmd - 1);
    close(fd);
    if (n <= 0) continue;
    cmd[n] = 0;
    const char *base = strrchr(cmd, '/') ? strrchr(cmd, '/') + 1 : cmd;
    if (!strcmp(base, "XRService")) found = atoi(e->d_name);
  }
  closedir(d);
  return found;
}

// XRService logs which node each of its camera indices streams from:
//   "TrackingCameraInit: index: 0. video device: /dev/video0. v4l subdevice: /dev/v4l-subdev31"
static int camera_nodes(pid_t xr) {
  char dir[64], log[512] = "";
  snprintf(dir, sizeof dir, "/proc/%d/fd", xr);
  DIR *d = opendir(dir);
  if (!d) return 0;
  struct dirent *e;
  while ((e = readdir(d))) {
    char p[400], l[512];
    snprintf(p, sizeof p, "%s/%s", dir, e->d_name);
    ssize_t n = readlink(p, l, sizeof l - 1);
    if (n <= 0) continue;
    l[n] = 0;
    if (strstr(l, "/XRService-") && n > 4 && !strcmp(l + n - 4, ".log")) { snprintf(log, sizeof log, "%s", l); break; }
  }
  closedir(d);
  FILE *f = log[0] ? fopen(log, "r") : NULL;
  if (!f) return 0;
  char line[1024];
  int found = 0;
  while (fgets(line, sizeof line, f)) {
    const char *s = strstr(line, "TrackingCameraInit: index: ");
    int i;
    char dev[64];
    if (!s || sscanf(s, "TrackingCameraInit: index: %d. video device: %63[^. \n]", &i, dev) != 2 || i < 0 || i >= NCAM) continue;
    if (!C[i].dev[0]) found++;
    snprintf(C[i].dev, sizeof C[i].dev, "%s", dev);  // the last start's wins
  }
  fclose(f);
  return found;
}

static int querybuf(struct cam *c, int i, struct v4l2_buffer *b, struct v4l2_plane *pl) {
  memset(b, 0, sizeof *b);
  memset(pl, 0, VIDEO_MAX_PLANES * sizeof *pl);
  b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  b->index = i;
  b->m.planes = pl;
  b->length = VIDEO_MAX_PLANES;
  return ioctl(c->vfd, VIDIOC_QUERYBUF, b);
}

static int open_camera(struct cam *c) {
  c->vfd = open(c->dev, O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (c->vfd < 0) { out("E can't open %s: %s", c->dev, strerror(errno)); return -1; }
  struct v4l2_format f = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE};
  if (ioctl(c->vfd, VIDIOC_G_FMT, &f)) { out("E %s: VIDIOC_G_FMT: %s", c->dev, strerror(errno)); return -1; }
  c->w = f.fmt.pix_mp.width;
  c->h = f.fmt.pix_mp.height;
  if (c->w % 16 || c->h % 16 || c->w > 2048 || c->h > 2048) { out("E %s: unexpected size %dx%d", c->dev, c->w, c->h); return -1; }
  for (c->nb = 0; c->nb < NBUF; c->nb++) {
    struct v4l2_buffer b;
    struct v4l2_plane pl[VIDEO_MAX_PLANES];
    if (querybuf(c, c->nb, &b, pl)) break;
    if (b.memory != V4L2_MEMORY_DMABUF) { out("E %s: buffers aren't dmabufs", c->dev); return -1; }
    c->xfd[c->nb] = pl[0].m.fd;
    c->len[c->nb] = pl[0].length;
    c->seq[c->nb] = b.sequence;
  }
  if (!c->nb) { out("E %s: no buffers (is XRService streaming?)", c->dev); return -1; }
  // The planes are laid out as NV12 (luma, then half as many rows for chroma that a mono camera leaves empty), so the
  // luma's row pitch is the buffer over 1.5 heights: 1152 for the 1056-wide cameras, where V4L2 reports 1056.
  size_t rows = (size_t)c->h * 3 / 2;
  c->stride = c->len[0] % rows == 0 && c->len[0] / rows >= (size_t)c->w ? (int)(c->len[0] / rows) : c->w;
  if ((size_t)c->stride * (c->h - 1) + c->w > c->len[0]) { out("E %s: buffer too small", c->dev); return -1; }
  return 0;
}

// XRService's buffers, lent by the questlhsync_frame driver (in order)
static int borrow(pid_t xr) {
  int s = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  struct sockaddr_un a = {.sun_family = AF_UNIX};
  const char *rt = getenv("XDG_RUNTIME_DIR");
  snprintf(a.sun_path, sizeof a.sun_path, "%s/questlhsync/xrfds.sock", rt ? rt : "/run/user/1000");
  if (connect(s, (struct sockaddr *)&a, sizeof a)) {
    out("E the questlhsync_frame SteamVR driver isn't running on the headset (%s): install it and restart SteamVR",
        strerror(errno));
    close(s);
    return -1;
  }
  char req[1024];
  int n = snprintf(req, sizeof req, "G"), want = 0;
  for (int k = 0; k < NCAM; k++)
    for (int i = 0; i < C[k].nb; i++) n += snprintf(req + n, sizeof req - n, " %d", C[k].xfd[i]), want++;
  req[n++] = '\n';
  if (send(s, req, n, MSG_NOSIGNAL) != n) { close(s); out("E the driver hung up"); return -1; }
  char txt[256] = "";
  char ctl[CMSG_SPACE(sizeof(int) * NCAM * NBUF)];
  struct iovec io = {txt, sizeof txt - 1};
  struct msghdr m = {.msg_iov = &io, .msg_iovlen = 1, .msg_control = ctl, .msg_controllen = sizeof ctl};
  struct pollfd pf = {s, POLLIN, 0};
  ssize_t r = poll(&pf, 1, 3000) == 1 ? recvmsg(s, &m, MSG_CMSG_CLOEXEC) : -1;
  close(s);
  if (r <= 0) { out("E no answer from the questlhsync_frame driver"); return -1; }
  txt[r] = 0;
  int pid = 0, got = 0;
  if (sscanf(txt, "OK %d %d", &pid, &got) != 2) { out("E driver: %s", txt[0] == 'E' ? txt + 2 : txt); return -1; }
  struct cmsghdr *h = CMSG_FIRSTHDR(&m);
  int nfd = h && h->cmsg_type == SCM_RIGHTS ? (int)((h->cmsg_len - CMSG_LEN(0)) / sizeof(int)) : 0;
  int fds[NCAM * NBUF];
  if (nfd > 0) memcpy(fds, CMSG_DATA(h), sizeof(int) * (nfd < NCAM * NBUF ? nfd : NCAM * NBUF));
  if (pid != xr || got != want || nfd != want) {
    for (int i = 0; i < nfd && i < NCAM * NBUF; i++) close(fds[i]);
    out("E the driver lent %d of %d buffers of XRService %d (wanted %d's)", nfd, want, pid, (int)xr);
    return -1;
  }
  int j = 0;
  for (int k = 0; k < NCAM; k++)
    for (int i = 0; i < C[k].nb; i++, j++) {
      C[k].fd[i] = fds[j];
      C[k].map[i] = mmap(NULL, C[k].len[i], PROT_READ, MAP_SHARED, fds[j], 0);
      if (C[k].map[i] == MAP_FAILED) { out("E mmap %s buffer %d: %s", C[k].dev, i, strerror(errno)); return -1; }
    }
  return 0;
}

// ---------------------------------------------------------------- frames
static void cpu_access(int fd, int end) {  // the cameras write behind the CPU's caches
  struct dma_buf_sync s = {.flags = DMA_BUF_SYNC_READ | (end ? DMA_BUF_SYNC_END : DMA_BUF_SYNC_START)};
  ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
}

static int smean(const struct cam *c, const u8 *p) {
  u32 n = (u32)(c->w * c->h), step = n / 4096, s = 0, k = 0;
  for (u32 o = step / 2; o < n; o += step) { s += p[(o / c->w) * c->stride + o % c->w]; k++; }
  return (int)(s * 10 / k);  // mean * 10
}

static int find(int x) {
  while (lab[x] != x) { lab[x] = lab[lab[x]]; x = lab[x]; }
  return x;
}

// The blobs of pixels >= T in scratch (w x h, packed): 16x16 cells holding one, 8-connected, centroid of those pixels;
// added to the arrays from nbl on, up to MAXB. Cells marked hot are left out, and the first pass marks its cells and
// their neighbours (mark), so a later, dimmer pass doesn't take in a saturated blob's glow.
static int blobs(int w, int h, int T, int mark, int nbl, u64 *sx, u64 *sy, u32 *np, int *pk) {
  int cw = w >> 4, ch = h >> 4, nc = cw * ch;
  for (int q = 0; q < nc; q++) cells[q] = 0;
  // fast reject: no byte > T - 1 in this word (at most 127: brighter thresholds only reject less)
  const u64 add = 0x0101010101010101ull * (u64)(127 - (T - 1 > 127 ? 127 : T - 1));
  const u64 *q8 = (const u64 *)scratch;
  u32 n = (u32)(w * h), nw = n >> 3;
  for (u32 j = 0; j < nw; j++) {
    u64 x = q8[j];
    if (!(((x + add) | x) & 0x8080808080808080ull)) continue;
    for (int bb = 0; bb < 8; bb++) {
      if ((int)((x >> (8 * bb)) & 0xFF) < T) continue;
      u32 px = (j << 3) + bb;
      int y = (int)(px / (u32)w), xx = (int)(px - (u32)y * (u32)w);
      int q = (y >> 4) * cw + (xx >> 4);
      if (hot[q]) continue;
      if (cells[q] < 65535) cells[q]++;
    }
  }
  for (int q = 0; q < nc; q++) lab[q] = q;
  for (int cy = 0; cy < ch; cy++)
    for (int cx = 0; cx < cw; cx++) {
      int q = cy * cw + cx;
      if (!cells[q]) continue;
      if (mark)
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++)
            if (cy + dy >= 0 && cy + dy < ch && cx + dx >= 0 && cx + dx < cw) hot[q + dy * cw + dx] = 1;
      int nb[4] = {cx > 0 ? q - 1 : -1, cy > 0 ? q - cw : -1, (cy > 0 && cx > 0) ? q - cw - 1 : -1,
                   (cy > 0 && cx < cw - 1) ? q - cw + 1 : -1};
      for (int e = 0; e < 4; e++)
        if (nb[e] >= 0 && cells[nb[e]]) {
          int r1 = find(q), r2 = find(nb[e]);
          if (r1 != r2) lab[r1] = r2;
        }
    }
  int roots[MAXB], b0 = nbl;
  for (int q = 0; q < nc; q++) {
    if (!cells[q]) continue;
    int r = find(q), bi = -1;
    for (int e = b0; e < nbl; e++) if (roots[e] == r) { bi = e; break; }
    if (bi < 0) {
      if (nbl == MAXB) continue;
      bi = nbl++;
      roots[bi] = r;
      sx[bi] = sy[bi] = 0;
      np[bi] = 0;
      pk[bi] = 0;
    }
    int x0 = (q % cw) * 16, y0 = (q / cw) * 16;
    for (int y = y0; y < y0 + 16; y++) {
      const u8 *row = scratch + (u32)y * (u32)w;
      for (int x = x0; x < x0 + 16; x++) {
        int v = row[x];
        if (v > pk[bi]) pk[bi] = v;
        if (v < T) continue;
        sx[bi] += (u64)(x * 10 + 5);
        sy[bi] += (u64)(y * 10 + 5);
        np[bi]++;
      }
    }
  }
  return nbl;
}

// V4L2 stamps the frames on CLOCK_MONOTONIC_RAW here (the buffers claim MONOTONIC): take whichever the newest frame
// is just behind
static void pick_clock(double ts) {
  double raw = now_s(CLOCK_MONOTONIC_RAW) - ts, mono = now_s(CLOCK_MONOTONIC) - ts;
  ts_clock = raw >= 0 && raw < 0.2 ? CLOCK_MONOTONIC_RAW : CLOCK_MONOTONIC;
  out("I frame timestamps: CLOCK_%s (%.1f ms behind it, %.1f ms behind the other)",
      ts_clock == CLOCK_MONOTONIC_RAW ? "MONOTONIC_RAW" : "MONOTONIC", (ts_clock == CLOCK_MONOTONIC_RAW ? raw : mono) * 1e3,
      (ts_clock == CLOCK_MONOTONIC_RAW ? mono : raw) * 1e3);
}

static void frame(int k, int i, const struct v4l2_buffer *b) {
  struct cam *c = &C[k];
  double ts = b->timestamp.tv_sec + b->timestamp.tv_usec / 1e6;
  if (ts_clock < 0) pick_clock(ts);
  double cnow = now_s(ts_clock), mnow = now_s(CLOCK_MONOTONIC);
  u64 t_us = (u64)((ts + (mnow - cnow)) * 1e6);
  lag_sum += cnow - ts;
  lag_n++;
  cpu_access(c->fd[i], 0);
  const u8 *p = c->map[i];
  int mean = smean(c, p);
  u32 kk = c->k++;
  int hi = c->m1 > c->m2 ? c->m1 : c->m2;
  c->m2 = c->m1;
  c->m1 = mean;
  // short exposure: well under the brighter of the previous two (long and short frames alternate)
  int is_short = kk >= 2 && mean * 10 < hi * 6;
  u64 t0 = now_ns();
  if (is_short)
    for (int y = 0; y < c->h; y++) memcpy(scratch + (size_t)y * c->w, p + (size_t)y * c->stride, c->w);
  cpu_access(c->fd[i], 1);
  // XRService may have queued it again while this read it: then the camera may have been writing into it
  struct v4l2_buffer b2;
  struct v4l2_plane pl[VIDEO_MAX_PLANES];
  if (querybuf(c, i, &b2, pl) || (b2.flags & V4L2_BUF_FLAG_QUEUED) || b2.sequence != b->sequence) {
    ntorn++;
    return;
  }
  char line[2400];
  int L = snprintf(line, sizeof line, "F %d %u %llu %d", k, b->sequence, (unsigned long long)t_us, mean);
  if (!is_short) L += snprintf(line + L, sizeof line - L, " -1");
  else {
    // saturated blobs (base station dots, lamps), then dimmer ones in the cells they leave alone
    u64 sx[MAXB], sy[MAXB];
    u32 np[MAXB];
    int pk[MAXB], T = mean * 4 / 10, nc = (c->w >> 4) * (c->h >> 4);
    if (T < THR) T = THR;
    memset(hot, 0, (size_t)nc);
    int nbl = blobs(c->w, c->h, 250, 1, 0, sx, sy, np, pk);
    if (T < 250) nbl = blobs(c->w, c->h, T, 0, nbl, sx, sy, np, pk);
    L += snprintf(line + L, sizeof line - L, " %d", nbl);
    for (int e = 0; e < nbl && L < (int)sizeof line - 40; e++)
      L += snprintf(line + L, sizeof line - L, " %llu %llu %u %d", (unsigned long long)(sx[e] / np[e]),
                    (unsigned long long)(sy[e] / np[e]), np[e], pk[e]);
    scan_ns += now_ns() - t0;
    nscan++;
  }
  out("%.*s", L, line);
}

static void poll_camera(int k) {
  struct cam *c = &C[k];
  npoll++;
  int idx[NBUF], n = 0;
  struct v4l2_buffer b[NBUF];
  struct v4l2_plane pl[NBUF][VIDEO_MAX_PLANES];
  for (int i = 0; i < c->nb; i++) {
    if (querybuf(c, i, &b[i], pl[i])) { out("W %s: VIDIOC_QUERYBUF: %s", c->dev, strerror(errno)); exit(4); }
    if (pl[i][0].m.fd != c->xfd[i]) { out("W %s: XRService replaced its buffers", c->dev); exit(4); }
    // a new frame: XRService has it (dequeued) or it's done and waiting for XRService
    if (!(b[i].flags & V4L2_BUF_FLAG_QUEUED) && b[i].sequence != c->seq[i]) idx[n++] = i;
  }
  for (int a = 1; a < n; a++)  // oldest first
    for (int j = a; j > 0 && b[idx[j]].sequence < b[idx[j - 1]].sequence; j--) { int t = idx[j]; idx[j] = idx[j - 1]; idx[j - 1] = t; }
  for (int j = 0; j < n; j++) {
    c->seq[idx[j]] = b[idx[j]].sequence;
    frame(k, idx[j], &b[idx[j]]);
  }
  if (n) {
    c->prev2 = c->prev;
    c->prev = c->last;
    c->last = now_s(CLOCK_MONOTONIC);
  }
}

// when to look at a camera again: every ms from shortly before its next frame (the gaps alternate, 8.4 and 21.6 ms:
// the next one is like the one before the last), every 20 ms while it's paused
static double due(const struct cam *c, double t) {
  if (!c->last || t - c->last > STALLED) return c->polled + 0.02;
  double gap = c->prev2 ? c->prev - c->prev2 : 0, d = c->last + (gap > QUIET + 0.0015 && gap < 0.1 ? gap - 0.0015 : QUIET);
  return d > c->polled + 0.001 ? d : c->polled + 0.001;
}

int lhsight_main(void) {
  pid_t xr = xrservice_pid();
  if (!xr) { out("E XRService (the Frame's tracking) isn't running"); return 2; }
  if (camera_nodes(xr) != NCAM) { out("E XRService's log doesn't say which nodes its %d cameras stream from", NCAM); return 2; }
  int maxw = 0, maxh = 0;
  for (int k = 0; k < NCAM; k++) {
    if (open_camera(&C[k])) return 2;
    if (C[k].w > maxw) maxw = C[k].w;
    if (C[k].h > maxh) maxh = C[k].h;
  }
  if (borrow(xr)) return 2;
  int pidfd = (int)syscall(SYS_pidfd_open, xr, 0);
  scratch = aligned_alloc(64, (size_t)maxw * maxh);
  cells = malloc((size_t)(maxw / 16) * (maxh / 16) * sizeof *cells);
  lab = malloc((size_t)(maxw / 16) * (maxh / 16) * sizeof *lab);
  hot = malloc((size_t)(maxw / 16) * (maxh / 16));
  char desc[400];
  int L = 0;
  for (int k = 0; k < NCAM; k++)
    L += snprintf(desc + L, sizeof desc - L, "%s%d %s %dx%d/%d x%d", k ? ", " : "", k, C[k].dev, C[k].w, C[k].h, C[k].stride, C[k].nb);
  out("I lhsight: XRService %d, cameras %s", (int)xr, desc);
  double next_beat = now_s(CLOCK_MONOTONIC) + 1;
  for (;;) {
    double t = now_s(CLOCK_MONOTONIC), wake = t + 0.02;
    for (int k = 0; k < NCAM; k++) {
      struct cam *c = &C[k];
      if (t >= due(c, t)) {
        poll_camera(k);
        c->polled = t = now_s(CLOCK_MONOTONIC);
      }
      double d = due(c, t);
      if (d < wake) wake = d;
    }
    t = now_s(CLOCK_MONOTONIC);
    if (t >= next_beat) {
      next_beat = t + 1;
      struct pollfd pf = {pidfd, POLLIN, 0};
      if (pidfd >= 0 && poll(&pf, 1, 0) == 1) { out("W XRService exited"); return 3; }
      out("T %llu scans %u scan_us %.0f torn %u polls %u lag_us %.0f", (unsigned long long)(t * 1e6), nscan,
          nscan ? scan_ns / 1e3 / nscan : 0.0, ntorn, npoll, lag_n ? lag_sum / lag_n * 1e6 : 0.0);
      lag_sum = 0;
      lag_n = 0;
    }
    if (wake > t) usleep((useconds_t)((wake - t) * 1e6));
  }
}
