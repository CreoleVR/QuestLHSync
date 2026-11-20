// picosight: lhsight for the PICO 4, run as a Frida CModule inside pvrtrackingservice. Every camera frame the tracking
// service gets passes through pvr::AlgorithmManager::updateSlamImage(std::vector<alg_frame_item_t>, bool head, int):
// the four tracking cameras at once, head (SLAM, long exposure) or controller (LED, short exposure) frames. on_enter
// (the tracking service's thread) copies the frames of the cameras picosight scans into a ring and queues them; scan()
// (Frida's JS thread) finds the bright spots, as lhsight does on a Quest, and writes the same lines:
//   F cam k t_us mean nblobs [x y npx peak]...      (x,y in pixels*10; nblobs -1: a frame that wasn't scanned)
// Reads only: the frames are copied, nothing the tracking service has is written.
//
// alg_frame_item_t, 72 bytes (libpvrtrackingservice.so: pvr::brightnessValue, AlgorithmManager::updateSlamImage):
//   +4 width, +8 height (u32), +12 format (36, 291: 8 bit; 32: 16 bit), +20 exposure, +24 gain, +40 timestamp (ns),
//   +64 pixels
#include <gum/guminterceptor.h>

typedef unsigned char u8; typedef unsigned short u16; typedef unsigned int u32; typedef unsigned long long u64;
typedef long long s64;
struct ts { long s; long ns; };
extern int clock_gettime(int, struct ts *);
extern void *memcpy(void *, const void *, unsigned long);
extern int snprintf(char *, unsigned long, const char *, ...);
// the ARM arch counter and its rate: two-instruction routines picosight.js writes (TinyCC has no arm64 inline asm)
extern u64 cntvct(void);
extern u64 cntfrq(void);

#define ITEM 72
#define NCAM 4
#define NSLOT 4                                // copies kept per camera
#define MAXPIX (640 * 512)                     // a slot: a 640x480 frame and then some
#define NEV 256                                // queued frames
#define MAXB 48
#define MAXCELL 2048
#define OUTCAP (1 << 20)
#define CORE_PX 30                             // blobs this big get their centre where they're thickest (core())
#define CORE_BOX 160                           // ... unless their box is wider or taller than this

struct ev { int cam, slot, head; u32 k, seq; s64 t; int mean; };
struct st {
  int on;                                      // set by JS once the buffers are in
  volatile gint barrier;                       // MB()'s
  int scan[NCAM];                              // cameras whose frames are scanned
  u8 *ring;                                    // NCAM * NSLOT * MAXPIX
  u32 seq[NCAM][NSLOT];                        // a slot's frame number, 0 while being written
  int sw[NCAM][NSLOT], sh[NCAM][NSLOT];
  int next[NCAM];
  u32 k[NCAM], nseq;
  struct ev q[NEV]; volatile u32 qhead, qtail;  // on_enter writes qhead, scan() qtail
  int clock;                                   // the frames' clock: 0 unknown, 1 boottime, 2 monotonic, 3 arch counter
  u64 frq;                                     // the arch counter's rate (Hz), 0: none
  s64 age_max;                                 // since the last heartbeat: the oldest frame when it came (ns)
  u32 ncall[2], nqdrop, nlap, nbadclk, badfmt, nsmall;
  u8 hot[MAXCELL]; u16 *cells; int *lab;
  u16 dist[CORE_BOX * CORE_BOX];               // core()'s distance transform
  char *out; u32 head, tail, ndrop;
  u64 scan_ns; u32 nscan;
  int thr;
  int max_gain;                                // frames over this gain aren't scanned (set_max_gain)
  u32 ngain;                                   // frames left unscanned for their gain
  u8 item0[ITEM]; int item0_set;               // the first item seen, for the log
};
extern struct st S;
// a full barrier: TinyCC (Frida's CModule compiler) has no __sync builtins, glib's atomics are sequentially consistent
#define MB() g_atomic_int_add(&S.barrier, 0)

static s64 clk(int id) { struct ts t; clock_gettime(id, &t); return (s64)t.s * 1000000000ll + (s64)t.ns; }

void setup(u8 *ring, u16 *cells, int *lab, char *out, int thr) {
  S.ring = ring; S.cells = cells; S.lab = lab; S.out = out; S.thr = thr; S.head = S.tail = 0;
}
void set_scan(int cam, int on) { if (cam >= 0 && cam < NCAM) S.scan[cam] = on; }
// Frames over this gain left unscanned (0: none). Off by default: a 1.0 base station's flash blooms with the gain, but
// low-gain frames come only in some rooms (one PICO 4 in a dark room: several a second; lit: none), and core() keeps
// the bloomed ones' centres on the station
void set_max_gain(int g) { S.max_gain = g; }
u32 n_gain(void) { return S.ngain; }
void start(void) { S.frq = cntfrq(); MB(); S.on = 1; }
void stop(void) { S.on = 0; MB(); }
int state_size(void) { return (int)sizeof(struct st); }
u32 n_drop(void) { return S.ndrop; }
u64 scan_ns(void) { return S.scan_ns; }
u32 n_scan(void) { return S.nscan; }
u32 n_call(int head) { return S.ncall[head & 1]; }
u32 n_qdrop(void) { return S.nqdrop; }
u32 n_lap(void) { return S.nlap; }
u32 n_badclk(void) { return S.nbadclk; }
u32 bad_fmt(void) { return S.badfmt; }
int clock_kind(void) { return S.clock; }
s64 take_age_max(void) { s64 a = S.age_max; S.age_max = 0; return a; }
int item0(u8 *dst) { if (!S.item0_set) return 0; memcpy(dst, S.item0, ITEM); return ITEM; }

// the arch counter in ns (it counts from power-on, seconds before BOOTTIME starts)
static s64 cnt_ns(void) {
  if (!S.frq) return 0;
  u64 c = cntvct();
  return (s64)((c / S.frq) * 1000000000ull + (c % S.frq) * 1000000000ull / S.frq);
}

// the frame's time on CLOCK_MONOTONIC. PICO OS 5.13 stamps frames with the arch counter in ns (sensors_timeutil: they
// come 10-17 ms after their stamp, which is 4.75 s ahead of BOOTTIME), so that clock goes first; the others in case
// another build stamps them otherwise. The one that puts the frame a moment ago is taken
static s64 to_mono(s64 ts, s64 mono, s64 boot, s64 cnt) {
  s64 ac = cnt - ts, ab = boot - ts, am = mono - ts;
  int okc = cnt && ac > -5000000ll && ac < 500000000ll;
  int okb = ab > -5000000ll && ab < 500000000ll, okm = am > -5000000ll && am < 500000000ll;
  if (okc) { S.clock = 3; if (ac > S.age_max) S.age_max = ac; return ts - (cnt - mono); }
  if (okb && (!okm || S.clock != 2)) { S.clock = 1; if (ab > S.age_max) S.age_max = ab; return ts - (boot - mono); }
  if (okm) { S.clock = 2; if (am > S.age_max) S.age_max = am; return ts; }
  S.nbadclk++;
  return mono;  // neither: when it came, roughly
}

// the mean of a frame * 10, from 1024 of its pixels
static int smean(const u8 *p, u32 n, int bpp) {
  u32 step = n / 1024, s = 0, c = 0;
  if (!step) step = 1;
  for (u32 o = step / 2; o < n; o += step) { s += bpp == 2 ? (u32)(((const u16 *)p)[o] >> 2) : p[o]; c++; }
  return c ? (int)(s * 10 / c) : 0;
}

static void copy8(u8 *dst, const u8 *src, u32 n, int bpp) {
  if (bpp == 1) { memcpy(dst, src, n); return; }
  const u16 *s = (const u16 *)src;  // 10 bit in 16
  for (u32 i = 0; i < n; i++) { u32 v = s[i] >> 2; dst[i] = v > 255 ? 255 : (u8)v; }
}

void on_enter(GumInvocationContext *ic) {
  if (!S.on) return;
  u8 **vec = (u8 **)gum_invocation_context_get_nth_argument(ic, 1);
  int head = (int)(gsize)gum_invocation_context_get_nth_argument(ic, 2) & 1;
  if (!vec || !vec[0] || vec[1] <= vec[0]) return;
  long n = (long)(vec[1] - vec[0]) / ITEM;
  if (n > NCAM) n = NCAM;
  s64 mono = clk(1), boot = clk(7), cnt = cnt_ns();
  S.ncall[head]++;
  // both kinds are scanned. The controller frames' exposure isn't short enough to matter: the tracker's auto exposure
  // runs both from about 1 to 15 ms. The 30 Hz frames are locked to a 1.0 base station's 60 Hz cycle, so which
  // exposures catch its sync flash depends on their length, not their kind: on one PICO 4 the head frames from ~160 up
  // saw both stations in every frame, the 52 ones in none
  int use = 1;
  for (long i = 0; i < n; i++) {
    const u8 *it = vec[0] + ITEM * i;
    if (!S.item0_set) { memcpy(S.item0, it, ITEM); S.item0_set = 1; }
    int w = *(const int *)(it + 4), h = *(const int *)(it + 8), fmt = *(const int *)(it + 12);
    int gain = *(const int *)(it + 24);
    s64 ts = *(const s64 *)(it + 40);
    const u8 *px = *(const u8 *const *)(it + 64);
    int bpp = fmt == 32 ? 2 : (fmt == 36 || fmt == 291) ? 1 : 0;
    if (!bpp) { S.badfmt = (u32)fmt; continue; }
    if (!px || w < 16 || h < 16) continue;
    if (w * h > MAXPIX) { S.nsmall++; continue; }
    u32 np = (u32)(w * h);
    struct ev e;
    e.cam = (int)i; e.head = head; e.k = S.k[i]++; e.t = to_mono(ts, mono, boot, cnt); e.mean = smean(px, np, bpp);
    e.slot = -1; e.seq = 0;
    int gain_ok = !S.max_gain || gain <= S.max_gain;
    if (S.scan[i] && !gain_ok) S.ngain++;
    if (use && S.scan[i] && gain_ok) {
      int s = S.next[i]; S.next[i] = (s + 1) % NSLOT;
      u8 *dst = S.ring + ((u64)i * NSLOT + (u64)s) * MAXPIX;
      S.seq[i][s] = 0;
      MB();
      copy8(dst, px, np, bpp);
      S.sw[i][s] = w; S.sh[i][s] = h;
      if (++S.nseq == 0) S.nseq = 1;
      MB();
      S.seq[i][s] = S.nseq;
      e.slot = s; e.seq = S.nseq;
    }
    u32 qh = S.qhead;
    if (qh - S.qtail >= NEV) { S.nqdrop++; continue; }
    S.q[qh % NEV] = e;
    MB();
    S.qhead = qh + 1;
  }
}

// ---------------------------------------------------------------- output ring (JS drains it), as lhsight's
static void put(const char *s, int len) {
  u32 used = S.head - S.tail;
  if (used + (u32)len > OUTCAP) { S.ndrop++; return; }
  for (int j = 0; j < len; j++) S.out[(S.head + (u32)j) & (OUTCAP - 1)] = s[j];
  S.head += (u32)len;
}
int drain(char *dst, int cap) {
  u32 used = S.head - S.tail; int n = (int)(used < (u32)cap ? used : (u32)cap);
  for (int j = 0; j < n; j++) dst[j] = S.out[(S.tail + (u32)j) & (OUTCAP - 1)];
  S.tail += (u32)n;
  return n;
}

// ---------------------------------------------------------------- blobs, as lhsight.c's (img in place of its scratch)
static int find(int x) { while (S.lab[x] != x) { S.lab[x] = S.lab[S.lab[x]]; x = S.lab[x]; } return x; }

// A blob's centre where it is thickest. A 1.0 base station is a blooming sync flash: at higher gains its saturated
// round core merges with a thin lens streak, a reflection beside it and the wall it lights, and the blob's centroid
// lands 3-7 px off (one PICO 4: 1-2 deg). The core is its thickest part, so the centre of the largest disc inside the
// blob (a 3-4 chamfer distance transform over its pixels >= T in its cells) stays on it: there, it fell within 1-3 px
// of where low-gain frames, without the bloom, put the station. False (keep the centroid) for a blob too big to be one
// (a window: CORE_BOX). Saturated blobs only (blobs()' first pass): the dim pass's don't bloom
static int core(const u8 *img, int w, int cw, int T, int root, int x0, int y0, int x1, int y1, int *cx10, int *cy10) {
  int bw = x1 - x0 + 3, bh = y1 - y0 + 3;  // a 1 px frame of outside around the box
  if (bw > CORE_BOX || bh > CORE_BOX) return 0;
  u16 *d = S.dist;
  int last_q = -1, last_in = 0;  // whether a cell is the blob's: looked up once per cell, not per pixel
  for (int yy = 0; yy < bh; yy++)
    for (int xx = 0; xx < bw; xx++) {
      int x = x0 - 1 + xx, y = y0 - 1 + yy, in = 0;
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1 && img[(u32)y * (u32)w + (u32)x] >= T) {
        int q = (y >> 4) * cw + (x >> 4);
        if (q != last_q) { last_q = q; last_in = S.cells[q] && find(q) == root; }
        in = last_in;
      }
      d[yy * bw + xx] = in ? 60000 : 0;
    }
  for (int yy = 1; yy < bh - 1; yy++)
    for (int xx = 1; xx < bw - 1; xx++) {
      u16 *p = d + yy * bw + xx;
      if (!*p) continue;
      int m = *p, v;
      if ((v = p[-1] + 3) < m) m = v;
      if ((v = p[-bw] + 3) < m) m = v;
      if ((v = p[-bw - 1] + 4) < m) m = v;
      if ((v = p[-bw + 1] + 4) < m) m = v;
      *p = (u16)m;
    }
  int best = 0;
  for (int yy = bh - 2; yy >= 1; yy--)
    for (int xx = bw - 2; xx >= 1; xx--) {
      u16 *p = d + yy * bw + xx;
      if (!*p) continue;
      int m = *p, v;
      if ((v = p[1] + 3) < m) m = v;
      if ((v = p[bw] + 3) < m) m = v;
      if ((v = p[bw + 1] + 4) < m) m = v;
      if ((v = p[bw - 1] + 4) < m) m = v;
      *p = (u16)m;
      if (m > best) best = m;
    }
  if (!best) return 0;
  // the centre of the pixels within a pixel's step of the deepest: a plateau is averaged, the result sub-pixel
  u64 sx = 0, sy = 0, n = 0;
  for (int yy = 1; yy < bh - 1; yy++)
    for (int xx = 1; xx < bw - 1; xx++)
      if (d[yy * bw + xx] + 3 > best) { sx += (u64)((x0 - 1 + xx) * 10 + 5); sy += (u64)((y0 - 1 + yy) * 10 + 5); n++; }
  if (!n) return 0;
  *cx10 = (int)(sx / n);
  *cy10 = (int)(sy / n);
  return 1;
}
static int blobs(const u8 *img, int w, int h, int T, int mark, int nbl, u64 *sx, u64 *sy, u32 *np, int *pk) {
  u32 n = (u32)(w * h); int cw = w >> 4, ch = h >> 4, nc = cw * ch, hot = nc <= MAXCELL;
  for (int q = 0; q < nc; q++) S.cells[q] = 0;
  u64 add = 0x0101010101010101ull * (u64)(127 - (T - 1 > 127 ? 127 : T - 1));
  const u64 *q8 = (const u64 *)img; u32 nw = n >> 3;
  for (u32 j = 0; j < nw; j++) {
    u64 x = q8[j];
    if (!(((x + add) | x) & 0x8080808080808080ull)) continue;
    for (int bb = 0; bb < 8; bb++) {
      if ((int)((x >> (8 * bb)) & 0xFF) < T) continue;
      u32 px = (j << 3) + bb; int y = (int)(px / (u32)w), xx = (int)(px - (u32)y * (u32)w);
      if ((y >> 4) >= ch || (xx >> 4) >= cw) continue;
      int q = (y >> 4) * cw + (xx >> 4);
      if (hot && S.hot[q]) continue;
      if (S.cells[q] < 65535) S.cells[q]++;
    }
  }
  for (int q = 0; q < nc; q++) S.lab[q] = q;
  for (int cy = 0; cy < ch; cy++)
    for (int cx = 0; cx < cw; cx++) {
      int q = cy * cw + cx;
      if (!S.cells[q]) continue;
      if (mark && hot)
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++)
            if (cy + dy >= 0 && cy + dy < ch && cx + dx >= 0 && cx + dx < cw) S.hot[q + dy * cw + dx] = 1;
      int nb[4] = { cx > 0 ? q - 1 : -1, cy > 0 ? q - cw : -1, (cy > 0 && cx > 0) ? q - cw - 1 : -1, (cy > 0 && cx < cw - 1) ? q - cw + 1 : -1 };
      for (int e = 0; e < 4; e++) if (nb[e] >= 0 && S.cells[nb[e]]) { int r1 = find(q), r2 = find(nb[e]); if (r1 != r2) S.lab[r1] = r2; }
    }
  int roots[MAXB], b0 = nbl, bx0[MAXB], by0[MAXB], bx1[MAXB], by1[MAXB];
  for (int q = 0; q < nc; q++) {
    if (!S.cells[q]) continue;
    int r = find(q), bi = -1;
    for (int e = b0; e < nbl; e++) if (roots[e] == r) { bi = e; break; }
    if (bi < 0) {
      if (nbl == MAXB) continue;
      bi = nbl++; roots[bi] = r; sx[bi] = sy[bi] = 0; np[bi] = 0; pk[bi] = 0;
      bx0[bi] = by0[bi] = 1 << 30; bx1[bi] = by1[bi] = -1;
    }
    int x0 = (q % cw) * 16, y0 = (q / cw) * 16;
    if (x0 < bx0[bi]) bx0[bi] = x0;
    if (y0 < by0[bi]) by0[bi] = y0;
    if (x0 + 15 > bx1[bi]) bx1[bi] = x0 + 15;
    if (y0 + 15 > by1[bi]) by1[bi] = y0 + 15;
    for (int y = y0; y < y0 + 16; y++) {
      const u8 *row = img + (u32)y * (u32)w;
      for (int x = x0; x < x0 + 16; x++) {
        int v = row[x];
        if (v > pk[bi]) pk[bi] = v;
        if (v < T) continue;
        sx[bi] += (u64)(x * 10 + 5); sy[bi] += (u64)(y * 10 + 5); np[bi]++;
      }
    }
  }
  for (int bi = b0; bi < nbl; bi++) {
    int cx10, cy10;
    if (mark && np[bi] >= CORE_PX && core(img, w, cw, T, roots[bi], bx0[bi], by0[bi], bx1[bi], by1[bi], &cx10, &cy10)) {
      sx[bi] = (u64)cx10 * np[bi];
      sy[bi] = (u64)cy10 * np[bi];
    }
  }
  return nbl;
}

// the queued frames: scanned ones get their bright spots, the others a "-1" line. Returns how many were taken
int scan(void) {
  int got = 0;
  char line[2400]; int L;
  while (S.qtail != S.qhead) {
    MB();
    struct ev e = S.q[S.qtail % NEV];
    S.qtail++;
    got++;
    s64 tus = e.t / 1000;
    if (e.slot < 0) {
      L = snprintf(line, sizeof line, "F %d %u %lld %d -1\n", e.cam, e.k, tus, e.mean);
      put(line, L);
      continue;
    }
    if (S.seq[e.cam][e.slot] != e.seq) { S.nlap++; continue; }  // already written over
    const u8 *img = S.ring + ((u64)e.cam * NSLOT + (u64)e.slot) * MAXPIX;
    int w = S.sw[e.cam][e.slot], h = S.sh[e.cam][e.slot];
    s64 t0 = clk(1);
    u64 sx[MAXB], sy[MAXB]; u32 np[MAXB]; int pk[MAXB];
    int cw = w >> 4, ch = h >> 4, nc = cw * ch, T = e.mean * 4 / 10;
    if (T < S.thr) T = S.thr;
    for (int q = 0; q < nc && q < MAXCELL; q++) S.hot[q] = 0;
    int nbl = blobs(img, w, h, 250, 1, 0, sx, sy, np, pk);
    if (T < 250 && nc <= MAXCELL) nbl = blobs(img, w, h, T, 0, nbl, sx, sy, np, pk);
    MB();
    if (S.seq[e.cam][e.slot] != e.seq) { S.nlap++; continue; }  // written over while it was scanned
    S.scan_ns += (u64)(clk(1) - t0); S.nscan++;
    L = snprintf(line, sizeof line, "F %d %u %lld %d %d", e.cam, e.k, tus, e.mean, nbl);
    for (int b = 0; b < nbl && L < (int)sizeof line - 40; b++)
      L += snprintf(line + L, sizeof line - L, " %llu %llu %u %d", sx[b] / np[b], sy[b] / np[b], np[b], pk[b]);
    line[L++] = '\n';
    put(line, L);
  }
  return got;
}
