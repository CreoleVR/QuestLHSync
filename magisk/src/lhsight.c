// lhsight: streaming bright-spot detector for the four tracking cameras, run as a Frida CModule
// inside the sensors HAL. Read-only on the dmabuf rings. For every new frame: a sampled mean
// (exposure class), and on short-exposure frames the saturated blobs (centroid, size, peak).
// Output is text lines in a ring the JS side drains:
//   F cam k t_us mean nblobs [x y npx peak]...      (x,y in pixels*10)
typedef unsigned char u8; typedef unsigned short u16; typedef unsigned int u32; typedef unsigned long long u64;
struct ts { long s; long ns; };
extern int clock_gettime(int, struct ts *);
extern void *memcpy(void *, const void *, unsigned long);
extern int snprintf(char *, unsigned long, const char *, ...);
#define MAXS 64
#define MAXB 24
#define OUTCAP (1 << 20)
// A camera's next frame comes 26.7 ms after its last (37.5 fps): polling sleeps until shortly before it is due.
// Reading the camera buffers is slow, so checking all of them every millisecond cost far more than the scans.
#define QUIET_NS 22000000ull
struct st {
  u8 *slot[MAXS]; int sw[MAXS], sh[MAXS], cam[MAXS], pend[MAXS]; u32 shash[MAXS]; int nslots;
  u32 k[8]; int m1[8], m2[8];                  // per-camera frame counter, previous two means
  int thr;
  u8 *scratch; u16 *cells; int *lab;
  char *out; u32 head, tail, ndrop;
  u64 scan_ns; u32 nscan;
  u64 last_t[8]; u32 npoll;                    // per camera: when its last frame was found
};
extern struct st S;
static u64 now(void) { struct ts t; clock_gettime(1, &t); return (u64)t.s * 1000000000ull + (u64)t.ns; }
void set_bufs(u8 *scratch, u16 *cells, int *lab, char *out, int thr) {
  S.scratch = scratch; S.cells = cells; S.lab = lab; S.out = out; S.head = S.tail = 0; S.thr = thr;
}
void reset_slots(void) { S.nslots = 0; }
void add_slot(u8 *p, int w, int h, int cam) {
  int i = S.nslots++; S.slot[i] = p; S.sw[i] = w; S.sh[i] = h; S.cam[i] = cam; S.pend[i] = 0;
}
u32 n_drop(void) { return S.ndrop; }
u64 scan_ns(void) { return S.scan_ns; }
u32 n_scan(void) { return S.nscan; }
u32 n_poll(void) { return S.npoll; }
static u32 hash(int i) {
  const u8 *p = S.slot[i]; u32 n = (u32)(S.sw[i] * S.sh[i]), step = n / 61, h = 2166136261u;
  for (u32 o = step / 2; o < n; o += step) { h ^= p[o]; h *= 16777619u; }
  return h;
}
void prime(void) { for (int i = 0; i < S.nslots; i++) S.shash[i] = hash(i); }
static int smean(const u8 *p, u32 n) {
  u32 step = n / 4096, s = 0, c = 0;
  for (u32 o = step / 2; o < n; o += step) { s += p[o]; c++; }
  return (int)(s * 10 / c);                  // mean * 10
}
static void put(const char *s, int len) {
  u32 used = S.head - S.tail;
  if (used + (u32)len > OUTCAP) { S.ndrop++; return; }
  for (int j = 0; j < len; j++) S.out[(S.head + (u32)j) & (OUTCAP - 1)] = s[j];
  S.head += (u32)len;
}
// JS drains: copy up to cap bytes out, returns count
int drain(char *dst, int cap) {
  u32 used = S.head - S.tail; int n = (int)(used < (u32)cap ? used : (u32)cap);
  for (int j = 0; j < n; j++) dst[j] = S.out[(S.tail + (u32)j) & (OUTCAP - 1)];
  S.tail += (u32)n;
  return n;
}
static int find(int x) { while (S.lab[x] != x) { S.lab[x] = S.lab[S.lab[x]]; x = S.lab[x]; } return x; }
static void frame(int i, u64 t) {
  int w = S.sw[i], h = S.sh[i], c = S.cam[i];
  S.last_t[c] = t;
  u32 n = (u32)(w * h);
  int mean = smean(S.slot[i], n);
  u32 k = S.k[c]++;
  int a = S.m1[c], b = S.m2[c];
  S.m2[c] = a; S.m1[c] = mean;
  int lo = a < b ? a : b;
  int is_short = (k >= 2) && mean * 10 < lo * 6;  // short exposure: well under both previous frames
  char line[1200]; int L;
  if (!is_short) {
    L = snprintf(line, sizeof line, "F %d %u %llu %d -1\n", c, k, t / 1000, mean);
    put(line, L);
    return;
  }
  u64 t0 = now();
  memcpy(S.scratch, S.slot[i], n);
  int cw = w >> 4, ch = h >> 4, nc = cw * ch, T = S.thr;
  for (int q = 0; q < nc; q++) S.cells[q] = 0;
  u64 M = 0xF8F8F8F8F8F8F8F8ull;              // fast reject: no byte >= 248 in this word
  const u64 *q8 = (const u64 *)S.scratch; u32 nw = n >> 3;
  for (u32 j = 0; j < nw; j++) {
    u64 x = q8[j], v = ~x & M;
    if (!((v - 0x0101010101010101ull) & ~v & 0x8080808080808080ull)) continue;
    for (int bb = 0; bb < 8; bb++) {
      if ((int)((x >> (8 * bb)) & 0xFF) < T) continue;
      u32 px = (j << 3) + bb; int y = (int)(px / (u32)w), xx = (int)(px - (u32)y * (u32)w);
      u16 *cc = &S.cells[(y >> 4) * cw + (xx >> 4)];
      if (*cc < 65535) (*cc)++;
    }
  }
  // union 8-connected bright cells into blobs
  for (int q = 0; q < nc; q++) S.lab[q] = q;
  for (int cy = 0; cy < ch; cy++)
    for (int cx = 0; cx < cw; cx++) {
      int q = cy * cw + cx;
      if (!S.cells[q]) continue;
      int nb[4] = { cx > 0 ? q - 1 : -1, cy > 0 ? q - cw : -1, (cy > 0 && cx > 0) ? q - cw - 1 : -1, (cy > 0 && cx < cw - 1) ? q - cw + 1 : -1 };
      for (int e = 0; e < 4; e++) if (nb[e] >= 0 && S.cells[nb[e]]) { int r1 = find(q), r2 = find(nb[e]); if (r1 != r2) S.lab[r1] = r2; }
    }
  // per blob: centroid of pixels >= T inside its cells
  int roots[MAXB], nbl = 0; u64 sx[MAXB], sy[MAXB]; u32 np[MAXB]; int pk[MAXB];
  for (int q = 0; q < nc; q++) {
    if (!S.cells[q]) continue;
    int r = find(q), bi = -1;
    for (int e = 0; e < nbl; e++) if (roots[e] == r) { bi = e; break; }
    if (bi < 0) { if (nbl == MAXB) continue; bi = nbl++; roots[bi] = r; sx[bi] = sy[bi] = 0; np[bi] = 0; pk[bi] = 0; }
    int x0 = (q % cw) * 16, y0 = (q / cw) * 16;
    for (int y = y0; y < y0 + 16; y++) {
      const u8 *row = S.scratch + (u32)y * (u32)w;
      for (int x = x0; x < x0 + 16; x++) {
        int v = row[x];
        if (v > pk[bi]) pk[bi] = v;
        if (v < T) continue;
        sx[bi] += (u64)(x * 10 + 5); sy[bi] += (u64)(y * 10 + 5); np[bi]++;
      }
    }
  }
  S.scan_ns += now() - t0; S.nscan++;
  L = snprintf(line, sizeof line, "F %d %u %llu %d %d", c, k, t / 1000, mean, nbl);
  for (int e = 0; e < nbl && L < (int)sizeof line - 40; e++)
    L += snprintf(line + L, sizeof line - L, " %llu %llu %u %d", sx[e] / np[e], sy[e] / np[e], np[e], pk[e]);
  line[L++] = '\n';
  put(line, L);
}
int poll(void) {
  int got = 0;
  S.npoll++;
  for (int i = 0; i < S.nslots; i++) {
    u32 hh = hash(i);
    if (hh != S.shash[i]) { S.shash[i] = hh; S.pend[i] = 1; continue; }
    if (S.pend[i]) { S.pend[i] = 0; frame(i, now()); got++; }
  }
  return got;
}
// how long the next poll can wait (us): 0 while a frame is pending or a camera is due, else until the first one is
int idle_us(void) {
  u64 t = now(), due = ~0ull;
  for (int i = 0; i < S.nslots; i++) if (S.pend[i]) return 0;
  for (int c = 0; c < 8; c++) if (S.last_t[c] && S.last_t[c] + QUIET_NS < due) due = S.last_t[c] + QUIET_NS;
  if (due == ~0ull || due <= t) return 0;
  u64 us = (due - t) / 1000;
  return us > 22000 ? 22000 : (int)us;
}
