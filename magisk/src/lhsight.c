// lhsight: streaming bright-spot detector for the tracking cameras, run as a Frida CModule
// inside the sensors HAL. Read-only on the dmabuf rings. For every new frame: a sampled mean
// (exposure class), and on short-exposure frames the saturated blobs, then dimmer ones (centroid, size, peak).
// Output is text lines in a ring the JS side drains:
//   F cam k t_us mean nblobs [x y npx peak]...      (x,y in pixels*10)
typedef unsigned char u8; typedef unsigned short u16; typedef unsigned int u32; typedef unsigned long long u64;
struct ts { long s; long ns; };
extern int clock_gettime(int, struct ts *);
extern void *memcpy(void *, const void *, unsigned long);
extern int snprintf(char *, unsigned long, const char *, ...);
#define MAXS 128
#define MAXC 16                                // camera ids 0..15
#define MAXB 48
#define MAXCELL 2048                           // 16x16 cells a frame may have for the dim pass
#define OUTCAP (1 << 20)
// Polling sleeps until shortly before a camera's next frame is due. Reading the camera buffers is slow, so checking
// all of them every millisecond cost far more than the scans. Frames don't come evenly: the Quest Pro's come 20, 21
// and 39 ms apart (long-short, short-long, long-long), so the next one is predicted from the frames a cycle before.
#define EARLY_US 3000                          // wake this long before the predicted frame
#define HIST 32                                // frame times kept per camera
struct st {
  u8 *slot[MAXS]; int sw[MAXS], sh[MAXS], cam[MAXS], pend[MAXS]; u32 shash[MAXS]; int nslots;
  u32 k[MAXC]; int m1[MAXC], m2[MAXC];         // per camera: frame counter, previous two means
  int nostrict[MAXC], nalt[MAXC];              // frames since the last strict short one, alternating ones since
  int thr;                                     // the dim pass's floor
  u8 hot[MAXCELL];                             // cells the dim pass leaves to the saturated one
  u8 *scratch; u16 *cells; int *lab;
  char *out; u32 head, tail, ndrop;
  u64 scan_ns; u32 nscan;
  u64 last_t[MAXC], due[MAXC]; u32 npoll;      // per camera: when its last frame was found, when to look for the next
  u64 ft[MAXC][HIST]; u32 nft[MAXC];           // its last frame times (ns)
  int adj[MAXC], late[MAXC], slept;            // how much earlier to wake for it (us), its frame waited for us
  int cyes[MAXC], cno[MAXC];                   // clearly darker frames the cycle took for short ones, and didn't
  int ph[MAXC], phn[MAXC];                     // the short frame's k % 3, and its net votes (frame())
};
// the place: trusted from PH_LOCK net votes, held to PH_MAX, PH_MISS off per vote elsewhere; lost after a PH_GAP (ns)
#define PH_LOCK 10
#define PH_MAX 60
#define PH_MISS 5
#define PH_GAP 75000000ull
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
static int rel_us(int c, u32 i, u64 t) { return (int)((long long)(S.ft[c][i % HIST] - t) / 1000); }  // frame i, us from t
// when camera c's next frame is due, now that one was found at t. The cycle: the 1-4 frame shift the recent gaps
// repeat best at. The cycle's length: the median of the last 12 frame-to-same-frame spans. The next frame: one cycle
// after each of the last 3 cycles' frames at its place, the earliest of those (a frame found late doesn't push the
// prediction late)
static void predict(int c, u64 t) {
  if (S.nft[c] && t - S.ft[c][(S.nft[c] - 1) % HIST] >= 200000000ull) S.nft[c] = 0;  // the camera paused: start over
  u32 n = S.nft[c]++, k = n;
  S.ft[c][n % HIST] = t;
  S.due[c] = 0;
  if (n < 24) return;
  u32 best = 1; int be = 0x7fffffff;
  for (u32 sh = 1; sh <= 4; sh++) {
    int e = 0;
    for (u32 i = k - 5; i <= k; i++) {
      int g1 = rel_us(c, i, t) - rel_us(c, i - 1, t), g2 = rel_us(c, i - sh, t) - rel_us(c, i - sh - 1, t);
      e += g1 > g2 ? g1 - g2 : g2 - g1;
    }
    if (e < be) { be = e; best = sh; }
  }
  int v[12];
  for (u32 j = 0; j < 12; j++) {
    int x = rel_us(c, k - j, t) - rel_us(c, k - j - best, t), b = (int)j;
    for (; b > 0 && v[b - 1] > x; b--) v[b] = v[b - 1];
    v[b] = x;
  }
  int C = (v[5] + v[6]) / 2, lo = 0x7fffffff;
  for (u32 j = 1; j <= 3; j++) {
    int p = rel_us(c, k + 1 - best * j, t) + (int)j * C;  // us after this frame
    if (p < lo) lo = p;
  }
  if (lo > 0) S.due[c] = t + (u64)lo * 1000ull;
  // found times lag the frames, so a prediction made from them can stay late: a frame that was already there when
  // polling woke up wakes it a millisecond earlier for this camera next time, and that eases off while frames are
  // caught arriving
  if (S.late[c]) { S.late[c] = 0; if (S.adj[c] < 8000) S.adj[c] += 1000; }
  else if (S.adj[c] >= 250) S.adj[c] -= 250;
}
// The Quest Pro's cameras take 3 frames on a 4-slot grid, long, short, long, and leave the 4th slot out (gaps ~20, 21,
// 39 ms at 37.5 frames/s, 17, 17, 33 at 45): the short frame is the one between the two short gaps. Its place tells it
// in any light, where its brightness doesn't: in a lit room the long frames' exposure shortens until they are hardly
// brighter than the short one, or darker. From the last 10 cycles' gaps (found times jitter by several ms): 1 for the
// short frame, 0 for a long one, -1 without such a cycle (yet)
#define NCYC 10
static int cycle_short(int c) {
  u32 n = S.nft[c];                            // this frame is n - 1
  if (n < 3 * NCYC + 2) return -1;
  int g[3] = { 0, 0, 0 };                      // the gaps before: this frame's place, the one before, the one before that
  for (u32 j = 0; j < 3 * NCYC; j++) g[j % 3] += rel_us(c, n - 1 - j, S.ft[c][(n - 2 - j) % HIST]);
  int lo = 0;
  for (int j = 1; j < 3; j++) if (g[j] > g[lo]) lo = j;
  int a = g[(lo + 1) % 3], b = g[(lo + 2) % 3];
  if (g[lo] * 10 < (a + b) * 7 || g[lo] * 10 > (a + b) * 14 || (a < b ? a * 10 < b * 6 : b * 10 < a * 6)) return -1;
  return lo == 1;                              // the long gap was before the frame before
}
// The blobs of pixels >= T in scratch (w x h): 16x16 cells holding one, 8-connected, centroid of those pixels; added
// to the arrays from nbl on, up to MAXB. Cells marked hot are left out, and the first pass marks its cells and
// their neighbours (mark), so a later, dimmer pass doesn't take in a saturated blob's glow.
static int blobs(int w, int h, int T, int mark, int nbl, u64 *sx, u64 *sy, u32 *np, int *pk) {
  u32 n = (u32)(w * h); int cw = w >> 4, ch = h >> 4, nc = cw * ch, hot = nc <= MAXCELL;
  for (int q = 0; q < nc; q++) S.cells[q] = 0;
  // fast reject: no byte > T - 1 in this word (at most 127: brighter thresholds only reject less)
  u64 add = 0x0101010101010101ull * (u64)(127 - (T - 1 > 127 ? 127 : T - 1));
  const u64 *q8 = (const u64 *)S.scratch; u32 nw = n >> 3;
  for (u32 j = 0; j < nw; j++) {
    u64 x = q8[j];
    if (!(((x + add) | x) & 0x8080808080808080ull)) continue;
    for (int bb = 0; bb < 8; bb++) {
      if ((int)((x >> (8 * bb)) & 0xFF) < T) continue;
      u32 px = (j << 3) + bb; int y = (int)(px / (u32)w), xx = (int)(px - (u32)y * (u32)w);
      if ((y >> 4) >= ch || (xx >> 4) >= cw) continue;  // the edge strip of a size that isn't a multiple of 16
      int q = (y >> 4) * cw + (xx >> 4);
      if (hot && S.hot[q]) continue;
      if (S.cells[q] < 65535) S.cells[q]++;
    }
  }
  // union 8-connected bright cells into blobs
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
  // per blob: centroid of pixels >= T inside its cells
  int roots[MAXB], b0 = nbl;
  for (int q = 0; q < nc; q++) {
    if (!S.cells[q]) continue;
    int r = find(q), bi = -1;
    for (int e = b0; e < nbl; e++) if (roots[e] == r) { bi = e; break; }
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
  return nbl;
}
static void frame(int i, u64 t) {
  int w = S.sw[i], h = S.sh[i], c = S.cam[i];
  // a gap this long (frames come at most 39 ms apart) could have let a frame go by unseen: the count's place is lost
  if (S.last_t[c] && t - S.last_t[c] > PH_GAP) S.phn[c] = 0;
  predict(c, t);
  S.last_t[c] = t;
  u32 n = (u32)(w * h);
  int mean = smean(S.slot[i], n);
  u32 k = S.k[c]++;
  int a = S.m1[c], b = S.m2[c];
  S.m2[c] = a; S.m1[c] = mean;
  int lo = a < b ? a : b;
  // short exposure: well under both previous frames (long-short-long, the Quest Pro), or at the short frame's place in
  // the cycle (cycle_short), which a lit room doesn't hide. The place is trusted while the clearly darker frames agree
  // with it (a headset with another cycle turns it off). Without a cycle: a camera alternating long and short frames
  // never passes the first test (one of the two is short too): after 60 frames without a short one while every fourth
  // or more is well under the frame before, it is judged against the frame before alone
  int strict = k >= 2 && mean * 10 < lo * 6, alt = k >= 1 && mean * 10 < a * 6;
  if (strict) S.nostrict[c] = S.nalt[c] = 0;
  else if (S.nostrict[c] < 100000) { S.nostrict[c]++; S.nalt[c] += alt; }
  int cyc = cycle_short(c);
  if (strict && cyc >= 0 && S.cyes[c] + S.cno[c] < 1000000) { if (cyc) S.cyes[c]++; else S.cno[c]++; }
  if (S.cno[c] * 4 > S.cyes[c] + 4) cyc = -1;
  // the short frame's place in the frame count: the short frames the two tests find vote for their k % 3. A camera
  // whose frames are found late, two at a time, hides the cycle in its found times, and a lit room the darker frame
  // (a recording: the right camera searched 8% of its short frames, the left 94%); the count keeps the place. One
  // that wanders (another cycle) never gathers PH_LOCK votes
  if (strict || cyc == 1) {
    int p = (int)(k % 3);
    if (S.phn[c] > 0 && S.ph[c] != p) S.phn[c] -= PH_MISS;
    else { S.ph[c] = p; if (S.phn[c] < PH_MAX) S.phn[c]++; }
    if (S.phn[c] <= 0) { S.ph[c] = p; S.phn[c] = 1; }
  }
  int place = S.phn[c] >= PH_LOCK ? (int)(k % 3) == S.ph[c] : -1;
  int is_short = strict || (cyc >= 0 ? cyc : place >= 0 ? place : alt && S.nostrict[c] >= 60 && S.nalt[c] * 4 >= S.nostrict[c]);
  char line[2400]; int L;
  if (!is_short) {
    L = snprintf(line, sizeof line, "F %d %u %llu %d -1\n", c, k, t / 1000, mean);
    put(line, L);
    return;
  }
  u64 t0 = now();
  memcpy(S.scratch, S.slot[i], n);
  // saturated blobs (base station dots, lamps), then dimmer ones (>= 4x the frame's mean, at least S.thr) in the cells
  // the saturated ones leave alone: a dot that doesn't saturate, in a room light enough to shorten the exposure
  u64 sx[MAXB], sy[MAXB]; u32 np[MAXB]; int pk[MAXB];
  int cw = w >> 4, ch = h >> 4, nc = cw * ch, T = mean * 4 / 10;
  if (T < S.thr) T = S.thr;
  for (int q = 0; q < nc && q < MAXCELL; q++) S.hot[q] = 0;
  int nbl = blobs(w, h, 250, 1, 0, sx, sy, np, pk);
  if (T < 250 && nc <= MAXCELL) nbl = blobs(w, h, T, 0, nbl, sx, sy, np, pk);
  S.scan_ns += now() - t0; S.nscan++;
  L = snprintf(line, sizeof line, "F %d %u %llu %d %d", c, k, t / 1000, mean, nbl);
  for (int e = 0; e < nbl && L < (int)sizeof line - 40; e++)
    L += snprintf(line + L, sizeof line - L, " %llu %llu %u %d", sx[e] / np[e], sy[e] / np[e], np[e], pk[e]);
  line[L++] = '\n';
  put(line, L);
}
int poll(void) {
  int got = 0, first = S.slept;  // the first poll after a sleep until a predicted frame
  S.npoll++;
  S.slept = 0;
  for (int i = 0; i < S.nslots; i++) {
    u32 hh = hash(i);
    if (hh != S.shash[i]) { S.shash[i] = hh; S.pend[i] = 1; if (first) S.late[S.cam[i]] = 1; continue; }
    if (S.pend[i]) { S.pend[i] = 0; frame(i, now()); got++; }
  }
  return got;
}
// how long the next poll can wait (us): 0 while a frame is pending or a camera is due, else until the first one is
int idle_us(void) {
  u64 t = now(), due = ~0ull;
  for (int i = 0; i < S.nslots; i++) if (S.pend[i]) return 0;
  for (int c = 0; c < MAXC; c++) {
    if (!S.last_t[c]) continue;
    if (!S.due[c]) return 0;  // no prediction yet: poll every millisecond
    u64 d = S.due[c] - (u64)(EARLY_US + S.adj[c]) * 1000ull;
    if (d < due) due = d;
  }
  if (due == ~0ull || due <= t) return 0;
  u64 us = (due - t) / 1000;
  if (us > 50000) us = 50000;
  S.slept = us > 1500;
  return (int)us;
}
