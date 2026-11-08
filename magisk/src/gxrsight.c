// lhsight-gxr: the Galaxy XR's camera reader as a process of its own, a QVR camera client as Samsung's controller
// service is one. It attaches the tracking cameras (trackingA/B) and the ctrl-tracking ones (trackingC/D), turns the
// frame pattern on (the short exposures base station sweeps show in come only while a client has it on: otherwise
// only while the controllers are tracked), and runs lhsight.c's scan on every frame where its buffer lies, mapped here.
// Nothing is injected anywhere; when it exits, the camera service drops its clients and the pattern with them.
// Lines on stdout as lhsight.js wrote them:  F ... (frames)  T ... (heartbeat)  I/W/E ...
//   lhsight-gxr idA idB idC idD     the camera ids lhsyncd's calibration gives trackingA..D (-1: not scanned)
//   lhsight-gxr --transform a b ... Google's live calibration, per pair of frames (latest_transforms), then exits
#include <dlfcn.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

// lhsight.c (lhsight_native.c)
void set_bufs(unsigned char *scratch, unsigned short *cells, int *lab, char *out, int thr);
void set_gxr(int on);
void scan_frame(unsigned char *p, int w, int h, int c, unsigned long long t, int is_short);
int drain(char *dst, int cap);
unsigned n_drop(void);
unsigned long long scan_ns(void);
unsigned n_scan(void);

#define W 640
#define H 640
#define SHORT_NS 1000000  // short (base station) frames: 144 us, long ones 2-8 ms
#define NMAP 64           // dmabufs mapped per camera client (its ring has 8-16)

// ---------------------------------------------------------------- the QVR camera client, from its library
// No headers: the C API (qvrcamera_client.h) is inline over getQvrCameraClientInstance's ops table, whose first entry
// makes a client; the rest are QVRCameraClientImpl / QVRCameraDeviceClient methods the library exports.
// qvrcamera_frame_t: u32 frame number @0, u64 start of exposure (BOOTTIME ns) @8, u32 exposure (ns) @16
typedef struct { const char *name, *val; } plugin_param;
typedef struct { int32_t ver, pad; void **ops; } client_instance;
static struct {
  void *(*attach)(void *client, const char *name, plugin_param *p, int n);
  int (*start)(void *dev), (*stop)(void *dev);
  int (*set_param)(void *dev, const char *name, const char *val);
  int (*set_param_num)(void *dev, const char *name, int type, unsigned char n, const char *val);
  int (*current)(void *dev, int *fn);
  int (*get_frame)(void *dev, int *fn, int block, int drop, void *frame);  // block 0: blocking; drop 1: that frame
  int (*to_hw)(void *dev, void *frame, uint32_t *n, void **hwbuf);
  int (*release)(void *dev, int fn);
} q;

static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;  // lhsight.c's state
static volatile sig_atomic_t quit;
static volatile int frames, shorts, errors;

static void say(const char *fmt, ...) {
  char b[512];
  va_list a;
  va_start(a, fmt);
  int n = vsnprintf(b, sizeof b - 1, fmt, a);
  va_end(a);
  if (n < 0) return;
  if (n > (int)sizeof b - 2) n = sizeof b - 2;
  b[n++] = '\n';
  write(1, b, n);  // under PIPE_BUF: whole, among the other lines
}
static long long clk(clockid_t id) { struct timespec t; clock_gettime(id, &t); return t.tv_sec * 1000000000ll + t.tv_nsec; }
static void *sym(void *h, const char *n) {
  void *p = dlsym(h, n);
  if (!p) { say("E lhsight-gxr: %s isn't in libqvrcamera_client.qti.so", n); exit(2); }
  return p;
}

// ---------------------------------------------------------------- one camera client: its frames, scanned
struct cam {
  const char *name;
  int id[2];  // the camera ids of its physical cameras 0 and 1
  void *dev;
  unsigned long long ino[NMAP]; uint8_t *map[NMAP]; long long used[NMAP]; int nmap;
};
// physical camera k's image: hwbuffer entry 0's GraphicBuffer @+16k, its native handle @+0x70, the handle's first fd
// @+12. Mapped once per dmabuf; the least recently used mapping goes when the ring is allocated anew (a wake)
static uint8_t *image(struct cam *c, void *hb, int k) {
  uint8_t *gb = *(uint8_t **)((uint8_t *)hb + 16 * k);
  uint8_t *nh = gb ? *(uint8_t **)(gb + 0x70) : NULL;
  if (!nh) return NULL;
  struct stat st;
  if (fstat(*(int *)(nh + 12), &st) != 0) return NULL;
  long long now = clk(CLOCK_MONOTONIC);
  for (int i = 0; i < c->nmap; i++)
    if (c->ino[i] == (unsigned long long)st.st_ino) { c->used[i] = now; return c->map[i]; }
  void *p = mmap(NULL, W * H, PROT_READ, MAP_SHARED, *(int *)(nh + 12), 0);
  if (p == MAP_FAILED) return NULL;
  int i = c->nmap < NMAP ? c->nmap++ : 0;
  if (c->nmap == NMAP && i == 0) {
    for (int j = 1; j < NMAP; j++) if (c->used[j] < c->used[i]) i = j;
    munmap(c->map[i], W * H);
  }
  c->ino[i] = st.st_ino; c->map[i] = p; c->used[i] = now;
  return p;
}
// A frame held here holds the camera service: its capture loop waits on the buffers its clients haven't given back,
// for every camera (a recording: all of them stalled 300 ms at once, and the streamer died). So the readers run at a
// real-time priority, nothing here waits on lhsyncd while holding one, and the longest hold of each second is reported
static volatile int hold_us;
static void *reader(void *arg) {
  struct cam *c = arg;
  static __thread uint8_t fr[4096];
  struct sched_param sp = {.sched_priority = 2};
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  int fn = -1;
  while (!quit) {
    if (fn < 0 && (q.current(c->dev, &fn) != 0 || fn < 0)) { fn = -1; usleep(50000); continue; }
    if (q.get_frame(c->dev, &fn, 0, 1, fr) != 0) { __sync_fetch_and_add(&errors, 1); fn = -1; continue; }
    long long got = clk(CLOCK_MONOTONIC);
    uint64_t t = *(uint64_t *)(fr + 8);
    uint32_t ex = *(uint32_t *)(fr + 16);
    int shrt = ex < SHORT_NS;
    __sync_fetch_and_add(&frames, 1);
    if (shrt) __sync_fetch_and_add(&shorts, 1);
    uint32_t n = 0;
    void *hb = NULL;
    if (q.to_hw(c->dev, fr, &n, &hb) == 0 && hb) {
      long long mono = (long long)(t + ex / 2) - (clk(CLOCK_BOOTTIME) - clk(CLOCK_MONOTONIC));  // mid-exposure
      for (int k = 0; k < 2; k++) {
        uint8_t *img = c->id[k] >= 0 ? image(c, hb, k) : NULL;
        if (!img) continue;
        pthread_mutex_lock(&mu);
        scan_frame(img, W, H, c->id[k], (unsigned long long)mono, shrt);
        pthread_mutex_unlock(&mu);
      }
    }
    q.release(c->dev, fn);
    int held = (int)((clk(CLOCK_MONOTONIC) - got) / 1000);
    if (held > hold_us) hold_us = held;
    fn++;
  }
  return NULL;
}

static void on_term(int s) { (void)s; quit = 1; }
static int pattern(void *dev, int on) {
  unsigned char v = on ? 1 : 0;
  return q.set_param_num(dev, "qvrcamdevice_enable_frame_pattern", 0, 1, (const char *)&v);  // uint8
}

// ---------------------------------------------------------------- --transform: Google's live calibration
// A line per pair of frames: "a b rx ry rz tx ty tz", a_t_b as a Rodrigues rotation (rad) and metres, or "a b E why".
// The client's ABI on firmware I610UEU2AZF3, as stearnsbq's questlhsync-galaxy-xr found it (src/head_origin.c, MIT):
// interface 0xc0001, its read-only getLatestTransform at byte 0xa0, six float32s out. A field of canaries rejects any
// other layout, and lhsyncd asks only on this firmware. Each line is out before the next query: a frame the client
// doesn't survive costs only the lines after it.
#define CAL_IFACE 0xc0001
#define CAL_CANARY 0x7fc00001u
static int latest_transforms(int n, char **pairs) {
  alarm(8);
  void *lib = dlopen("/vendor/lib64/libperception_calibration_client_plugin.so", RTLD_NOW);
  if (!lib) { say("E %s", dlerror()); return 2; }
  int (*create)(int, const char *, void **) = dlsym(lib, "GoogleClient_CreateInstance");
  void (*destroy)(void *) = dlsym(lib, "GoogleClient_DestroyInstance");
  void (*activate)(void *) = dlsym(lib, "PerceptionClient_Activated");
  void (*deactivate)(void *) = dlsym(lib, "PerceptionClient_Deactivated");
  if (!create || !destroy || !activate || !deactivate) { say("E the calibration client lacks a function"); return 2; }
  void *client = NULL;
  int r = create(CAL_IFACE, "QuestLHSync", &client);
  if (r || !client) { say("E no calibration client (%d)", r); return 2; }
  if (*(uint32_t *)client != CAL_IFACE) { say("E the calibration client's interface differs"); destroy(client); return 2; }
  activate(client);
  int (*get)(void *, const char *, const char *, void *) =
      *(int (**)(void *, const char *, const char *, void *))((uint8_t *)client + 0xa0);
  for (int p = 0; p + 1 < n; p += 2) {
    union { float v[1024]; uint32_t w[1024]; } out;
    for (int i = 0; i < 1024; i++) out.w[i] = CAL_CANARY;
    r = get ? get(client, pairs[p], pairs[p + 1], &out) : -1;
    int ok = !r;
    for (int i = 0; ok && i < 6; i++) ok = __builtin_isfinite(out.v[i]) && __builtin_fabsf(out.v[i]) <= (i < 3 ? 3.15f : .5f);
    for (int i = 6; ok && i < 1024; i++) ok = out.w[i] == CAL_CANARY;
    if (ok) say("%s %s %.9g %.9g %.9g %.9g %.9g %.9g", pairs[p], pairs[p + 1], out.v[0], out.v[1], out.v[2], out.v[3], out.v[4], out.v[5]);
    else say("%s %s E %d", pairs[p], pairs[p + 1], r);
  }
  deactivate(client);
  destroy(client);
  return 0;
}

int main(int argc, char **argv) {
  if (argc >= 4 && !strcmp(argv[1], "--transform")) return latest_transforms(argc - 2, argv + 2);
  struct cam cams[2] = {{"tracking", {0, 1}}, {"ctrl-tracking", {2, 3}}};
  for (int i = 0; i < 4 && i + 1 < argc; i++) cams[i / 2].id[i % 2] = atoi(argv[i + 1]);
  signal(SIGTERM, on_term);
  signal(SIGINT, on_term);
  signal(SIGPIPE, SIG_IGN);

  void *h = dlopen("/vendor/lib64/libqvrcamera_client.qti.so", RTLD_NOW);
  if (!h) { say("E lhsight-gxr: %s", dlerror()); return 2; }
  client_instance *(*instance)(void) = sym(h, "getQvrCameraClientInstance");
  q.attach = sym(h, "_ZN19QVRCameraClientImpl22AttachCameraWithParamsEPKcP17_qvr_plugin_parami");
  q.start = sym(h, "_ZN21QVRCameraDeviceClient5StartEv");
  q.stop = sym(h, "_ZN21QVRCameraDeviceClient4StopEv");
  q.set_param = sym(h, "_ZN21QVRCameraDeviceClient8SetParamEPKcS1_");
  q.set_param_num = sym(h, "_ZN21QVRCameraDeviceClient11SetParamNumEPKc24QVRCAMERA_PARAM_NUM_TYPEhS1_");
  q.current = sym(h, "_ZN21QVRCameraDeviceClient21GetCurrentFrameNumberEPi");
  q.get_frame = sym(h, "_ZN21QVRCameraDeviceClient8GetFrameEPi20QVRCAMERA_BLOCK_MODE19QVRCAMERA_DROP_MODEP17qvrcamera_frame_t");
  q.to_hw = sym(h, "_ZN21QVRCameraDeviceClient21FrameToHardwareBufferEP17qvrcamera_frame_tPjPP21_qvrcamera_hwbuffer_t");
  q.release = sym(h, "_ZN21QVRCameraDeviceClient12ReleaseFrameEi");
  client_instance *in = instance();
  void *client = in && in->ops ? ((void *(*)(void))in->ops[0])() : NULL;
  if (!client) { say("E lhsight-gxr: no connection to the QVR camera service"); return 2; }

  // the camera service merges no frames here ("frame merge is force off"): a client must ask for none, as the
  // controller service does
  plugin_param pp = {"qvrcamdevice_get_frame_img_disable", "true"};
  for (int i = 0; i < 2; i++) {
    struct cam *c = &cams[i];
    c->dev = q.attach(client, c->name, &pp, 1);
    if (!c->dev) { say("E lhsight-gxr: attaching %s failed", c->name); return 2; }
    int r = q.start(c->dev);
    if (r) { say("E lhsight-gxr: starting %s failed (%d)", c->name, r); return 2; }
    q.set_param(c->dev, "qvrcamdevice_get_frame_img_disable", "true");
  }
  void *ctrl = cams[1].dev;
  int pr = pattern(ctrl, 1);

  static unsigned char scratch[1];  // the Galaxy XR's scan reads the frame where it lies (set_gxr)
  static unsigned short cells[(W >> 4) * (H >> 4)];
  static int lab[(W >> 4) * (H >> 4)];
  static char out[1 << 20];
  set_bufs(scratch, cells, lab, out, 64);
  set_gxr(1);
  say("I lhsight-gxr: QVR camera client, tracking (A %d, B %d) and ctrl-tracking (C %d, D %d), frame pattern on (%d)",
      cams[0].id[0], cams[0].id[1], cams[1].id[0], cams[1].id[1], pr);
  pthread_t th[2];
  for (int i = 0; i < 2; i++) pthread_create(&th[i], NULL, reader, &cams[i]);

  // lines out each 100 ms, never waiting: taken out under the lock, written without it, and dropped when lhsyncd's pipe
  // is full (it waits on a slow network itself)
  fcntl(1, F_SETFL, fcntl(1, F_GETFL) | O_NONBLOCK);
  static char buf[1 << 16];
  long long beat = clk(CLOCK_MONOTONIC), set_at = beat;
  int pf = 0, ps = 0, quiet = 0, lost = 0;
  while (!quit) {
    usleep(100000);
    for (;;) {
      pthread_mutex_lock(&mu);
      int n = drain(buf, sizeof buf);
      pthread_mutex_unlock(&mu);
      if (n <= 0) break;
      if (write(1, buf, n) != n) lost++;
    }
    long long now = clk(CLOCK_MONOTONIC);
    if (now - beat < 1000000000ll) continue;
    beat = now;
    int f = frames, s = shorts;
    // frames without short ones for 3 s (the cameras woke, and the pattern went with the old streams): on again,
    // at most every 10 s
    quiet = f != pf && s == ps ? quiet + 1 : 0;
    if (quiet >= 3 && now - set_at > 10000000000ll) {
      set_at = now;
      say("I no short frames for %d s: frame pattern on again (%d)", quiet, pattern(ctrl, 1));
    }
    pf = f; ps = s;
    pthread_mutex_lock(&mu);
    unsigned ns = n_scan();
    unsigned long long sn = scan_ns();
    unsigned nd = n_drop();
    pthread_mutex_unlock(&mu);
    int held = hold_us;
    hold_us = 0;
    say("T %lld scans %u scan_us %.0f drops %u faults 0 frames %d short %d errors %d hold_max_us %d lost_writes %d",
        now / 1000, ns, sn / 1e3 / (ns ? ns : 1), nd, f, s, errors, held, lost);
  }
  pattern(ctrl, 0);
  for (int i = 0; i < 2; i++) q.stop(cams[i].dev);
  _exit(0);  // the readers may sit in a blocking GetFrame
}
