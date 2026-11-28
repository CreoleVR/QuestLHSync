// QuestLHSync: keeps the lighthouse universe aligned to a headset's own tracking, a rooted Quest's or a Steam Frame's,
// whatever streams it to SteamVR. The headset's lhsyncd (Magisk module / Frame package) serves base station laser
// flashes seen by the tracking cameras; this driver solves the 4-DOF lighthouse -> headset transform (sync.cpp) and
// applies it to every lighthouse-tracked device (trackers, controllers, base stations).
//
// Hooks IVRServerDriverHost::TrackedDevicePoseUpdated (MinHook on the function, so every driver's calls go
// through it): lighthouse devices get the transform prepended to their WorldFromDriver, and the HMD's own poses
// (any streamer's) feed the solver with their exact times. Status and commands for the dashboard overlay
// (QuestLHSync.exe, launched from here) go through the shared memory in qlhs_status.h.
#include <winsock2.h>
#include <windows.h>
#include <shlobj.h>
#include <timeapi.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "../common/qlhs_status.h"
#include "MinHook.h"
#include "gravity.h"
#include "lighthouse_input.h"
#include "net.h"
#include "openvr_driver.h"
#include "relations.h"
#include "sync.h"

static const char *kSection = "driver_questlhsync";

// ---------------------------------------------------------------- logging
static std::mutex g_log_m;
static std::mutex g_st_m;
static FILE *g_logf;
static QlhsStatus *g_st;
static std::unique_ptr<Sync> g_sync;
static std::unique_ptr<Gravity> g_gravity;
static std::unique_ptr<LighthouseInput> g_lh;
static std::unique_ptr<Relations> g_rel;

static void PushStatusLog(const std::string &s);

static void Log(const std::string &s) {
  if (vr::VRDriverLog()) vr::VRDriverLog()->Log(("questlhsync: " + s).c_str());
  std::lock_guard<std::mutex> g(g_log_m);
  if (g_logf) {
    time_t t = time(nullptr);
    struct tm tm;
    localtime_s(&tm, &t);
    char b[32];
    strftime(b, sizeof b, "%Y-%m-%d %H:%M:%S", &tm);
    fprintf(g_logf, "%s %s\n", b, s.c_str());
    fflush(g_logf);
  }
  PushStatusLog(s);
}

static std::string Fmt(const char *fmt, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  return buf;
}

static std::string DataDir() {
  PWSTR p = nullptr;
  std::filesystem::path d;
  if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, nullptr, &p))) d = std::filesystem::path(p) / "QuestLHSync";
  CoTaskMemFree(p);
  std::error_code ec;
  std::filesystem::create_directories(d, ec);
  return d.string();
}

// ---------------------------------------------------------------- the transform the hook applies
struct Xf { int active; Quat q; double t[3]; };
static std::atomic<uint32_t> g_xf_seq{0};
static Xf g_xf_buf{};

static void WriteXf(const Transform &x) {
  g_xf_seq.fetch_add(1, std::memory_order_acq_rel);  // odd: writing
  std::atomic_thread_fence(std::memory_order_release);
  g_xf_buf.active = x.active;
  g_xf_buf.q = x.q;
  g_xf_buf.t[0] = x.t.x; g_xf_buf.t[1] = x.t.y; g_xf_buf.t[2] = x.t.z;
  std::atomic_thread_fence(std::memory_order_release);
  g_xf_seq.fetch_add(1, std::memory_order_acq_rel);
}

static bool ReadXf(Xf &x) {
  static thread_local Xf last{};
  for (int i = 0; i < 64; i++) {
    uint32_t s = g_xf_seq.load(std::memory_order_acquire);
    if (s & 1) { YieldProcessor(); continue; }
    Xf c = g_xf_buf;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (g_xf_seq.load(std::memory_order_acquire) == s) { last = c; break; }
  }
  x = last;
  return x.active != 0;
}

// ---------------------------------------------------------------- devices
enum Kind { kUnknown = 0, kLighthouse = 1, kOther = 2, kHmd = 3 };
static std::atomic<int> g_kind[vr::k_unMaxTrackedDeviceCount];
static std::atomic<int> g_hmd{-1};             // the eligible HMD's index
static std::atomic<bool> g_off{false};         // a wired headset: poses pass untouched
static std::atomic<bool> g_hide{false};        // the headset is about to place the lighthouse devices
static std::atomic<double> g_unplaced{0};      // when a controller or tracker first came while not placed (0: none)
constexpr double kHideFor = 20;                // s from then, at most
struct StationPose {
  char serial[32];
  int cls;
  std::atomic<int> channel;  // 0 unknown
  std::atomic<int> role;     // TrackedControllerRole (1 left, 2 right)
  std::atomic<uint32_t> seq;
  double p[3];
  double q[4];
};
static StationPose g_dev[vr::k_unMaxTrackedDeviceCount];
static std::atomic<bool> g_connected[vr::k_unMaxTrackedDeviceCount];
static std::atomic<float> g_battery[vr::k_unMaxTrackedDeviceCount];
static std::atomic<bool> g_charging[vr::k_unMaxTrackedDeviceCount];
static std::atomic<long long> g_hooked{0}, g_moved{0};

static void OnHaptic(int hand, int us) {
  if (!g_st) return;
  int device = -1;
  const int want = hand == 0 ? vr::TrackedControllerRole_LeftHand
                             : vr::TrackedControllerRole_RightHand;
  for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
    if (g_dev[i].cls == vr::TrackedDeviceClass_Controller && g_dev[i].role.load() == want) {
      device = (int)i;
      break;
    }
  }
  std::lock_guard<std::mutex> g(g_st_m);
  InterlockedIncrement((volatile LONG *)&g_st->seq);
  MemoryBarrier();
  g_st->haptic_device = device;
  g_st->haptic_hand = hand;
  g_st->haptic_us = us;
  MemoryBarrier();
  InterlockedIncrement((volatile LONG *)&g_st->haptic_seq);
  InterlockedIncrement((volatile LONG *)&g_st->seq);
}

// last corrected lighthouse pose and last HMD pose, for the input sender
struct PoseSnap {
  std::atomic<uint32_t> seq{0};
  double t = 0;
  double p[3]{};
  double q[4]{1, 0, 0, 0};
};
static PoseSnap g_pose[vr::k_unMaxTrackedDeviceCount];
static PoseSnap g_hmd_pose;

static void StorePose(PoseSnap &s, double t, const double p[3], const vr::HmdQuaternion_t &q) {
  s.seq.fetch_add(1, std::memory_order_acq_rel);
  std::atomic_thread_fence(std::memory_order_release);
  s.t = t;
  memcpy(s.p, p, sizeof s.p);
  s.q[0] = q.w; s.q[1] = q.x; s.q[2] = q.y; s.q[3] = q.z;
  std::atomic_thread_fence(std::memory_order_release);
  s.seq.fetch_add(1, std::memory_order_acq_rel);
}

static bool LoadPose(PoseSnap &s, double p[3], double q[4], double &t) {
  for (int i = 0; i < 8; i++) {
    const uint32_t a = s.seq.load(std::memory_order_acquire);
    if (a & 1) continue;
    p[0] = s.p[0]; p[1] = s.p[1]; p[2] = s.p[2];
    q[0] = s.q[0]; q[1] = s.q[1]; q[2] = s.q[2]; q[3] = s.q[3];
    t = s.t;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (s.seq.load(std::memory_order_acquire) == a) return true;
  }
  return false;
}

static vr::HmdQuaternion_t Mul(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

static void Rotate(const vr::HmdQuaternion_t &q, const double v[3], double out[3]) {
  double ux = q.x, uy = q.y, uz = q.z;  // v + 2w(u x v) + 2 u x (u x v)
  double cx = uy * v[2] - uz * v[1], cy = uz * v[0] - ux * v[2], cz = ux * v[1] - uy * v[0];
  double dx = uy * cz - uz * cy, dy = uz * cx - ux * cz, dz = ux * cy - uy * cx;
  out[0] = v[0] + 2 * (q.w * cx + dx);
  out[1] = v[1] + 2 * (q.w * cy + dy);
  out[2] = v[2] + 2 * (q.w * cz + dz);
}

// the pose as OpenVR reports it (raw universe): WorldFromDriver * pose * DriverFromHead
static void RawPose(const vr::DriverPose_t &p, double pos[3], vr::HmdQuaternion_t &q) {
  double head[3], loc[3], raw[3];
  Rotate(p.qRotation, p.vecDriverFromHeadTranslation, head);
  for (int i = 0; i < 3; i++) loc[i] = p.vecPosition[i] + head[i];
  Rotate(p.qWorldFromDriverRotation, loc, raw);
  for (int i = 0; i < 3; i++) pos[i] = raw[i] + p.vecWorldFromDriverTranslation[i];
  q = Mul(Mul(p.qWorldFromDriverRotation, p.qRotation), p.qDriverFromHeadRotation);
}

static bool Apply(uint32_t id, vr::DriverPose_t &p) {
  g_hooked.fetch_add(1, std::memory_order_relaxed);
  if (g_off.load(std::memory_order_relaxed) || id >= vr::k_unMaxTrackedDeviceCount) return false;
  int kind = g_kind[id].load(std::memory_order_relaxed);
  if (kind == kHmd) {
    if ((int)id == g_hmd.load(std::memory_order_relaxed) && p.poseIsValid && g_sync) {
      double pos[3];
      vr::HmdQuaternion_t q;
      RawPose(p, pos, q);
      g_sync->OnHmdPose(QpcNow() + p.poseTimeOffset, Quat{q.w, q.x, q.y, q.z}, V3{pos[0], pos[1], pos[2]});
      StorePose(g_hmd_pose, QpcNow() + p.poseTimeOffset, pos, q);
    }
    return false;
  }
  if (kind != kLighthouse) return false;
  StationPose &d = g_dev[id];
  // a base station's pose is kept only while valid: SteamVR hands out zero poses while it starts and stops (a NaN
  // reference frame, and the measured station "408 cm away")
  if (d.cls == vr::TrackedDeviceClass_TrackingReference && p.poseIsValid && p.deviceIsConnected &&
      p.result == vr::TrackingResult_Running_OK) {
    double pos[3];
    vr::HmdQuaternion_t q;
    RawPose(p, pos, q);
    double qq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (qq > 0.9 && qq < 1.1 && std::isfinite(pos[0] + pos[1] + pos[2])) {
      d.seq.fetch_add(1, std::memory_order_acq_rel);
      memcpy(d.p, pos, sizeof pos);
      d.q[0] = q.w; d.q[1] = q.x; d.q[2] = q.y; d.q[3] = q.z;
      d.seq.fetch_add(1, std::memory_order_acq_rel);
    }
  }
  // controllers and trackers: worn or held, they stay near the head, which tells a fit from its mirror (sync.cpp)
  if (d.cls != vr::TrackedDeviceClass_TrackingReference && p.poseIsValid && p.result == vr::TrackingResult_Running_OK &&
      g_sync) {
    double pos[3];
    vr::HmdQuaternion_t q;
    RawPose(p, pos, q);
    if (std::isfinite(pos[0] + pos[1] + pos[2])) {
      double t = QpcNow() + p.poseTimeOffset;
      g_sync->OnBodyPose((int)id, t, V3{pos[0], pos[1], pos[2]});
      if (g_gravity) g_gravity->OnPose((int)id, t, V3{pos[0], pos[1], pos[2]}, Quat{q.w, q.x, q.y, q.z});
    }
  }
  Xf x;
  if (!ReadXf(x)) {
    // not placed yet while the headset's cameras are about to: hidden, rather than shown metres away in SteamVR's own
    // lighthouse universe and then snapped into place
    if (d.cls == vr::TrackedDeviceClass_TrackingReference || !g_hide.load(std::memory_order_relaxed)) return false;
    double now = QpcNow(), since = g_unplaced.load(std::memory_order_relaxed);
    if (since == 0) g_unplaced.compare_exchange_strong(since, now);
    else if (now - since > kHideFor) return false;
    p.poseIsValid = false;
    p.result = vr::TrackingResult_Running_OutOfRange;
    return true;
  }
  vr::HmdQuaternion_t xq{x.q.w, x.q.x, x.q.y, x.q.z};
  double wt[3];
  Rotate(xq, p.vecWorldFromDriverTranslation, wt);
  p.qWorldFromDriverRotation = Mul(xq, p.qWorldFromDriverRotation);
  for (int i = 0; i < 3; i++) p.vecWorldFromDriverTranslation[i] = wt[i] + x.t[i];
  if (p.poseIsValid && p.result == vr::TrackingResult_Running_OK) {
    double pos[3];
    vr::HmdQuaternion_t q;
    RawPose(p, pos, q);
    StorePose(g_pose[id], QpcNow() + p.poseTimeOffset, pos, q);
  }
  g_moved.fetch_add(1, std::memory_order_relaxed);
  return true;
}

using PoseFn = void (*)(void *, uint32_t, const vr::DriverPose_t &, uint32_t);
static PoseFn g_orig[2];
static void *g_target[2], *g_detour[2];

template <int I>
static void Detour(void *self, uint32_t id, const vr::DriverPose_t &pose, uint32_t size) {
  if (size == sizeof(vr::DriverPose_t)) {
    vr::DriverPose_t p = pose;
    if (Apply(id, p)) { g_orig[I](self, id, p, size); return; }
  }
  g_orig[I](self, id, pose, size);
}

static void HookHost(const char *version, int i, void *detour) {
  vr::EVRInitError err = vr::VRInitError_None;
  void *host = vr::VRDriverContext()->GetGenericInterface(version, &err);
  if (!host) { Log(Fmt("no %s", version)); return; }
  void *fn = (*(void ***)host)[1];
  for (int j = 0; j < 2; j++)
    if (g_target[j] == fn) { Log(Fmt("%s shares the hooked function", version)); return; }
  if (MH_CreateHook(fn, detour, (void **)&g_orig[i]) != MH_OK || MH_EnableHook(fn) != MH_OK) {
    Log(Fmt("hooking %s failed", version));
    return;
  }
  g_target[i] = fn;
  g_detour[i] = detour;
  Log(Fmt("hooked %s::TrackedDevicePoseUpdated", version));
}

// takes the pose hooks out, each only while its function still jumps straight to our detour (MinHook's jmp to a relay
// holding the detour's address): one another driver has hooked over since stays, passing poses on untouched, as taking
// it out would take theirs with it
static void UnhookHost() {
  for (int j = 0; j < 2; j++) {
    if (!g_target[j]) continue;
    auto *p = (const uint8_t *)g_target[j];
    const uint8_t *relay = p[0] == 0xE9 ? p + 5 + *(const int32_t *)(p + 1) : nullptr;
    if (relay && relay[0] == 0xFF && relay[1] == 0x25 && *(const int32_t *)(relay + 2) == 0 &&
        *(void *const *)(relay + 6) == g_detour[j]) {
      MH_DisableHook(g_target[j]);
      Log("pose hook taken out");
    } else {
      Log("another driver hooked SteamVR's pose updates after QuestLHSync: its hook stays, passing poses on untouched");
    }
  }
}

// ---------------------------------------------------------------- status shared memory
static std::mutex g_stlog_m;
static std::vector<std::string> g_pending_log;

static void PushStatusLog(const std::string &s) {
  // Keep the dashboard window quiet; detailed input traces stay in questlhsync.log.
  if (s.rfind("input ", 0) == 0 || s.rfind("lighthouse input", 0) == 0 ||
      s.rfind("haptic", 0) == 0)
    return;
  std::lock_guard<std::mutex> g(g_stlog_m);
  time_t t = time(nullptr);
  struct tm tm;
  localtime_s(&tm, &t);
  char b[16];
  strftime(b, sizeof b, "%H:%M:%S", &tm);
  g_pending_log.push_back(std::string(b) + "  " + s);
  if (g_pending_log.size() > 16) g_pending_log.erase(g_pending_log.begin());
}

static void Copy(char *dst, size_t n, const std::string &s) {
  strncpy(dst, s.c_str(), n - 1);
  dst[n - 1] = 0;
}

static std::string GetStr(vr::PropertyContainerHandle_t c, vr::ETrackedDeviceProperty p) {
  char buf[256] = "";
  vr::ETrackedPropertyError e = vr::TrackedProp_Success;
  vr::VRProperties()->GetStringProperty(c, p, buf, sizeof buf, &e);
  return e == vr::TrackedProp_Success ? buf : "";
}

// the streamer's name for people, from its driver's
static std::string StreamerName(const std::string &driver) {
  static const char *const names[][2] = {{"oculus_virtualdesktop", "Virtual Desktop"}, {"oculus", "Quest Link"},
                                         {"creolecast", "CreoleCast"}, {"vrlink", "Steam Link"},
                                         {"alvr_server", "ALVR"}};
  for (auto &n : names)
    if (driver == n[0]) return n[1];
  return driver;
}

// ---------------------------------------------------------------- provider
class Provider : public vr::IServerTrackedDeviceProvider {
 public:
  vr::EVRInitError Init(vr::IVRDriverContext *ctx) override {
    VR_INIT_SERVER_DRIVER_CONTEXT(ctx);
    timeBeginPeriod(1);  // the input sender sleeps 8 ms; the default timer rounds that to 15.6 ms
    dir_ = DataDir();
    {
      std::lock_guard<std::mutex> g(g_log_m);
      std::string lp = dir_ + "\\questlhsync.log";
      std::error_code ec;
      if (std::filesystem::file_size(lp, ec) > (2 << 20)) std::filesystem::rename(lp, lp + ".old", ec);
      g_logf = fopen(lp.c_str(), "a");
    }
    map_ = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, sizeof(QlhsStatus), QLHS_SHM_NAME);
    if (map_) g_st = (QlhsStatus *)MapViewOfFile(map_, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(QlhsStatus));
    if (g_st) {
      memset((void *)g_st, 0, sizeof(QlhsStatus));
      g_st->version = QLHS_VERSION;
      g_st->magic = QLHS_MAGIC;
    }
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) g_battery[i].store(-1.f);
    Log("QuestLHSync " QLHS_RELEASE " driver starting, data in %LOCALAPPDATA%\\QuestLHSync");
    SyncConfig cfg;
    cfg.dir = dir_;
    cfg.per_headset = true;
    g_sync = std::make_unique<Sync>(cfg, [](const std::string &s) { Log(s); });
    g_gravity = std::make_unique<Gravity>(dir_, [](const std::string &s) { Log(s); });
    g_gravity->SetRecord([](double t, const std::string &s) { if (g_sync) g_sync->Rec(t, "%s", s.c_str()); });
    g_rel = std::make_unique<Relations>(dir_, [](const std::string &s) { Log(s); });
    link_ = std::make_unique<HeadsetLink>(
        g_sync.get(), [](const std::string &s) { Log(s); },
        [](double t, const std::string &s) { if (g_sync) g_sync->Rec(t, "%s", s.c_str()); });
    link_->SetHapticHandler([](int hand, int us) { OnHaptic(hand, us); });
    link_->SetMemory(dir_ + "\\headset.txt");
    ReadSettings();
    if (MH_Initialize() != MH_OK) { Log("MH_Initialize failed"); return vr::VRInitError_None; }
    HookHost("IVRServerDriverHost_006", 0, (void *)&Detour<0>);
    HookHost("IVRServerDriverHost_005", 1, (void *)&Detour<1>);
    g_lh = std::make_unique<LighthouseInput>(ctx, [](const std::string &s) { Log(s); });
    g_lh->Poll();  // hook the context before lighthouse can cache its IVRDriverInput
    rel_done_ = !steady_ || g_rel->Hook();  // the lighthouse driver loads first; if not, the worker retries
    link_->Start();
    run_ = true;
    input_thread_ = std::thread(&Provider::InputWorker, this);
    worker_ = std::thread(&Provider::Worker, this);
    return vr::VRInitError_None;
  }

  void Cleanup() override {
    run_ = false;
    if (input_thread_.joinable()) input_thread_.join();
    if (worker_.joinable()) worker_.join();
    if (g_gravity) g_gravity->Stop();
    if (link_) link_->Stop();
    if (g_lh) {
      g_lh->Stop();
      g_lh.reset();
    }
    for (int j = 0; j < 2; j++)
      if (g_target[j]) MH_DisableHook(g_target[j]);
    if (g_rel) { g_rel->Unhook(); g_rel->Flush(true); }
    MH_Uninitialize();
    if (g_sync) g_sync->SetRecord(nullptr);
    Log("QuestLHSync driver stopped");
    timeEndPeriod(1);
    // g_sync and the mappings stay: a late pose update on another thread may still touch them while vrserver exits
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
  }

  const char *const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

  void RunFrame() override {
    if (wired_) return;
    vr::TrackedDevicePose_t raw[vr::k_unMaxTrackedDeviceCount]{};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, raw, vr::k_unMaxTrackedDeviceCount);
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
      g_connected[i].store(raw[i].bDeviceIsConnected);
    if (++tick_ % 45) return;  // classify devices about twice a second
    auto *props = vr::VRProperties();
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
      int kind = g_kind[i].load();
      if (kind == kLighthouse && g_dev[i].cls == vr::TrackedDeviceClass_TrackingReference) ReadChannel(i);
      if (kind == kLighthouse && g_dev[i].cls != vr::TrackedDeviceClass_TrackingReference) ReadReceiver(i);
      if (kind != kUnknown) continue;
      auto c = props->TrackedDeviceToPropertyContainer(i);
      if (c == vr::k_ulInvalidPropertyContainer) continue;
      std::string sys = GetStr(c, vr::Prop_TrackingSystemName_String);
      if (sys.empty()) continue;
      vr::ETrackedPropertyError e = vr::TrackedProp_Success;
      int32_t cls = props->GetInt32Property(c, vr::Prop_DeviceClass_Int32, &e);
      std::string serial = GetStr(c, vr::Prop_SerialNumber_String), model = GetStr(c, vr::Prop_ModelNumber_String);
      if (cls == vr::TrackedDeviceClass_HMD && sys == "lighthouse" && g_hmd.load() < 0) {  // an Index, a Vive, ...
        SetWired(sys, model);
        g_kind[i] = kOther;  // not moved: its own lighthouse tracking is the truth
        continue;
      }
      if (sys == "lighthouse") {
        Copy(g_dev[i].serial, sizeof g_dev[i].serial, serial);
        g_dev[i].cls = cls;
        g_kind[i] = kLighthouse;
        Log(Fmt("device %u: %s %s (%s) -> moved", i, sys.c_str(), serial.c_str(), model.c_str()));
        if (cls == vr::TrackedDeviceClass_TrackingReference) ReadChannel(i);
        else ReadReceiver(i);
        continue;
      }
      if (cls == vr::TrackedDeviceClass_HMD && g_hmd.load() < 0) {
        // a Quest Pro, 3, 3S or a Steam Frame, whoever streams it: Link, Air Link, Virtual Desktop, ALVR, Steam Link,
        // CreoleCast, ...
        std::string family;
        for (const std::string &p : {model, GetStr(c, vr::Prop_RenderModelName_String), serial,
                                     GetStr(c, vr::Prop_ManufacturerName_String)})
          if (family.empty()) family = QuestFamily(p);
        // Virtual Desktop goes through SteamVR's oculus driver like Link does, but times its poses its own way: the
        // actual driver name tells them apart ("oculus_virtualdesktop")
        std::string actual = GetStr(c, vr::Prop_ActualTrackingSystemName_String);
        if (!actual.empty()) sys = actual;
        // Virtual Desktop names every headset newer than the Quest Pro "Meta Quest 3", ALVR the one it emulates (a
        // Quest 2 unless set otherwise): neither name tells, so whichever headset answers is used (the one last used
        // with that streamer first). Virtual Desktop's "Meta Quest Pro" is one, though, and a Steam Frame or another
        // Quest left awake isn't used in its place
        bool any = any_hmd_ || (sys == "oculus_virtualdesktop" && family != "Quest Pro") || sys == "alvr_server";
        if (family.empty() && !any && StreamerName(sys) == sys) {  // no streamer's driver: a headset on a cable
          SetWired(sys, model);
          g_kind[i] = kOther;
          continue;
        }
        if (!family.empty() || any) {
          link_->SetFamily(any ? "" : family);  // before g_hmd: the link searches from then on
          link_->SetStreamer(sys);
          g_hmd = (int)i;
          g_kind[i] = kHmd;
          std::lock_guard<std::mutex> g(hmd_m_);
          hmd_model_ = model.empty() ? "headset" : model;
          hmd_system_ = StreamerName(sys);
          g_sync->SetStreamer(sys);
          Log(Fmt("HMD %u: %s via %s%s", i, hmd_model_.c_str(), hmd_system_.c_str(), !family.empty() || !any_hmd_ ? "" :
                  " - not named a Quest Pro, 3, 3S or Steam Frame, used because anyHmd is set"));
          continue;
        }
        std::lock_guard<std::mutex> g(hmd_m_);
        hmd_model_ = model.empty() ? "headset" : model;
        hmd_system_ = StreamerName(sys);
        Log(Fmt("HMD %u: %s via %s isn't a Quest Pro, 3, 3S or Steam Frame: idle", i, hmd_model_.c_str(),
                hmd_system_.c_str()));
      }
      g_kind[i] = kOther;
    }
  }

  // a headset that isn't streamed: nothing of QuestLHSync applies, the worker stands down and the dashboard closes
  void SetWired(const std::string &sys, const std::string &model) {
    if (wired_.exchange(true)) return;
    g_off = true;
    g_gravity->SetEnabled(false);
    g_rel->SetEnabled(false);
    Log(Fmt("HMD: %s (%s) isn't streamed, it's a wired headset: QuestLHSync is off", model.empty() ? "headset" : model.c_str(),
            sys.c_str()));
  }

  // a 2.0 base station's mode label is its channel, "1".."16"
  void ReadChannel(uint32_t i) {
    auto c = vr::VRProperties()->TrackedDeviceToPropertyContainer(i);
    if (c == vr::k_ulInvalidPropertyContainer) return;
    std::string label = GetStr(c, vr::Prop_ModeLabel_String);
    int ch = 0;
    if (!label.empty() && std::all_of(label.begin(), label.end(), [](char x) { return isdigit((unsigned char)x); }))
      ch = atoi(label.c_str());
    if (ch < 1 || ch > 16) ch = 0;
    if (ch != g_dev[i].channel.exchange(ch) && !label.empty())
      Log(Fmt("base station %s: channel %s", g_dev[i].serial, label.c_str()));
  }

  // a controller's or tracker's wireless receiver (gravity.h reads its accelerometer through it)
  void ReadReceiver(uint32_t i) {
    auto c = vr::VRProperties()->TrackedDeviceToPropertyContainer(i);
    if (c == vr::k_ulInvalidPropertyContainer) return;
    // The role hint can arrive after the dongle string (and wired controllers have no dongle
    // string at all), so read it before the receiver early-out.
    vr::ETrackedPropertyError re = vr::TrackedProp_Success;
    const int32_t role = vr::VRProperties()->GetInt32Property(c, vr::Prop_ControllerRoleHint_Int32, &re);
    if (re == vr::TrackedProp_Success) g_dev[i].role.store(role);
    auto *props = vr::VRProperties();
    vr::ETrackedPropertyError be = vr::TrackedProp_Success;
    g_battery[i].store(props->GetFloatProperty(c, vr::Prop_DeviceBatteryPercentage_Float, &be));
    if (be != vr::TrackedProp_Success) g_battery[i].store(-1.f);
    be = vr::TrackedProp_Success;
    const bool charging = props->GetBoolProperty(c, vr::Prop_DeviceIsCharging_Bool, &be);
    g_charging[i].store(be == vr::TrackedProp_Success && charging);
    std::string r = GetStr(c, vr::Prop_ConnectedWirelessDongle_String);
    if (r == receiver_[i]) return;
    receiver_[i] = r;
    g_gravity->SetDevice((int)i, g_dev[i].serial, r);
  }

  bool ShouldBlockStandbyMode() override { return false; }
  void EnterStandby() override {}
  void LeaveStandby() override {}

 private:
  HANDLE map_ = nullptr;
  unsigned tick_ = 0;
  std::string dir_;
  std::unique_ptr<HeadsetLink> link_;
  std::atomic<bool> run_{false};
  std::thread worker_, input_thread_;
  std::mutex hmd_m_;
  std::string hmd_model_, hmd_system_;
  std::string receiver_[vr::k_unMaxTrackedDeviceCount];
  double last_gravity_ = 0;
  std::atomic<bool> wired_{false};  // SteamVR's headset is wired (not streamed): everything here stays off
  bool any_hmd_ = false, recording_ = false, overlay_started_ = false, steady_ = true, rel_done_ = true;
  bool stood_down_ = false;  // the worker's, once wired: hooks out, search and levelling stopped
  int cmd_seen_ = 0;
  double started_ = 0, last_status_ = 0;
  Sync::Spots rec_spots_;  // when the recording started

  void ReadSettings() {
    auto *s = vr::VRSettings();
    vr::EVRSettingsError e = vr::VRSettingsError_None;
    any_hmd_ = s->GetBool(kSection, "anyHmd", &e);
    if (e != vr::VRSettingsError_None) any_hmd_ = false;
    char hosts[512] = "";
    s->GetString(kSection, "host", hosts, sizeof hosts, &e);
    std::vector<std::string> hv;
    if (e == vr::VRSettingsError_None) {
      std::string h = hosts, cur;
      for (char c : h + ",") {
        if (c == ',' || c == ' ' || c == ';') { if (!cur.empty()) hv.push_back(cur); cur.clear(); }
        else cur += c;
      }
    }
    link_->SetHosts(hv);
    char pref[64] = "";
    s->GetString(kSection, "headset", pref, sizeof pref, &e);
    if (e == vr::VRSettingsError_None) link_->SetPreferred(pref);
    bool grav = s->GetBool(kSection, "gravity", &e);  // levelling by resting devices' gravity (gravity.h); on unset
    g_gravity->SetEnabled(e != vr::VRSettingsError_None || grav);
    bool steady = s->GetBool(kSection, "steadyStations", &e);  // base station moves averaged (relations.h); on unset
    steady_ = e != vr::VRSettingsError_None || steady;
    g_rel->SetEnabled(steady_);
    bool rec = s->GetBool(kSection, "record", &e);
    SetRecording(e == vr::VRSettingsError_None && rec);
  }

  void SetRecording(bool on) {
    if (on == recording_) return;
    recording_ = on;
    if (!on) {
      g_sync->SetRecord(nullptr);
      Log("recording off: " + Sync::Describe(rec_spots_, g_sync->spots()));
      return;
    }
    namespace fs = std::filesystem;
    fs::path rd = fs::path(dir_) / "recordings";
    std::vector<fs::path> old;  // keep the newest 9 + this one
    std::error_code ec;
    fs::create_directories(rd, ec);
    for (auto &e : fs::directory_iterator(rd, ec))
      if (e.path().extension() == ".txt") old.push_back(e.path());
    std::sort(old.begin(), old.end());
    while (old.size() > 9) { fs::remove(old.front(), ec); old.erase(old.begin()); }
    time_t t = time(nullptr);
    struct tm tm;
    localtime_s(&tm, &t);
    char b[64];
    strftime(b, sizeof b, "qlhs-%Y%m%d-%H%M%S.txt", &tm);
    FILE *f = fopen((rd / b).string().c_str(), "w");
    if (!f) { Log("can't open a recording file"); recording_ = false; return; }
    setvbuf(f, nullptr, _IOFBF, 1 << 16);
    g_sync->SetRecord(f);
    rec_spots_ = g_sync->spots();
    g_sync->Rec(QpcNow(), "I QuestLHSync recording (qlhs_replay reads it)");
    Log(std::string("recording to recordings\\") + b);
  }

  void LaunchOverlay() {
    HANDLE m = OpenMutexW(SYNCHRONIZE, FALSE, QLHS_OVERLAY_MUTEX);
    if (m) { CloseHandle(m); return; }
    HMODULE mod = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       (LPCWSTR)&HmdDriverFactoryAnchor, &mod);
    wchar_t path[MAX_PATH];
    GetModuleFileNameW(mod, path, MAX_PATH);
    std::filesystem::path exe = std::filesystem::path(path).parent_path() / L"QuestLHSync.exe";
    if (!std::filesystem::exists(exe)) { Log("dashboard app missing: no QuestLHSync.exe next to the driver"); return; }
    std::wstring cmd = L"\"" + exe.wstring() + L"\"";
    STARTUPINFOW si{};
    si.cb = sizeof si;
    PROCESS_INFORMATION pi{};
    if (CreateProcessW(exe.c_str(), cmd.data(), nullptr, nullptr, FALSE, CREATE_NO_WINDOW | DETACHED_PROCESS, nullptr,
                       exe.parent_path().c_str(), &si, &pi)) {
      CloseHandle(pi.hThread);
      CloseHandle(pi.hProcess);
      Log("dashboard app started");
    } else {
      Log(Fmt("dashboard app didn't start (error %lu)", GetLastError()));
    }
  }

  static void HmdDriverFactoryAnchor() {}

  void Stations() {
    std::map<std::string, std::pair<V3, M3>> raw;
    std::map<std::string, int> chans;
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
      if (g_kind[i].load() != kLighthouse || g_dev[i].cls != vr::TrackedDeviceClass_TrackingReference) continue;
      StationPose &d = g_dev[i];
      if (int ch = d.channel.load()) chans[d.serial] = ch;
      for (int tries = 0; tries < 8; tries++) {
        uint32_t s = d.seq.load(std::memory_order_acquire);
        if (s == 0 || (s & 1)) { Sleep(0); continue; }
        double p[3], q[4];
        memcpy(p, d.p, sizeof p);
        memcpy(q, d.q, sizeof q);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (d.seq.load(std::memory_order_acquire) != s) continue;
        raw[d.serial] = {V3{p[0], p[1], p[2]}, ToM3(Quat{q[0], q[1], q[2], q[3]})};
        break;
      }
    }
    g_sync->SetStationsRaw(raw);
    g_sync->SetChannels(chans);
  }

  void Commands() {
    if (!g_st) return;
    int s = g_st->cmd_seq;
    if (s == cmd_seen_) return;
    cmd_seen_ = s;
    switch (g_st->cmd) {
      case QLHS_CMD_PAUSE: g_sync->SetPaused(true); Log("corrections paused"); break;
      case QLHS_CMD_RESUME: g_sync->SetPaused(false); Log("corrections resumed"); break;
      case QLHS_CMD_RECORD_ON: SetRecording(true); break;
      case QLHS_CMD_RECORD_OFF: SetRecording(false); break;
      case QLHS_CMD_RECALIBRATE: g_sync->ForceRecalibrate(); break;
    }
  }

  void Publish(double now) {
    if (!g_st) return;
    std::lock_guard<std::mutex> g(g_st_m);
    Sync::Status s = g_sync->GetStatus(now);
    int hmd = g_hmd.load();
    std::string model, system;
    {
      std::lock_guard<std::mutex> g(hmd_m_);
      model = hmd_model_;
      system = hmd_system_;
    }
    int state;
    auto ls = link_->state();
    if (wired_) state = QLHS_WIRED;
    else if (hmd < 0) state = QLHS_NO_HMD;
    else if (ls == HeadsetLink::kSearching || ls == HeadsetLink::kIdle) state = QLHS_SEARCHING;
    else if (ls == HeadsetLink::kConnecting) state = QLHS_CONNECTING;
    else if (now - link_->last_frame() > 3 && now - link_->connected_at() > 5) state = QLHS_NO_CAMERAS;
    else if (s.nstations < 2) state = QLHS_NO_STATIONS;
    else if (s.locked) state = QLHS_LOCKED;
    else state = QLHS_ACQUIRING;
    QlhsStatus *t = g_st;
    InterlockedIncrement((volatile LONG *)&t->seq);  // odd: writing
    MemoryBarrier();
    t->state = state;
    t->updated = now;
    Copy(t->hmd, sizeof t->hmd, model);
    Copy(t->hmd_system, sizeof t->hmd_system, system);
    Copy(t->headset, sizeof t->headset, link_->model());
    Copy(t->headset_addr, sizeof t->headset_addr, ls == HeadsetLink::kConnected ? link_->addr() : "");
    Copy(t->headset_fw, sizeof t->headset_fw, link_->fw());
    t->cam_fps = s.cam_fps;
    t->sight_rate = s.sight_rate;
    t->spot_rate = s.spot_rate;
    t->head_still = s.head_still;
    t->rtt_ms = s.rtt * 1000;
    t->expo_ms = s.expo * 1000;
    t->expo_learned = s.timing_learned;
    t->nst = (int)std::min<size_t>(s.st.size(), 8);
    for (int i = 0; i < t->nst; i++) {
      auto &e = s.st[i];
      Copy(t->st[i].serial, sizeof t->st[i].serial, e.serial);
      t->st[i].support = e.support;
      t->st[i].anchor = e.anchor;
      t->st[i].measured = e.measured;
      t->st[i].last_seen = e.last_seen;
      t->st[i].dist = e.dist;
    }
    t->locked = s.locked;
    t->cond = s.cond;
    t->yaw_deg = s.has_x ? s.x[0] * kDeg : 0;
    for (int i = 0; i < 3; i++) t->t[i] = s.has_x ? s.x[i + 1] : 0;
    t->med_deg = s.med;
    t->nfit = s.n;
    t->paused = g_sync->paused();
    t->locked_for = s.locked_for;
    t->lag_cm = s.lag_cm;
    t->recording = recording_;
    {
      std::lock_guard<std::mutex> g(g_stlog_m);
      for (auto &l : g_pending_log) {
        Copy(t->log[t->nlog % 8], sizeof t->log[0], l);
        t->nlog++;
      }
      g_pending_log.clear();
    }
    MemoryBarrier();
    InterlockedIncrement((volatile LONG *)&t->seq);
  }

  // a wired headset: the hooks come out, the headset search and levelling stop, as close to the add-on being off as
  // a loaded driver gets
  void StandDown() {
    stood_down_ = true;
    link_->Stop();
    g_gravity->Stop();
    g_rel->Unhook();
    UnhookHost();
  }

  void InputWorker() {
    uint32_t seq = 0;
    bool last_sys[2] = {}, last_a[2] = {}, last_b[2] = {}, last_sc[2] = {}, last_pc[2] = {};
    bool last_tc[2] = {};
    bool last_pad_touch[2] = {};
    bool first[2] = {true, true};
    bool was_connected[2] = {false, false};
    double release_until[2] = {0, 0};
    double release_last[2] = {0, 0};
    double invalid_since[2] = {0, 0};
    double bad_pose_since[2] = {0, 0};
    bool invalid_logged[2] = {false, false};
    double hmd_bad_since = 0;
    bool hmd_bad_logged = false;
    auto release_hand = [&](int hand) {
      if (!was_connected[hand]) return;
      char d[24];
      snprintf(d, sizeof d, "D %d\n", hand);
      link_->SubmitInput(d);
      was_connected[hand] = false;
      first[hand] = true;
    };
    std::set<uint32_t> logged_bad_role;
    while (run_) {
      if (!link_ || !link_->CapInput() || !g_gravity) {
        Sleep(20);
        continue;
      }
      double hp[3] = {0, 0, 0}, hq[4] = {1, 0, 0, 0}, ht = 0;
      const bool have_hmd = LoadPose(g_hmd_pose, hp, hq, ht);
      const vr::HmdQuaternion_t inv{hq[0], -hq[1], -hq[2], -hq[3]};
      const double hpos[3] = {hp[0], hp[1], hp[2]};
      const double now = QpcNow();
      const bool hmd_ok = have_hmd && now - ht < 0.1 && std::isfinite(hp[0] + hp[1] + hp[2]) &&
                          std::isfinite(hq[0] + hq[1] + hq[2] + hq[3]);
      if (!hmd_ok) {
        if (hmd_bad_since == 0) hmd_bad_since = now;
        if (!hmd_bad_logged && now - hmd_bad_since > 2.0) {
          Log("input relay paused: no valid, fresh host HMD pose");
          hmd_bad_logged = true;
        }
      } else {
        hmd_bad_since = 0;
        hmd_bad_logged = false;
      }
      uint32_t hand_dev[2] = {vr::k_unTrackedDeviceIndexInvalid, vr::k_unTrackedDeviceIndexInvalid};
      for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; id++) {
        if (g_kind[id].load() != kLighthouse || g_dev[id].cls != vr::TrackedDeviceClass_Controller)
          continue;
        const int role = g_dev[id].role.load();
        if (role != vr::TrackedControllerRole_LeftHand && role != vr::TrackedControllerRole_RightHand) {
          if (logged_bad_role.insert(id).second)
            Log(Fmt("input skip: device %u has no hand role (hint %d)", id, role));
          continue;
        }
        const int hand = role == vr::TrackedControllerRole_RightHand ? 1 : 0;
        if (hand_dev[hand] != vr::k_unTrackedDeviceIndexInvalid && hand_dev[hand] != id)
          continue;  // two devices claim the same hand; first wins this frame
        hand_dev[hand] = id;
        if (!g_connected[id].load()) {
          if (was_connected[hand]) {
            was_connected[hand] = false;
            first[hand] = true;
            release_until[hand] = now + 1.0;
            release_last[hand] = 0;
          }
          bad_pose_since[hand] = 0;
          if (now < release_until[hand] && now - release_last[hand] >= 0.1) {
            char d[24];
            snprintf(d, sizeof d, "D %d\n", hand);
            link_->SubmitInput(d);
            release_last[hand] = now;
          }
          continue;
        }
        LighthouseInput::State in;
        if (g_lh) in = g_lh->GetState(id);
        if (!in.valid) {
          if (invalid_since[hand] == 0) invalid_since[hand] = now;
          if (!invalid_logged[hand] && now - invalid_since[hand] > 2.0) {
            Log(Fmt("input capture invalid for hand %d for 2 s; releasing", hand));
            invalid_logged[hand] = true;
          }
          release_hand(hand);
          bad_pose_since[hand] = 0;
          continue;
        }
        invalid_since[hand] = 0;
        invalid_logged[hand] = false;
        double p[3], q[4], t = 0;
        const bool pose_ok = LoadPose(g_pose[id], p, q, t) && now - t < 0.1 &&
                             std::isfinite(p[0] + p[1] + p[2]);
        if (!hmd_ok || !pose_ok) {
          if (bad_pose_since[hand] == 0) bad_pose_since[hand] = now;
          if (now - bad_pose_since[hand] >= 0.4) release_hand(hand);
          continue;
        }
        bad_pose_since[hand] = 0;
        was_connected[hand] = true;
        release_until[hand] = 0;
        double rel[3] = {0, 0, -0.45};
        vr::HmdQuaternion_t rq{1, 0, 0, 0};
        {
          const double diff[3] = {p[0] - hpos[0], p[1] - hpos[1], p[2] - hpos[2]};
          Rotate(inv, diff, rel);
          rq = Mul(inv, vr::HmdQuaternion_t{q[0], q[1], q[2], q[3]});
        }
        uint32_t buttons = 0;
        if (first[hand]) {
          last_sys[hand] = in.system;
          last_a[hand] = in.a; last_b[hand] = in.b;
          last_sc[hand] = in.stick_click; last_pc[hand] = in.pad_click;
          last_tc[hand] = in.trigger_click;
          last_pad_touch[hand] = false;
          first[hand] = false;
        } else {
          if (in.system && !last_sys[hand]) buttons |= 1;
          if (in.a && !last_a[hand]) buttons |= 2;
          if (in.b && !last_b[hand]) buttons |= 4;
          if (in.stick_click && !last_sc[hand]) buttons |= 8;
          if (in.pad_click && !last_pc[hand]) buttons |= 16;
          if (in.trigger_click && !last_tc[hand]) buttons |= 32;
          last_sys[hand] = in.system;
          last_a[hand] = in.a; last_b[hand] = in.b;
          last_sc[hand] = in.stick_click; last_pc[hand] = in.pad_click;
          last_tc[hand] = in.trigger_click;
        }

        if ((buttons & 16) || (in.pad_touch && !last_pad_touch[hand]))
          OnHaptic(hand, 1500);
        last_pad_touch[hand] = in.pad_touch;
        const uint32_t levels = (in.system ? 1u : 0u) | (in.a ? 2u : 0u) |
                                (in.b ? 4u : 0u) | (in.stick_click ? 8u : 0u) |
                                (in.pad_click ? 16u : 0u) | (in.trigger_click ? 32u : 0u);
        char line[512];
        snprintf(line, sizeof line,
                 "X %d %u %lld %u %.3f %.3f %.3f %.3f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.3f %.3f %d %.3f %d"
                 " %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %d %d %u\n",
                 hand, seq++, (long long)(QpcNow() * 1e9), buttons, in.trigger, in.grip, in.stick_x,
                 in.stick_y, rq.w, rq.x, rq.y, rq.z, rel[0], rel[1], rel[2], in.pad_x, in.pad_y,
                 in.pad_touch ? 1 : 0, g_battery[id].load(),
                 g_charging[id].load() ? 1 : 0,
                 p[0], p[1], p[2], q[0], q[1], q[2], q[3],
                 hpos[0], hpos[1], hpos[2], hq[0], hq[1], hq[2], hq[3], hmd_ok ? 1 : 0,
                 pose_ok ? 1 : 0, levels);
        link_->SubmitInput(line);
      }
      for (int hand = 0; hand < 2; ++hand)
        if (hand_dev[hand] == vr::k_unTrackedDeviceIndexInvalid) release_hand(hand);
      Sleep(8);
    }
  }

  void Worker() {
    started_ = QpcNow();
    while (run_) {
      double now = QpcNow();
      if (wired_) {  // only the status goes on, which closes a dashboard already open
        if (!stood_down_) StandDown();
      g_hide.store(false);
        if (now - last_status_ >= 0.25) { last_status_ = now; Publish(now); }
        Sleep(50);
        continue;
      }
      bool has_controller = false;
      for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
        if (g_kind[i].load() == kLighthouse && g_dev[i].cls == vr::TrackedDeviceClass_Controller)
          has_controller = true;
      link_->SetWanted(g_hmd.load() >= 0 || has_controller);
      Stations();
      Commands();
      if (g_lh) g_lh->Poll();
      Transform xf = g_sync->Tick(now);
      WriteXf(xf);
      g_hide.store(!xf.active && link_->state() == HeadsetLink::kConnected);
      if (xf.active) g_unplaced.store(0);
      if (now - last_gravity_ >= 1) {
        last_gravity_ = now;
        Gravity::Ref ref;
        ref.ok = g_sync->GravityFrame(ref.C, ref.turn, ref.key);
        bool on;
        M3 tilt;
        if (g_gravity->Step(ref, on, tilt)) g_sync->SetGravity(on, tilt);
      }
      if (!rel_done_ && now - started_ < 60) rel_done_ = g_rel->Hook();
      g_rel->Flush();
      if (now - last_status_ >= 0.25) { last_status_ = now; Publish(now); }
      if (!overlay_started_ && !wired_ && now - started_ > 2 && !vr::VRServerDriverHost()->IsExiting()) {
        overlay_started_ = true;
        LaunchOverlay();
      }
      Sleep(50);
    }
  }
};

static Provider g_provider;

extern "C" __declspec(dllexport) void *HmdDriverFactory(const char *name, int *ret) {
  if (strcmp(name, vr::IServerTrackedDeviceProvider_Version) == 0) return &g_provider;
  if (ret) *ret = vr::VRInitError_Init_InterfaceNotFound;
  return nullptr;
}
