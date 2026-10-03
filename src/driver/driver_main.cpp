// QuestLHSync: keeps the lighthouse universe aligned to a headset's own tracking, a rooted Quest Pro's or a Steam
// Frame's, whatever streams it to SteamVR. The headset's lhsyncd (Magisk module / Frame package) serves base station
// laser flashes seen by the tracking cameras; this driver solves the 4-DOF lighthouse -> headset transform (sync.cpp)
// and applies it to every lighthouse-tracked device (trackers, controllers, base stations).
//
// Hooks IVRServerDriverHost::TrackedDevicePoseUpdated (MinHook on the function, so every driver's calls go
// through it): lighthouse devices get the transform prepended to their WorldFromDriver, and the HMD's own poses
// (any streamer's) feed the solver with their exact times. Status and commands for the dashboard overlay
// (QuestLHSync.exe, launched from here) go through the shared memory in qlhs_status.h.
#include <winsock2.h>
#include <windows.h>
#include <shlobj.h>

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
#include <string>
#include <thread>
#include <vector>

#include "../common/qlhs_status.h"
#include "MinHook.h"
#include "net.h"
#include "openvr_driver.h"
#include "sync.h"

static const char *kSection = "driver_questlhsync";

// ---------------------------------------------------------------- logging
static std::mutex g_log_m;
static FILE *g_logf;
static QlhsStatus *g_st;
static std::unique_ptr<Sync> g_sync;

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
struct StationPose { char serial[32]; int cls; std::atomic<uint32_t> seq; double p[3]; double q[4]; };
static StationPose g_dev[vr::k_unMaxTrackedDeviceCount];
static std::atomic<long long> g_hooked{0}, g_moved{0};

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
  if (id >= vr::k_unMaxTrackedDeviceCount) return false;
  int kind = g_kind[id].load(std::memory_order_relaxed);
  if (kind == kHmd) {
    if ((int)id == g_hmd.load(std::memory_order_relaxed) && p.poseIsValid && g_sync) {
      double pos[3];
      vr::HmdQuaternion_t q;
      RawPose(p, pos, q);
      g_sync->OnHmdPose(QpcNow() + p.poseTimeOffset, Quat{q.w, q.x, q.y, q.z}, V3{pos[0], pos[1], pos[2]});
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
    if (std::isfinite(pos[0] + pos[1] + pos[2]))
      g_sync->OnBodyPose((int)id, QpcNow() + p.poseTimeOffset, V3{pos[0], pos[1], pos[2]});
  }
  Xf x;
  if (!ReadXf(x)) return false;
  vr::HmdQuaternion_t xq{x.q.w, x.q.x, x.q.y, x.q.z};
  double wt[3];
  Rotate(xq, p.vecWorldFromDriverTranslation, wt);
  p.qWorldFromDriverRotation = Mul(xq, p.qWorldFromDriverRotation);
  for (int i = 0; i < 3; i++) p.vecWorldFromDriverTranslation[i] = wt[i] + x.t[i];
  g_moved.fetch_add(1, std::memory_order_relaxed);
  return true;
}

using PoseFn = void (*)(void *, uint32_t, const vr::DriverPose_t &, uint32_t);
static PoseFn g_orig[2];
static void *g_target[2];

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
  Log(Fmt("hooked %s::TrackedDevicePoseUpdated", version));
}

// ---------------------------------------------------------------- status shared memory
static std::mutex g_stlog_m;
static std::vector<std::string> g_pending_log;

static void PushStatusLog(const std::string &s) {
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

static std::string Lower(std::string s) {
  std::string o;
  for (char c : s)
    if (isalnum((unsigned char)c)) o += (char)tolower((unsigned char)c);
  return o;
}

static std::string GetStr(vr::PropertyContainerHandle_t c, vr::ETrackedDeviceProperty p) {
  char buf[256] = "";
  vr::ETrackedPropertyError e = vr::TrackedProp_Success;
  vr::VRProperties()->GetStringProperty(c, p, buf, sizeof buf, &e);
  return e == vr::TrackedProp_Success ? buf : "";
}

// ---------------------------------------------------------------- provider
class Provider : public vr::IServerTrackedDeviceProvider {
 public:
  vr::EVRInitError Init(vr::IVRDriverContext *ctx) override {
    VR_INIT_SERVER_DRIVER_CONTEXT(ctx);
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
    Log("QuestLHSync driver starting, data in " + dir_);
    SyncConfig cfg;
    cfg.dir = dir_;
    g_sync = std::make_unique<Sync>(cfg, [](const std::string &s) { Log(s); });
    link_ = std::make_unique<HeadsetLink>(
        g_sync.get(), [](const std::string &s) { Log(s); },
        [](double t, const std::string &s) { if (g_sync) g_sync->Rec(t, "%s", s.c_str()); });
    link_->SetMemory(dir_ + "\\headset.txt");
    ReadSettings();
    if (MH_Initialize() != MH_OK) { Log("MH_Initialize failed"); return vr::VRInitError_None; }
    HookHost("IVRServerDriverHost_006", 0, (void *)&Detour<0>);
    HookHost("IVRServerDriverHost_005", 1, (void *)&Detour<1>);
    link_->Start();
    run_ = true;
    worker_ = std::thread(&Provider::Worker, this);
    return vr::VRInitError_None;
  }

  void Cleanup() override {
    run_ = false;
    if (worker_.joinable()) worker_.join();
    if (link_) link_->Stop();
    for (int j = 0; j < 2; j++)
      if (g_target[j]) MH_DisableHook(g_target[j]);
    MH_Uninitialize();
    if (g_sync) g_sync->SetRecord(nullptr);
    Log("QuestLHSync driver stopped");
    // g_sync and the mappings stay: a late pose update on another thread may still touch them while vrserver exits
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
  }

  const char *const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

  void RunFrame() override {
    if (++tick_ % 45) return;  // classify devices about twice a second
    auto *props = vr::VRProperties();
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
      if (g_kind[i].load() != kUnknown) continue;
      auto c = props->TrackedDeviceToPropertyContainer(i);
      if (c == vr::k_ulInvalidPropertyContainer) continue;
      std::string sys = GetStr(c, vr::Prop_TrackingSystemName_String);
      if (sys.empty()) continue;
      vr::ETrackedPropertyError e = vr::TrackedProp_Success;
      int32_t cls = props->GetInt32Property(c, vr::Prop_DeviceClass_Int32, &e);
      std::string serial = GetStr(c, vr::Prop_SerialNumber_String), model = GetStr(c, vr::Prop_ModelNumber_String);
      if (sys == "lighthouse") {
        Copy(g_dev[i].serial, sizeof g_dev[i].serial, serial);
        g_dev[i].cls = cls;
        g_kind[i] = kLighthouse;
        Log(Fmt("device %u: %s %s (%s) -> moved", i, sys.c_str(), serial.c_str(), model.c_str()));
        continue;
      }
      if (cls == vr::TrackedDeviceClass_HMD && g_hmd.load() < 0) {
        // a Quest Pro or a Steam Frame, whoever streams it: Link, Air Link, Virtual Desktop, ALVR, Steam Link, ...
        std::string all = Lower(model + " " + GetStr(c, vr::Prop_RenderModelName_String) + " " + serial + " " +
                                GetStr(c, vr::Prop_ManufacturerName_String));
        bool known = all.find("questpro") != std::string::npos || all.find("seacliff") != std::string::npos ||
                     all.find("steamframe") != std::string::npos || all.find("deckard") != std::string::npos;
        if (known || any_hmd_) {
          g_hmd = (int)i;
          g_kind[i] = kHmd;
          std::lock_guard<std::mutex> g(hmd_m_);
          hmd_model_ = model.empty() ? "headset" : model;
          hmd_system_ = sys;
          g_sync->SetStreamer(sys);
          Log(Fmt("HMD %u: %s via %s%s", i, hmd_model_.c_str(), sys.c_str(),
                  known ? "" : " - not named a Quest Pro or Steam Frame, used because anyHmd is set"));
          continue;
        }
        std::lock_guard<std::mutex> g(hmd_m_);
        hmd_model_ = model.empty() ? "headset" : model;
        hmd_system_ = sys;
        Log(Fmt("HMD %u: %s via %s isn't a Quest Pro or Steam Frame: idle", i, hmd_model_.c_str(), sys.c_str()));
      }
      g_kind[i] = kOther;
    }
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
  std::thread worker_;
  std::mutex hmd_m_;
  std::string hmd_model_, hmd_system_;
  bool any_hmd_ = false, recording_ = false, overlay_started_ = false;
  int cmd_seen_ = 0;
  double started_ = 0, last_status_ = 0;

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
    bool rec = s->GetBool(kSection, "record", &e);
    SetRecording(e == vr::VRSettingsError_None && rec);
  }

  void SetRecording(bool on) {
    if (on == recording_) return;
    recording_ = on;
    if (!on) { g_sync->SetRecord(nullptr); Log("recording off"); return; }
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
    if (!std::filesystem::exists(exe)) { Log("dashboard app missing: " + exe.string()); return; }
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
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
      if (g_kind[i].load() != kLighthouse || g_dev[i].cls != vr::TrackedDeviceClass_TrackingReference) continue;
      StationPose &d = g_dev[i];
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
  }

  void Commands() {
    if (!g_st) return;
    int s = g_st->cmd_seq;
    if (s == cmd_seen_) return;
    cmd_seen_ = s;
    switch (g_st->cmd) {
      case QLHS_CMD_REACQUIRE: g_sync->Reacquire(); break;
      case QLHS_CMD_PAUSE: g_sync->SetPaused(true); Log("corrections paused"); break;
      case QLHS_CMD_RESUME: g_sync->SetPaused(false); Log("corrections resumed"); break;
      case QLHS_CMD_RECORD_ON: SetRecording(true); break;
      case QLHS_CMD_RECORD_OFF: SetRecording(false); break;
    }
  }

  void Publish(double now) {
    if (!g_st) return;
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
    if (hmd < 0) state = QLHS_NO_HMD;
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

  void Worker() {
    started_ = QpcNow();
    while (run_) {
      double now = QpcNow();
      link_->SetWanted(g_hmd.load() >= 0);
      Stations();
      Commands();
      WriteXf(g_sync->Tick(now));
      if (now - last_status_ >= 0.25) { last_status_ = now; Publish(now); }
      if (!overlay_started_ && now - started_ > 2 && !vr::VRServerDriverHost()->IsExiting()) {
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
