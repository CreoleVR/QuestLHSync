#include "lighthouse_input.h"

#include <windows.h>

#include <algorithm>
#include <cstring>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <utility>

#include "MinHook.h"
#include "openvr_driver.h"

namespace {
LighthouseInput *g_self = nullptr;
}

struct LighthouseInput::Impl {
  struct Comp {
    uint32_t device = vr::k_unTrackedDeviceIndexInvalid;
    std::string path;
  };

  using GetGenericInterfaceFn = void *(*)(vr::IVRDriverContext *, const char *, vr::EVRInitError *);
  using CreateBoolFn = vr::EVRInputError (*)(vr::IVRDriverInput *, vr::PropertyContainerHandle_t,
                                             const char *, vr::VRInputComponentHandle_t *);
  using UpdateBoolFn = vr::EVRInputError (*)(vr::IVRDriverInput *, vr::VRInputComponentHandle_t,
                                             bool, double);
  using CreateScalarFn = vr::EVRInputError (*)(vr::IVRDriverInput *, vr::PropertyContainerHandle_t,
                                               const char *, vr::VRInputComponentHandle_t *,
                                               vr::EVRScalarType, vr::EVRScalarUnits);
  using UpdateScalarFn = vr::EVRInputError (*)(vr::IVRDriverInput *, vr::VRInputComponentHandle_t,
                                               float, double);

  LogFn log;
  vr::IVRDriverContext *ctx = nullptr;
  void *input = nullptr;
  bool ctx_hooked = false;
  bool input_hooked = false;
  double ctx_hooked_at = 0;
  double ctx_retry_at = 0;
  bool ctx_failure_logged = false;
  bool input_wait_logged = false;
  GetGenericInterfaceFn orig_get_generic = nullptr;
  CreateBoolFn orig_create_bool = nullptr;
  UpdateBoolFn orig_update_bool = nullptr;
  CreateScalarFn orig_create_scalar = nullptr;
  UpdateScalarFn orig_update_scalar = nullptr;

  std::mutex m;
  std::map<uint64_t, Comp> comps;
  std::map<uint32_t, std::map<std::string, float>> values;
  std::set<uint64_t> unknown;
  bool pad_pressed[64] = {};
  double pad_press_at[64] = {};
  std::set<uint32_t> logged_devices;

  void Log(const std::string &s) {
    if (log) log("lighthouse input: " + s);
  }

  uint32_t DeviceForContainer(vr::PropertyContainerHandle_t container) {
    auto *props = vr::VRProperties();
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
      if (props->TrackedDeviceToPropertyContainer(i) == container) return i;
    return vr::k_unTrackedDeviceIndexInvalid;
  }

  void Remember(vr::PropertyContainerHandle_t container, const char *path,
                vr::VRInputComponentHandle_t handle) {
    const uint32_t device = DeviceForContainer(container);
    bool log_device = false;
    {
      std::lock_guard<std::mutex> g(m);
      comps[handle] = {device, path ? path : ""};
      log_device = logged_devices.insert(device).second;
    }
    Log("create handle=" + std::to_string((unsigned long long)handle) + " dev=" +
        std::to_string(device) + " " + (path ? path : ""));
    if (log_device) {
      auto *props = vr::VRProperties();
      const std::string serial = props->GetStringProperty(container, vr::Prop_SerialNumber_String, nullptr);
      const int32_t role = props->GetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, nullptr);
      Log("device dev=" + std::to_string(device) + " role=" + std::to_string(role) + " serial=" +
          serial);
    }
  }

  void Value(uint64_t handle, float value) {
    std::lock_guard<std::mutex> g(m);
    const auto it = comps.find(handle);
    if (it == comps.end()) {
      if (unknown.size() < 32 && unknown.insert(handle).second)
        Log("update unknown handle=" + std::to_string((unsigned long long)handle) + " value=" +
            std::to_string(value));
      return;
    }
    const auto &c = it->second;
    auto &dev = values[c.device];
    const float old = dev.count(c.path) ? dev[c.path] : -999;
    dev[c.path] = value;
    const bool click = c.path.size() >= 6 && c.path.compare(c.path.size() - 6, 6, "/click") == 0;
    if (!click) return;
    const bool changed = old < -100 || (value >= .5f) != (old >= .5f);
    if (changed)
      Log("dev=" + std::to_string(c.device) + " " + c.path + " = " + std::to_string(value));
  }

  void HookContext() {
    if (ctx_hooked || !ctx) return;
    auto **vt = *(void ***)ctx;
    if (!vt) {
      if (!ctx_failure_logged) Log("driver context has no vtable; retrying");
      ctx_failure_logged = true;
      return;
    }
    extern void *GetGenericInterfaceDetour(vr::IVRDriverContext *, const char *,
                                          vr::EVRInitError *);
    if (MH_CreateHook(vt[0], (void *)&GetGenericInterfaceDetour, (void **)&orig_get_generic) !=
        MH_OK) {
      if (!ctx_failure_logged) Log("MH_CreateHook GetGenericInterface failed; retrying");
      ctx_failure_logged = true;
      return;
    }
    if (MH_EnableHook(vt[0]) != MH_OK) {
      if (!ctx_failure_logged) Log("MH_EnableHook GetGenericInterface failed; retrying");
      ctx_failure_logged = true;
      return;
    }
    ctx_hooked = true;
    ctx_failure_logged = false;
    ctx_hooked_at = GetTickCount64() / 1000.0;
    Log("hooked driver context GetGenericInterface");
  }

  void HookInput(void *iface) {
    if (input_hooked || !iface) return;
    auto **vt = *(void ***)iface;
    if (!vt) {
      Log("input interface has no vtable");
      return;
    }
    extern vr::EVRInputError CreateBoolDetour(vr::IVRDriverInput *, vr::PropertyContainerHandle_t,
                                              const char *, vr::VRInputComponentHandle_t *);
    extern vr::EVRInputError UpdateBoolDetour(vr::IVRDriverInput *, vr::VRInputComponentHandle_t,
                                              bool, double);
    extern vr::EVRInputError CreateScalarDetour(vr::IVRDriverInput *, vr::PropertyContainerHandle_t,
                                                const char *, vr::VRInputComponentHandle_t *,
                                                vr::EVRScalarType, vr::EVRScalarUnits);
    extern vr::EVRInputError UpdateScalarDetour(vr::IVRDriverInput *, vr::VRInputComponentHandle_t,
                                                float, double);
    bool ok = true;
    ok &= MH_CreateHook(vt[0], (void *)&CreateBoolDetour, (void **)&orig_create_bool) == MH_OK;
    ok &= MH_CreateHook(vt[1], (void *)&UpdateBoolDetour, (void **)&orig_update_bool) == MH_OK;
    ok &= MH_CreateHook(vt[2], (void *)&CreateScalarDetour, (void **)&orig_create_scalar) == MH_OK;
    ok &= MH_CreateHook(vt[3], (void *)&UpdateScalarDetour, (void **)&orig_update_scalar) == MH_OK;
    if (!ok) {
      Log("failed to hook input component methods");
      return;
    }
    for (int i = 0; i < 4; i++)
      if (MH_EnableHook(vt[i]) != MH_OK) Log("MH_EnableHook input " + std::to_string(i) + " failed");
    input = iface;
    input_hooked = true;
    Log("hooked lighthouse IVRDriverInput at " + std::to_string((unsigned long long)iface));
  }
};

LighthouseInput::LighthouseInput(vr::IVRDriverContext *ctx, LogFn log) : impl_(new Impl) {
  impl_->ctx = ctx;
  impl_->log = std::move(log);
  g_self = this;
}

LighthouseInput::~LighthouseInput() {
  Stop();
  delete impl_;
}

LighthouseInput::State LighthouseInput::GetState(uint32_t device) {
  State s;
  if (!impl_) return s;
  std::lock_guard<std::mutex> g(impl_->m);
  const auto it = impl_->values.find(device);
  if (it == impl_->values.end()) return s;
  auto v = [&](const char *path) {
    const auto j = it->second.find(path);
    return j == it->second.end() ? 0.f : j->second;
  };
  auto has = [&](const char *path) { return it->second.find(path) != it->second.end(); };
  s.valid = true;
  s.system = v("/input/system/click") >= 0.5f;
  s.a = v("/input/a/click") >= 0.5f;
  s.b = v("/input/b/click") >= 0.5f;
  s.stick_click = v("/input/thumbstick/click") >= 0.5f;
  const bool pad_latch = device < 64;
  const int hi = (int)device;
  if (pad_latch) {
    const float force = v("/input/trackpad/force");
    const double now = GetTickCount64() / 1000.0;
    if (!impl_->pad_pressed[hi]) {
      if (force >= 0.65f) {
        impl_->pad_pressed[hi] = true;
        impl_->pad_press_at[hi] = now;
      }
    } else if (force <= 0.25f && now - impl_->pad_press_at[hi] > 0.15) {
      impl_->pad_pressed[hi] = false;
    }
  }
  s.pad_click = (pad_latch && impl_->pad_pressed[hi]) || v("/input/trackpad/click") >= 0.5f;
  s.pad_touch = v("/input/trackpad/touch") >= 0.5f;
  s.trigger_click = v("/input/trigger/click") >= 0.5f;
  s.trigger = v("/input/trigger/value");
  s.grip = has("/input/grip/value") ? v("/input/grip/value") : v("/input/grip/force");
  s.stick_x = v("/input/thumbstick/x");
  s.stick_y = v("/input/thumbstick/y");
  s.pad_x = v("/input/trackpad/x");
  s.pad_y = v("/input/trackpad/y");
  return s;
}

void LighthouseInput::Poll() {
  if (!impl_) return;
  if (!impl_->ctx_hooked) {
    const double now = GetTickCount64() / 1000.0;
    if (now < impl_->ctx_retry_at) return;
    impl_->ctx_retry_at = now + 1.0;
    impl_->HookContext();
    return;
  }
  if (!impl_->input_hooked && !impl_->input_wait_logged) {
    const double now = GetTickCount64() / 1000.0;
    if (now - impl_->ctx_hooked_at > 5.0) {
      impl_->input_wait_logged = true;
      impl_->Log("waiting for lighthouse IVRDriverInput; controllers may already be active");
    }
  }
}

void LighthouseInput::Stop() {
  if (!impl_) return;
  if (impl_->input_hooked && impl_->input) {
    auto **vt = *(void ***)impl_->input;
    if (vt) {
      MH_DisableHook(vt[0]);
      MH_DisableHook(vt[1]);
      MH_DisableHook(vt[2]);
      MH_DisableHook(vt[3]);
    }
  }
  if (impl_->ctx_hooked && impl_->ctx) {
    auto **vt = *(void ***)impl_->ctx;
    if (vt) MH_DisableHook(vt[0]);
  }
  impl_->input = nullptr;
  impl_->input_hooked = false;
  impl_->ctx_hooked = false;
  g_self = nullptr;
}

void *GetGenericInterfaceDetour(vr::IVRDriverContext *self, const char *version,
                                vr::EVRInitError *err) {
  auto *impl = g_self ? g_self->impl() : nullptr;
  if (!impl || !impl->orig_get_generic) return nullptr;
  void *result = impl->orig_get_generic(self, version, err);
  if (result && version && std::strstr(version, "IVRDriverInput")) {
    impl->Log("GetGenericInterface " + std::string(version) + " ctx=" +
              std::to_string((unsigned long long)self) + " iface=" +
              std::to_string((unsigned long long)result));
    if (std::strcmp(version, "IVRDriverInput_004") != 0 &&
        std::strcmp(version, "IVRDriverInput_005") != 0) {
      impl->Log("unsupported IVRDriverInput version; not hooking");
    } else if (self == impl->ctx) {
      impl->Log("IVRDriverInput requested through our own context; not hooking");
    } else {
      impl->HookInput(result);
    }
  }
  return result;
}

vr::EVRInputError CreateBoolDetour(vr::IVRDriverInput *self, vr::PropertyContainerHandle_t container,
                                  const char *path, vr::VRInputComponentHandle_t *handle) {
  auto *impl = g_self ? g_self->impl() : nullptr;
  if (!impl || !impl->orig_create_bool) return vr::VRInputError_InvalidHandle;
  const auto r = impl->orig_create_bool(self, container, path, handle);
  if (r == vr::VRInputError_None && handle) impl->Remember(container, path, *handle);
  return r;
}

vr::EVRInputError UpdateBoolDetour(vr::IVRDriverInput *self, vr::VRInputComponentHandle_t handle,
                                   bool value, double offset) {
  auto *impl = g_self ? g_self->impl() : nullptr;
  if (!impl || !impl->orig_update_bool) return vr::VRInputError_InvalidHandle;
  const auto r = impl->orig_update_bool(self, handle, value, offset);
  impl->Value(handle, value ? 1.f : 0.f);
  return r;
}

vr::EVRInputError CreateScalarDetour(vr::IVRDriverInput *self,
                                     vr::PropertyContainerHandle_t container, const char *path,
                                     vr::VRInputComponentHandle_t *handle, vr::EVRScalarType type,
                                     vr::EVRScalarUnits units) {
  auto *impl = g_self ? g_self->impl() : nullptr;
  if (!impl || !impl->orig_create_scalar) return vr::VRInputError_InvalidHandle;
  const auto r = impl->orig_create_scalar(self, container, path, handle, type, units);
  if (r == vr::VRInputError_None && handle) impl->Remember(container, path, *handle);
  return r;
}

vr::EVRInputError UpdateScalarDetour(vr::IVRDriverInput *self, vr::VRInputComponentHandle_t handle,
                                     float value, double offset) {
  auto *impl = g_self ? g_self->impl() : nullptr;
  if (!impl || !impl->orig_update_scalar) return vr::VRInputError_InvalidHandle;
  const auto r = impl->orig_update_scalar(self, handle, value, offset);
  impl->Value(handle, value);
  return r;
}
