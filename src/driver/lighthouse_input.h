// Captures the lighthouse driver's IVRDriverInput interface by hooking the driver-context
// GetGenericInterface function. The lighthouse driver requests its input interface through the
// same shared context vtable; when it asks for IVRDriverInput we hook that object's component
// creation/update methods. This is the state SteamVR uses to render the knuckles, including the
// system button, and it is focus-independent.
#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace vr {
class IVRDriverContext;
}

class LighthouseInput {
 public:
  using LogFn = std::function<void(const std::string &)>;
  struct Impl;

  LighthouseInput(vr::IVRDriverContext *ctx, LogFn log);
  ~LighthouseInput();

  struct State {
    bool valid = false;
    bool system = false;
    bool a = false;
    bool b = false;
    bool stick_click = false;
    bool pad_click = false;
    bool pad_touch = false;
    bool trigger_click = false;
    float trigger = 0;
    float grip = 0;
    float stick_x = 0;
    float stick_y = 0;
    float pad_x = 0;
    float pad_y = 0;
  };
  State GetState(uint32_t device);

  void Poll();  // install the context hook; safe to call repeatedly
  void Stop();
  Impl *impl() { return impl_; }

 private:
  Impl *impl_;
};
