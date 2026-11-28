// The Frame-side half of the controller bridge: receives the PC's X samples on a Unix datagram
// socket and publishes one virtual controller per hand to the Frame's SteamVR.
#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "openvr_driver.h"

class FrameInput {
 public:
  FrameInput();
  ~FrameInput();

  bool Start();  // create the sockets and add the virtual controllers
  void Stop();
  void RunFrame();

 private:
  static constexpr int kHands = 2;

  struct Sample {
    uint32_t seq = 0;
    uint32_t buttons = 0;
    float trigger = 0, grip = 0, stick_x = 0, stick_y = 0;
    float pad_x = 0, pad_y = 0;
    bool pad_touch = false;
    float battery = -1;
    bool charging = false;
    double q[4] = {1, 0, 0, 0};
    double p[3] = {0, 0, 0};
    double ap[3] = {0, 0, 0}, aq[4] = {1, 0, 0, 0};
    double hp[3] = {0, 0, 0}, hq[4] = {1, 0, 0, 0};
    bool absolute = false;
    bool hmd_valid = false, pose_valid = false;
    bool flags_present = false;
    uint32_t levels = 0;
    bool levels_valid = false;
    double wp[3] = {0, 0, 0}, wq[4] = {1, 0, 0, 0};
    bool world = false;
    std::chrono::steady_clock::time_point rx;
  };

  class Device final : public vr::ITrackedDeviceServerDriver {
   public:
    explicit Device(int hand);
    vr::EVRInitError Activate(uint32_t id) override;
    void Deactivate() override;
    void EnterStandby() override;
    void *GetComponent(const char *) override { return nullptr; }
    void DebugRequest(const char *request, char *response, uint32_t size) override;
    vr::DriverPose_t GetPose() override { return last_pose_; }
    void Tick(const Sample &s, bool fresh, double now, uint32_t pulse_bits);
    const char *Serial() const;
    vr::ITrackedDeviceServerDriver *Driver() { return this; }
    vr::VRInputComponentHandle_t HapticHandle() const { return haptic_; }

   private:
    struct Pulse {
      bool down = false;
      double until = 0;
      void Fire(double now) {
        down = true;
        until = now + 0.12;
      }
      bool Update(double now) {
        if (down && now >= until) down = false;
        return down;
      }
      void Reset() {
        down = false;
        until = 0;
      }
    };

    int hand_;
    uint32_t index_ = vr::k_unTrackedDeviceIndexInvalid;
    vr::PropertyContainerHandle_t container_ = vr::k_ulInvalidPropertyContainer;
    vr::VRInputComponentHandle_t system_click_ = 0, system_touch_ = 0;
    vr::VRInputComponentHandle_t a_click_ = 0, a_touch_ = 0, b_click_ = 0, b_touch_ = 0;
    vr::VRInputComponentHandle_t trigger_value_ = 0, trigger_click_ = 0, trigger_touch_ = 0;
    vr::VRInputComponentHandle_t grip_value_ = 0, grip_click_ = 0, grip_touch_ = 0;
    vr::VRInputComponentHandle_t stick_x_ = 0, stick_y_ = 0, stick_click_ = 0, stick_touch_ = 0;
    vr::VRInputComponentHandle_t pad_x_ = 0, pad_y_ = 0, pad_touch_ = 0, pad_click_ = 0;
    vr::VRInputComponentHandle_t aim_pose_ = 0;
    vr::VRInputComponentHandle_t haptic_ = 0;
    float last_battery_ = -1;
    bool last_charging_ = false;
    bool battery_seen_ = false;
    bool claiming_ = false;
    vr::DriverPose_t last_pose_{};
    Pulse system_pulse_, a_pulse_, b_pulse_, stick_pulse_, pad_pulse_, trigger_pulse_;
    uint32_t last_seq_ = 0;
    uint32_t pending_buttons_ = 0;
    int active_frames_ = 0;
    bool active_ = false;

    void SetRole(bool claim);
    void ReleaseInputs();
    void SetBool(vr::VRInputComponentHandle_t h, bool v);
    void SetScalar(vr::VRInputComponentHandle_t h, float v);
  };

  std::string path_;
  std::string haptic_path_;
  int fd_ = -1;
  int haptic_fd_ = -1;
  std::atomic<bool> run_{false};
  std::thread thread_;
  std::mutex m_;
  std::atomic<uint32_t> pending_buttons_[kHands] = {};
  std::atomic<bool> link_up_{false};
  Sample latest_[kHands];
  bool have_sample_[kHands] = {false, false};
  bool xf_valid_ = false;
  double xf_q_[4] = {1, 0, 0, 0}, xf_p_[3] = {0, 0, 0};
  bool hmd_prev_ = false;
  double hmd_prev_t_ = 0, hmd_prev_q_[4] = {1, 0, 0, 0}, hmd_prev_p_[3] = {0, 0, 0};
  double hmd_still_since_ = 0;
  double align_wait_since_ = 0;
  bool align_wait_logged_ = false;
  bool have_any_sample_ = false;
  std::chrono::steady_clock::time_point last_sample_rx_;
  std::atomic<bool> realign_{false};
  Device devices_[kHands];

  void Serve();
  void Parse(const char *line, size_t len);
  void PollHaptics();
  void SendHaptic(int hand, const vr::VREvent_HapticVibration_t &hv);
  static double Now();
};
