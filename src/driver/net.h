// QuestLHSync's link to the headset's lhsyncd (Magisk module): UDP discovery, then one TCP stream per session.
//   discovery  UDP 47281: "QLHS?" (broadcast + configured hosts) -> "QLHS 1 <serial> <tcp port> <model>"
//   stream     TCP 47280: "H ..." hello, "C <len> <name>" + the camera calibration, then lhsight's lines
//              (F frames, T heartbeats, I/W/E messages); "P <seq> <pc_ns>" -> "Q <seq> <pc_ns> <mono_ns>" round trips
#pragma once
#include <atomic>
#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

class Sync;

class HeadsetLink {
 public:
  enum State { kIdle, kSearching, kConnecting, kConnected };
  using LogFn = std::function<void(const std::string &)>;
  using RecFn = std::function<void(double, const std::string &)>;

  HeadsetLink(Sync *sync, LogFn log, RecFn rec) : sync_(sync), log_(std::move(log)), rec_(std::move(rec)) {}
  ~HeadsetLink() { Stop(); }
  void Start();
  void Stop();
  void SetWanted(bool w) { wanted_ = w; }             // an eligible HMD is in SteamVR
  void SetHosts(const std::vector<std::string> &h);   // manual addresses (settings), tried first
  void SetPreferred(const std::string &serial);       // pick this headset when several answer
  void SetFamily(const std::string &family);          // SteamVR's HMD (QuestFamily): skip other kinds of headset
  void SetMemory(const std::string &path);            // the last headset's address, kept there: tried over TCP
                                                      // when nothing answers discovery (networks that drop broadcasts)
  State state() const { return state_; }
  std::string serial() const { std::lock_guard<std::mutex> g(m_); return serial_; }
  std::string model() const { std::lock_guard<std::mutex> g(m_); return model_; }  // "Quest Pro": what the page shows
  std::string addr() const { std::lock_guard<std::mutex> g(m_); return addr_; }
  std::string fw() const { std::lock_guard<std::mutex> g(m_); return fw_; }
  double last_frame() const { return last_frame_; }  // QPC s of the newest camera frame line
  double connected_at() const { return connected_at_; }

 private:
  struct Found { std::string ip, serial, model; int port; };
  Sync *sync_;
  LogFn log_;
  RecFn rec_;
  std::thread th_;
  std::atomic<bool> run_{false}, wanted_{false};
  std::atomic<State> state_{kIdle};
  std::atomic<double> last_frame_{0}, connected_at_{0};
  mutable std::mutex m_;
  std::vector<std::string> hosts_;
  std::string preferred_, serial_, model_, addr_, fw_, mem_path_, last_ip_, failed_ip_, family_, skipped_;
  std::set<std::string> wrong_ips_;  // addresses of headsets of another kind than SteamVR's (until that changes)

  void Loop();
  bool Discover(std::vector<Found> &out);
  bool Session(const Found &f);
};

double QpcNow();  // seconds, QueryPerformanceCounter

// "Quest Pro" or "Quest 3" (the 3S too) from a model name, device codename or serial, whatever spells it
// ("Meta Quest Pro", "Oculus Quest 3", "CREOLECAST-QUEST3S", "eureka", ...); "" for anything else
std::string QuestFamily(const std::string &text);
