// QuestLHSync's link to the headset's lhsyncd (Quest: Magisk module, Frame: user service): UDP discovery, then one TCP
// stream per session.
//   discovery  UDP 47281: "QLHS?" (broadcast, and straight to the hosts, the headsets remembered and the PC's TCP peers on
//              its networks) -> "QLHS 1 <serial> <tcp port> <model> <idle|capturing>
//              peers=<ip,..|->" (peers from module 1.29 on: who the headset has connections with, see lhsyncd.c)
//   stream     TCP 47280: "H ..." hello, "C <len> <name>" + the camera calibration, then lhsight's lines
//              (F frames, T heartbeats, I/W/E messages); "P <seq> <pc_ns>" -> "Q <seq> <pc_ns> <mono_ns>" round trips
//
// Which headset is SteamVR's, when several of its kind answer (two Quest Pros in one room, each streamed to its own
// PC): the one that streams with this PC. The streamer keeps a connection between the PC and the headset it streams,
// and either end shows it: this PC's TCP table, or the peers the headset reports. A headset whose peers are all other
// machines streams to one of them and goes after the rest. Without either (a cable, a streamer the tables don't show),
// the memory decides: the headset last used with this streamer first. What the settings name comes before all of that,
// and the page's Switch headset button passes over the current headset for the rest of the session.
#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

class Sync;

class HeadsetLink {
 public:
  enum State { kIdle, kSearching, kConnecting, kConnected };
  struct Known { std::string ip, id, model, streamer; };  // a headset that connected (the memory, latest first); id:
                                                          // HeadsetId of its serial
  using LogFn = std::function<void(const std::string &)>;
  using RecFn = std::function<void(double, const std::string &)>;

  HeadsetLink(Sync *sync, LogFn log, RecFn rec) : sync_(sync), log_(std::move(log)), rec_(std::move(rec)) {}
  ~HeadsetLink() { Stop(); }
  void Start();
  void Stop();
  void SetWanted(bool w) { wanted_ = w; }             // an eligible HMD is in SteamVR
  void SetHosts(const std::vector<std::string> &h);   // manual addresses (settings): asked and tried over TCP too, and
                                                      // used ahead of other headsets
  void SetPreferred(const std::string &headset);      // the headset to use when several answer: its serial, its id
                                                      // (headset.txt, the log) or its address
  void SetFamily(const std::string &family);          // SteamVR's HMD (QuestFamily): skip other kinds of headset
  void SetStreamer(const std::string &driver);        // SteamVR's streamer: with several headsets, the one last used
                                                      // with it comes first when the kind can't tell
  void SetHmdSerial(const std::string &serial);       // SteamVR's HMD's serial: a headset with that serial is it
  void SetMemory(const std::string &path);            // the headsets' addresses, kept there: tried over TCP when
                                                      // nothing answers discovery (networks that drop broadcasts)
  void NextHeadset() { next_ = true; }                // the page's Switch headset: leave this one for another
  State state() const { return state_; }
  std::string serial() const { std::lock_guard<std::mutex> g(m_); return serial_; }
  std::string model() const { std::lock_guard<std::mutex> g(m_); return model_; }  // "Quest Pro": what the page shows
  std::string addr() const { std::lock_guard<std::mutex> g(m_); return addr_; }
  std::string fw() const { std::lock_guard<std::mutex> g(m_); return fw_; }
  std::string why() const { std::lock_guard<std::mutex> g(m_); return why_; }  // why this headset, for the page
  int headsets() const;  // headsets that could be SteamVR's answering lately (the page offers a switch from 2 on)
  double last_frame() const { return last_frame_; }  // QPC s of the newest camera frame line
  double connected_at() const { return connected_at_; }

 private:
  struct Found {
    std::string ip, serial, model;
    int port;
    bool has_peers = false;          // the headset reports its peers (module 1.29 on)
    std::vector<std::string> peers;  // ... the machines it has connections with
  };
  enum Own { kMine, kUnknown, kTheirs };  // streams with this PC, can't tell, streams with other machines
  struct Answer { Found f; double t = 0; };
  Sync *sync_;
  LogFn log_;
  RecFn rec_;
  std::thread th_;
  std::atomic<bool> run_{false}, wanted_{false}, next_{false};
  std::atomic<State> state_{kIdle};
  std::atomic<double> last_frame_{0}, connected_at_{0};
  mutable std::mutex m_;
  std::vector<std::string> hosts_;
  std::string preferred_, serial_, model_, addr_, fw_, mem_path_, failed_ip_, family_, streamer_, skipped_, hmd_serial_;
  std::string why_;
  std::vector<Known> known_;
  std::set<std::string> wrong_ips_;  // addresses of headsets of another kind than SteamVR's (until that changes)
  std::set<std::string> quiet_ips_;  // ... and of ones that sent no camera frames while others might (tried last)
  std::set<std::string> answered_;   // ... and of those of SteamVR's kind that answered the last discovery
  std::set<std::string> passed_;     // ... and of those Switch headset left this session (tried last)
  std::string picked_;               // the address Switch headset went to: kept, whatever the connections say
  bool picking_ = false;             // Switch headset was pressed: the next headset that sends frames is picked_
  std::map<std::string, Answer> answers_;  // the latest discovery answer of each address
  std::set<std::string> tcp_, local_;      // this PC's TCP peers (TcpPeers) and own addresses, as of the last look
  std::vector<std::pair<uint32_t, uint32_t>> nets_;  // ... and its networks (address, mask; host order)

  void Loop();
  int Rank(const std::string &ip, const std::string &id) const;  // m_ held; less is better
  Own Owner(const std::string &ip, const std::string &id, std::string *why) const;  // m_ held
  bool Named(const std::string &ip, const std::string &id) const;  // m_ held: what the settings name
  int Count() const;  // m_ held: headsets()
  std::string Why(const std::string &ip, const std::string &id) const;  // takes m_: why_ for this headset
  void Look(const std::vector<Found> &found);  // takes m_: new answers, and this PC's connections now
  std::vector<std::string> Targets() const;    // m_ held: the addresses discovery asks straight
  bool OnLan(const std::string &ip) const;     // m_ held: on one of this PC's networks
  void Remember(const Known &k);  // m_ held
  bool Others(const std::string &ip) const;  // m_ held: another headset this could be, by discovery, memory or manual
                                             // hosts
  bool Discover(std::vector<Found> &out);
  bool Session(const Found &f);
};

double QpcNow();  // seconds, QueryPerformanceCounter

// "Quest Pro", "Quest 3" (the 3S too) or "Steam Frame" from a model name, device codename or serial, whatever spells
// it ("Meta Quest Pro", "Oculus Quest 3", "CREOLECAST-QUEST3S", "eureka", "Deckard MP", ...); "" for anything else
std::string QuestFamily(const std::string &text);
