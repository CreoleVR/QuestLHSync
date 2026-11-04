// The few things the driver needs from the OS, for Windows and Linux alike: where its data goes, a path joined the
// OS's way, sleeping, a millisecond tick and local time.
#pragma once
#include <cstdint>
#include <ctime>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#define QLHS_SEP "\\"
#else
#include <sched.h>
#include <unistd.h>
#define QLHS_SEP "/"
#endif

namespace qlhs {

inline void SleepMs(unsigned ms) {
#ifdef _WIN32
  Sleep(ms);
#else
  usleep((useconds_t)ms * 1000);
#endif
}

inline void Yield() {
#ifdef _WIN32
  YieldProcessor();
#else
  sched_yield();
#endif
}

inline uint64_t TickMs() {
#ifdef _WIN32
  return GetTickCount64();
#else
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
#endif
}

inline void LocalTime(time_t t, struct tm *out) {
#ifdef _WIN32
  localtime_s(out, &t);
#else
  localtime_r(&t, out);
#endif
}

// dir + separator + name
inline std::string Join(const std::string &dir, const std::string &name) {
  if (dir.empty()) return name;
  char e = dir.back();
  return dir + ((e == '\\' || e == '/') ? "" : QLHS_SEP) + name;
}

// the replace step of an atomic write (tmp -> path)
bool ReplaceFile(const std::string &tmp, const std::string &path);

// %LOCALAPPDATA%\QuestLHSync on Windows; $XDG_DATA_HOME/QuestLHSync (~/.local/share/QuestLHSync) on Linux. Created.
std::string DataDir();

// SteamVR's openvrpaths.vrpath: %LOCALAPPDATA%\openvr\... or $XDG_CONFIG_HOME/openvr/... (~/.config/openvr/...)
std::string OpenVrPathsFile();

}  // namespace qlhs
