// Small platform layer for the driver: paths, time, sleeping. Windows does what it always did; Linux maps
// %LOCALAPPDATA%\QuestLHSync to $XDG_DATA_HOME/QuestLHSync (~/.local/share/QuestLHSync).
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <shlobj.h>
#define QLHS_SEP "\\"
#define QLHS_DATA_DIR_TEXT "%LOCALAPPDATA%\\QuestLHSync"
#else
#include <sched.h>
#include <time.h>
#include <unistd.h>
#define QLHS_SEP "/"
#define QLHS_DATA_DIR_TEXT "~/.local/share/QuestLHSync"
#endif

namespace qlhs {

inline void LocalTime(time_t t, struct tm &tm) {
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
}

inline void SleepMs(unsigned ms) {
#ifdef _WIN32
  Sleep(ms);
#else
  if (!ms) { sched_yield(); return; }
  struct timespec ts{(time_t)(ms / 1000), (long)(ms % 1000) * 1000000L};
  while (nanosleep(&ts, &ts) != 0) {}
#endif
}

inline uint64_t TickMs() {
#ifdef _WIN32
  return GetTickCount64();
#else
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
#endif
}

inline void CpuRelax() {
#ifdef _WIN32
  YieldProcessor();
#elif defined(__x86_64__) || defined(__i386__)
  __builtin_ia32_pause();
#endif
}

// tmp -> path, replacing it
inline bool ReplaceFile(const std::string &tmp, const std::string &path) {
#ifdef _WIN32
  return MoveFileExA(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING) != 0;
#else
  return std::rename(tmp.c_str(), path.c_str()) == 0;
#endif
}

// Windows: %LOCALAPPDATA%. Linux: $XDG_DATA_HOME, else ~/.local/share
inline std::filesystem::path LocalAppData() {
#ifdef _WIN32
  PWSTR p = nullptr;
  std::filesystem::path d;
  if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, nullptr, &p))) d = std::filesystem::path(p);
  CoTaskMemFree(p);
  return d;
#else
  if (const char *x = getenv("XDG_DATA_HOME"); x && *x) return x;
  if (const char *h = getenv("HOME"); h && *h) return std::filesystem::path(h) / ".local" / "share";
  return "/tmp";
#endif
}

// SteamVR's openvrpaths.vrpath (it names SteamVR's config folder). Windows: %LOCALAPPDATA%\openvr. Linux:
// $XDG_CONFIG_HOME/openvr, else ~/.config/openvr
inline std::filesystem::path OpenVrPathsFile() {
#ifdef _WIN32
  return LocalAppData() / "openvr" / "openvrpaths.vrpath";
#else
  std::filesystem::path base;
  if (const char *x = getenv("XDG_CONFIG_HOME"); x && *x) base = x;
  else if (const char *h = getenv("HOME"); h && *h) base = std::filesystem::path(h) / ".config";
  return base / "openvr" / "openvrpaths.vrpath";
#endif
}

}  // namespace qlhs
