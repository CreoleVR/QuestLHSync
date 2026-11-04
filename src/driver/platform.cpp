#include "platform.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>

#ifdef _WIN32
#include <shlobj.h>
#endif

namespace qlhs {

bool ReplaceFile(const std::string &tmp, const std::string &path) {
#ifdef _WIN32
  return MoveFileExA(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING) != 0;
#else
  return std::rename(tmp.c_str(), path.c_str()) == 0;
#endif
}

#ifdef _WIN32
static std::filesystem::path LocalAppData() {
  PWSTR p = nullptr;
  std::filesystem::path d;
  if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, nullptr, &p))) d = p;
  CoTaskMemFree(p);
  return d;
}
#else
static std::filesystem::path Xdg(const char *var, const char *fallback) {
  const char *v = getenv(var);
  if (v && *v == '/') return v;
  const char *home = getenv("HOME");
  return std::filesystem::path(home ? home : "/tmp") / fallback;
}
#endif

std::string DataDir() {
#ifdef _WIN32
  std::filesystem::path d = LocalAppData() / "QuestLHSync";
#else
  std::filesystem::path d = Xdg("XDG_DATA_HOME", ".local/share") / "QuestLHSync";
#endif
  std::error_code ec;
  std::filesystem::create_directories(d, ec);
  return d.string();
}

std::string OpenVrPathsFile() {
#ifdef _WIN32
  return (LocalAppData() / "openvr" / "openvrpaths.vrpath").string();
#else
  return (Xdg("XDG_CONFIG_HOME", ".config") / "openvr" / "openvrpaths.vrpath").string();
#endif
}

}  // namespace qlhs
