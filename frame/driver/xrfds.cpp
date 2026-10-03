// questlhsync_frame: a SteamVR driver for the Steam Frame's own SteamVR, with no devices. It only lends lhsyncd the
// tracking cameras' frame buffers. XRService (the Frame's tracking) owns the cameras and runs as a child of vrserver;
// the kernel (yama ptrace_scope 1) lets only an ancestor duplicate another process's fds, so this has to happen here,
// in vrserver, and no root is needed.
//   $XDG_RUNTIME_DIR/questlhsync/xrfds.sock, same user only:
//   "G <fd> <fd> ...\n" (XRService's fd numbers, from VIDIOC_QUERYBUF) -> "OK <xrservice pid> <n>\n" with the n fds
//   attached (SCM_RIGHTS), or "E <why>\n". Only dmabufs are handed out.
// It reads nothing from the buffers and never touches XRService otherwise.
#include <dirent.h>
#include <stdarg.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <string>
#include <thread>

#include "openvr_driver.h"

namespace {

constexpr int kMaxFds = 64;

void Log(const char *fmt, ...) {
  char b[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  if (vr::VRDriverLog()) vr::VRDriverLog()->Log(b);
}

// vrserver's own child named XRService
pid_t XrServicePid() {
  DIR *d = opendir("/proc");
  if (!d) return 0;
  pid_t me = getpid(), found = 0;
  while (dirent *e = readdir(d)) {
    if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
    char p[300], buf[512];
    snprintf(p, sizeof p, "/proc/%s/stat", e->d_name);
    int fd = open(p, O_RDONLY);
    if (fd < 0) continue;
    ssize_t n = read(fd, buf, sizeof buf - 1);
    close(fd);
    if (n <= 0) continue;
    buf[n] = 0;
    const char *r = strrchr(buf, ')');  // "pid (comm) state ppid ..."; comm may hold spaces
    int ppid = 0;
    char state;
    if (!r || sscanf(r + 1, " %c %d", &state, &ppid) != 2 || ppid != me) continue;
    snprintf(p, sizeof p, "/proc/%s/cmdline", e->d_name);
    fd = open(p, O_RDONLY);
    if (fd < 0) continue;
    n = read(fd, buf, sizeof buf - 1);
    close(fd);
    if (n <= 0) continue;
    buf[n] = 0;
    const char *base = strrchr(buf, '/') ? strrchr(buf, '/') + 1 : buf;
    if (!strcmp(base, "XRService")) { found = atoi(e->d_name); break; }
  }
  closedir(d);
  return found;
}

bool IsDmabuf(pid_t pid, int fd) {
  char p[64], l[64];
  snprintf(p, sizeof p, "/proc/%d/fd/%d", pid, fd);
  ssize_t n = readlink(p, l, sizeof l - 1);
  if (n <= 0) return false;
  l[n] = 0;
  return !strncmp(l, "/dmabuf:", 8);
}

void Reply(int c, const char *text, const int *fds, int n) {
  iovec io{(void *)text, strlen(text)};
  msghdr m{};
  m.msg_iov = &io;
  m.msg_iovlen = 1;
  alignas(cmsghdr) char ctl[CMSG_SPACE(sizeof(int) * kMaxFds)];
  if (n > 0) {
    m.msg_control = ctl;
    m.msg_controllen = CMSG_SPACE(sizeof(int) * n);
    cmsghdr *h = CMSG_FIRSTHDR(&m);
    h->cmsg_level = SOL_SOCKET;
    h->cmsg_type = SCM_RIGHTS;
    h->cmsg_len = CMSG_LEN(sizeof(int) * n);
    memcpy(CMSG_DATA(h), fds, sizeof(int) * n);
  }
  sendmsg(c, &m, MSG_NOSIGNAL);
}

void Serve(int c) {
  ucred cr{};
  socklen_t cl = sizeof cr;
  if (getsockopt(c, SOL_SOCKET, SO_PEERCRED, &cr, &cl) || cr.uid != getuid()) return;
  char req[1024];
  size_t len = 0;
  while (len < sizeof req - 1) {  // one line, 2 s at most
    pollfd pf{c, POLLIN, 0};
    if (poll(&pf, 1, 2000) <= 0) return;
    ssize_t n = recv(c, req + len, sizeof req - 1 - len, 0);
    if (n <= 0) return;
    len += n;
    if (memchr(req, '\n', len)) break;
  }
  req[len] = 0;
  if (req[0] != 'G') { Reply(c, "E bad request\n", nullptr, 0); return; }
  pid_t xr = XrServicePid();
  if (!xr) { Reply(c, "E XRService isn't running under this SteamVR\n", nullptr, 0); return; }
  int pfd = (int)syscall(SYS_pidfd_open, xr, 0);
  if (pfd < 0) { Reply(c, "E pidfd_open failed\n", nullptr, 0); return; }
  int got[kMaxFds], n = 0;
  char err[128] = "";
  for (char *s = req + 1, *end; n < kMaxFds; s = end) {
    long fd = strtol(s, &end, 10);
    if (end == s) break;
    if (!IsDmabuf(xr, (int)fd)) { snprintf(err, sizeof err, "E fd %ld isn't one of XRService's dmabufs\n", fd); break; }
    int g = (int)syscall(SYS_pidfd_getfd, pfd, (int)fd, 0);
    if (g < 0) { snprintf(err, sizeof err, "E pidfd_getfd(%ld): %s\n", fd, strerror(errno)); break; }
    got[n++] = g;
  }
  close(pfd);
  if (err[0]) Reply(c, err, nullptr, 0);
  else {
    char ok[64];
    snprintf(ok, sizeof ok, "OK %d %d\n", (int)xr, n);
    Reply(c, ok, got, n);
  }
  for (int i = 0; i < n; i++) close(got[i]);
}

class Provider : public vr::IServerTrackedDeviceProvider {
 public:
  vr::EVRInitError Init(vr::IVRDriverContext *ctx) override {
    VR_INIT_SERVER_DRIVER_CONTEXT(ctx);
    const char *rt = getenv("XDG_RUNTIME_DIR");
    if (!rt || !rt[0]) { Log("no XDG_RUNTIME_DIR, not serving"); return vr::VRInitError_None; }
    std::string dir = std::string(rt) + "/questlhsync";
    mkdir(dir.c_str(), 0700);
    path_ = dir + "/xrfds.sock";
    sockaddr_un a{};
    a.sun_family = AF_UNIX;
    if (path_.size() >= sizeof a.sun_path) return vr::VRInitError_None;
    memcpy(a.sun_path, path_.c_str(), path_.size() + 1);
    unlink(path_.c_str());
    ls_ = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (ls_ < 0 || bind(ls_, (sockaddr *)&a, sizeof a) || listen(ls_, 4)) {
      Log("can't listen on %s: %s", path_.c_str(), strerror(errno));
      if (ls_ >= 0) close(ls_);
      ls_ = -1;
      return vr::VRInitError_None;
    }
    chmod(path_.c_str(), 0600);
    run_ = true;
    th_ = std::thread([this] {
      while (run_) {
        pollfd pf{ls_, POLLIN, 0};
        if (poll(&pf, 1, 500) <= 0) continue;
        int c = accept4(ls_, nullptr, nullptr, SOCK_CLOEXEC);
        if (c < 0) continue;
        Serve(c);
        close(c);
      }
    });
    Log("lending XRService's camera buffers on %s", path_.c_str());
    return vr::VRInitError_None;
  }
  void Cleanup() override {
    run_ = false;
    if (th_.joinable()) th_.join();
    if (ls_ >= 0) { close(ls_); unlink(path_.c_str()); ls_ = -1; }
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
  }
  const char *const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
  void RunFrame() override {}
  bool ShouldBlockStandbyMode() override { return false; }
  void EnterStandby() override {}
  void LeaveStandby() override {}

 private:
  int ls_ = -1;
  std::string path_;
  std::atomic<bool> run_{false};
  std::thread th_;
};

Provider g_provider;

}  // namespace

extern "C" __attribute__((visibility("default"))) void *HmdDriverFactory(const char *iface, int *code) {
  if (!strcmp(iface, vr::IServerTrackedDeviceProvider_Version)) return &g_provider;
  if (code) *code = vr::VRInitError_Init_InterfaceNotFound;
  return nullptr;
}
