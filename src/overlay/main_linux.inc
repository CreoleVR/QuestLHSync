// Linux main for overlay_main.cpp (included at its end): the page in the SteamVR dashboard and, when SDL2 and a
// display are there, in a desktop window. Mirrors the Windows wWinMain above it.

#include "sdl_dyn.h"

// ---------------------------------------------------------------- SteamVR dashboard page
struct Dash {
  vr::VROverlayHandle_t main = vr::k_ulOverlayHandleInvalid, thumb = vr::k_ulOverlayHandleInvalid;

  bool Start() {
    if (vr::VROverlay()->CreateDashboardOverlay("questlhsync.dashboard", "QuestLHSync", &main, &thumb) !=
        vr::VROverlayError_None)
      return false;
    vr::VROverlay()->SetOverlayWidthInMeters(main, 2.4f);
    vr::VROverlay()->SetOverlayInputMethod(main, vr::VROverlayInputMethod_Mouse);
    vr::HmdVector2_t scale{(float)W, (float)H};
    vr::VROverlay()->SetOverlayMouseScale(main, &scale);
    std::vector<uint8_t> icon;
    DrawIcon(icon, 256);
    vr::VROverlay()->SetOverlayRaw(thumb, icon.data(), 256, 256, 4);
    return true;
  }
  // SetOverlayRaw: no graphics API needed. The page goes up at most 5 times a second, and only while it's shown
  void Show(std::vector<uint8_t> &rgba) { vr::VROverlay()->SetOverlayRaw(main, rgba.data(), W, H, 4); }
};

// ---------------------------------------------------------------- desktop window
struct Window {
  sdl::Api api;
  void *win = nullptr, *ren = nullptr, *tex = nullptr, *hand = nullptr, *arrow = nullptr;
  bool hidden = false;

  bool Start() {
    if (!getenv("DISPLAY") && !getenv("WAYLAND_DISPLAY")) return false;
    if (!api.Load()) return false;
    api.SetHint("SDL_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR", "0");
    api.SetHint("SDL_RENDER_SCALE_QUALITY", "1");
    if (api.Init(sdl::INIT_VIDEO) != 0) return false;
    // SteamVR may be starting under a game: the window comes up without taking the focus where the WM allows
    api.SetHint("SDL_WINDOW_NO_ACTIVATION_WHEN_SHOWN", "1");
    win = api.CreateWindow("QuestLHSync by CreoleVR", sdl::WINDOWPOS_UNDEFINED, sdl::WINDOWPOS_UNDEFINED, W * 3 / 4,
                           H * 3 / 4, sdl::WINDOW_SHOWN | sdl::WINDOW_RESIZABLE | sdl::WINDOW_ALLOW_HIGHDPI);
    if (!win) { api.Quit(); return false; }
    ren = api.CreateRenderer(win, -1, sdl::RENDERER_ACCELERATED);
    if (!ren) ren = api.CreateRenderer(win, -1, 0);
    if (!ren) { api.DestroyWindow(win); api.Quit(); win = nullptr; return false; }
    api.RenderSetLogicalSize(ren, W, H);  // keeps the page's shape, and mouse positions come in page pixels
    tex = api.CreateTexture(ren, sdl::PIXELFORMAT_ABGR8888, sdl::TEXTUREACCESS_STREAMING, W, H);
    static std::vector<uint8_t> icon;
    DrawIcon(icon, 64);
    if (void *srf = api.CreateRGBSurfaceFrom(icon.data(), 64, 64, 32, 64 * 4, 0x000000FF, 0x0000FF00, 0x00FF0000,
                                             0xFF000000)) {
      api.SetWindowIcon(win, srf);
      api.FreeSurface(srf);
    }
    hand = api.CreateSystemCursor(sdl::CURSOR_HAND);
    arrow = api.CreateSystemCursor(sdl::CURSOR_ARROW);
    return true;
  }

  bool Visible() const {
    return win && !hidden && !(api.GetWindowFlags(win) & (sdl::WINDOW_HIDDEN | sdl::WINDOW_MINIMIZED));
  }

  void Present(const std::vector<uint8_t> &rgba) {
    if (!tex) return;
    api.UpdateTexture(tex, nullptr, rgba.data(), W * 4);
    api.SetRenderDrawColor(ren, (col::bg >> 16) & 255, (col::bg >> 8) & 255, col::bg & 255, 255);
    api.RenderClear(ren);
    api.RenderCopy(ren, tex, nullptr, nullptr);
    api.RenderPresent(ren);
  }

  // events: hover and clicks on the page's buttons (SendCmd); true when the window needs drawing
  bool Pump(Page &pg, int &down, bool &ui) {
    if (!win) return false;
    bool redraw = false;
    sdl::Event e;
    while (api.PollEvent(&e)) {
      switch (e.type) {
        case sdl::WINDOWEVENT:
          if (e.window.event == sdl::WE_CLOSE) {  // the dashboard page stays; the window is back next SteamVR start
            api.HideWindow(win);
            hidden = true;
          } else if (e.window.event == sdl::WE_EXPOSED || e.window.event == sdl::WE_SIZE_CHANGED) {
            redraw = true;
          } else if (e.window.event == sdl::WE_LEAVE && pg.hover >= 0) {
            pg.hover = -1;
            ui = true;
          }
          break;
        case sdl::MOUSEMOTION: {
          int hit = HitAt((float)e.motion.x, (float)e.motion.y);
          if (hit != pg.hover) {
            pg.hover = hit;
            ui = true;
            api.SetCursor(hit >= 0 ? hand : arrow);
          }
          break;
        }
        case sdl::MOUSEBUTTONDOWN:
          if (e.button.button == sdl::BUTTON_LEFT) down = HitAt((float)e.button.x, (float)e.button.y);
          break;
        case sdl::MOUSEBUTTONUP:
          if (e.button.button == sdl::BUTTON_LEFT) {
            int hit = HitAt((float)e.button.x, (float)e.button.y);
            if (hit >= 0 && hit == down) { SendCmd(pg.buttons[hit].cmd); ui = true; }
            down = -1;
          }
          break;
        case sdl::QUIT:  // the last window closed: handled as WE_CLOSE above
          break;
      }
    }
    return redraw;
  }

  ~Window() {
    if (win) { api.DestroyWindow(win); api.Quit(); }
  }
};

// vrserver is running (a process named vrserver)
static bool ServerUp() {
  DIR *d = opendir("/proc");
  if (!d) return true;
  bool up = false;
  while (dirent *e = readdir(d)) {
    if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
    char path[300], comm[64] = "";
    snprintf(path, sizeof path, "/proc/%s/comm", e->d_name);
    FILE *f = fopen(path, "r");
    if (!f) continue;
    if (fgets(comm, sizeof comm, f)) {
      comm[strcspn(comm, "\n")] = 0;
      up = !strcmp(comm, "vrserver");
    }
    fclose(f);
    if (up) break;
  }
  closedir(d);
  return up;
}

// one dashboard app per user: an flock held while it runs (the driver checks it before starting one)
static int TakeLock() {
  std::string lock;
  if (const char *r = getenv("XDG_RUNTIME_DIR"); r && *r) lock = std::string(r) + "/" QLHS_OVERLAY_LOCK ".lock";
  else lock = "/tmp/" QLHS_OVERLAY_LOCK "-" + std::to_string((unsigned)getuid()) + ".lock";
  int fd = open(lock.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
  if (fd < 0) return -2;  // can't tell: go on
  if (flock(fd, LOCK_EX | LOCK_NB) != 0) { close(fd); return -1; }
  return fd;
}

int main(int argc, char **argv) {
  if (argc >= 3 && !strcmp(argv[1], "--preview")) {
    const char *path = argv[2], *kind = argc >= 4 ? argv[3] : "locked";
    Canvas big(W * SS, H * SS);
    Page pg;
    std::vector<uint8_t> rgba;
    if (!strcmp(kind, "icon")) {
      DrawIcon(rgba, 256);
      return WritePng(path, rgba, 256, 256) ? 0 : 1;
    }
    Draw(big, pg, Sample(kind), !strcmp(kind, "stale"));
    Downsample(big, rgba);
    return WritePng(path, rgba, W, H) ? 0 : 1;
  }

  int lock = TakeLock();
  if (lock == -1) return 0;  // one runs already
  // The driver starts us while SteamVR is still coming up, maybe with no headset yet (it can connect much later).
  // We live as long as vrserver runs; the dashboard page joins once the driver reports a headset (VR_Init without
  // one is refused, and would start SteamVR if it weren't running).
  Canvas big(W * SS, H * SS);
  Page pg;
  g_page = &big;
  g_pg = &pg;
  QlhsStatus st{}, prev{};
  ReadStatus(st);
  Draw(big, pg, st, false);
  Window win;
  bool have_win = win.Start();
  Dash dash;
  bool vr_up = false, quit = false;
  std::vector<uint8_t> rgba;
  double last_upd = 0;
  uint64_t last_change = TickMs(), last_draw = 0, next_try = 0, last_check = 0;
  int down = -1, wdown = -1;
  bool first = true;
  while (!quit) {
    uint64_t now = TickMs();
    bool expose = have_win && win.Pump(pg, wdown, g_ui);
    if (now - last_check >= 1000) {  // also when vrserver crashed or was killed, with no Quit sent
      last_check = now;
      if (!ServerUp()) {
        vr_up = false;  // nothing left to shut down
        break;
      }
    }
    bool have = ReadStatus(st);
    if (!have) memset(&st, 0, sizeof st);
    if (st.state == QLHS_WIRED) break;  // a wired headset: nothing to show, and no dashboard page to join
    if (!vr_up && have && st.hmd[0] && now >= next_try) {
      vr::EVRInitError err = vr::VRInitError_None;
      vr::VR_Init(&err, vr::VRApplication_Overlay);
      if (err == vr::VRInitError_None && dash.Start()) vr_up = true;
      else {
        if (err == vr::VRInitError_None) vr::VR_Shutdown();
        next_try = now + 5000;
      }
    }
    if (vr_up) {
      vr::VREvent_t ev;
      while (vr::VRSystem()->PollNextEvent(&ev, sizeof ev))
        if (ev.eventType == vr::VREvent_Quit) {
          vr::VRSystem()->AcknowledgeQuit_Exiting();
          quit = true;
        }
      while (vr::VROverlay()->PollNextOverlayEvent(dash.main, &ev, sizeof ev)) {
        float mx = ev.data.mouse.x, my = H - ev.data.mouse.y;  // overlay mouse y is from the bottom
        switch (ev.eventType) {
          case vr::VREvent_MouseMove: {
            int h = HitAt(mx, my);
            if (h != pg.hover) { pg.hover = h; g_ui = true; }
            break;
          }
          case vr::VREvent_MouseButtonDown: down = HitAt(mx, my); break;
          case vr::VREvent_MouseButtonUp: {
            int h = HitAt(mx, my);
            if (h >= 0 && h == down) { SendCmd(pg.buttons[h].cmd); g_ui = true; }
            down = -1;
            break;
          }
          case vr::VREvent_OverlayShown: g_dirty = true; break;
        }
      }
      if (quit) break;
    }
    if (st.updated != last_upd) { last_upd = st.updated; last_change = now; }
    bool stale = !have || now - last_change > 3000;
    if (memcmp(&st, &prev, sizeof st)) { prev = st; g_dirty = true; }
    bool win_vis = have_win && win.Visible();
    bool dash_vis = vr_up && vr::VROverlay()->IsOverlayVisible(dash.main);
    if ((win_vis || dash_vis) &&
        (first || g_ui || expose || ((g_dirty || now - last_draw > 1000) && now - last_draw >= 200))) {
      Draw(big, pg, st, stale);
      Downsample(big, rgba);
      if (win_vis) win.Present(rgba);
      if (dash_vis) dash.Show(rgba);
      last_draw = now;
      g_dirty = g_ui = first = false;
    }
    SleepMs(30);
  }
  if (vr_up) vr::VR_Shutdown();
  if (lock >= 0) close(lock);
  return 0;
}
