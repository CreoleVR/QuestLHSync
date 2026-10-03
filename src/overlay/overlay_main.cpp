// QuestLHSync.exe: the SteamVR dashboard page. Started by the driver each session, exits with SteamVR.
// Reads the driver's status from the shared memory (qlhs_status.h), draws it with GDI at 2x and downsamples
// (anti-aliased shapes and text), and sends the buttons back as commands.
//   QuestLHSync.exe --preview out.png [locked|acquiring|searching|nohmd]   renders a sample page, no VR
#include <windows.h>
#include <d3d11.h>
#include <dxgi.h>
#include <shellapi.h>
#include <tlhelp32.h>
#undef small  // rpcndr.h (via d3d11.h): "#define small char"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <string>
#include <vector>

#include <openvr.h>

#include "../common/qlhs_status.h"

static const int W = 1280, H = 800, SS = 2;  // page size, supersampling

// ---------------------------------------------------------------- canvas
struct Canvas {
  int w, h;
  HDC dc;
  HBITMAP bmp;
  uint32_t *px;
  Canvas(int w_, int h_) : w(w_), h(h_) {
    BITMAPINFO bi{};
    bi.bmiHeader.biSize = sizeof bi.bmiHeader;
    bi.bmiHeader.biWidth = w;
    bi.bmiHeader.biHeight = -h;
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    dc = CreateCompatibleDC(nullptr);
    bmp = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, (void **)&px, nullptr, 0);
    SelectObject(dc, bmp);
    SetBkMode(dc, TRANSPARENT);
  }
  ~Canvas() { DeleteDC(dc); DeleteObject(bmp); }
};

static COLORREF C(uint32_t rgb) { return RGB((rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255); }

namespace col {
const uint32_t bg = 0x14161b, card = 0x1d2027, card2 = 0x252932, line = 0x30343e, text = 0xeceef2, dim = 0x9aa1ad,
               faint = 0x6b7280, green = 0x3ccf6e, amber = 0xffb224, red = 0xff5f57, blue = 0x5b8cff, grey = 0x8e93a0;
}

struct Font {
  HFONT f;
  Font(int px, int weight, const wchar_t *face = L"Segoe UI") {
    f = CreateFontW(-px * SS, 0, 0, 0, weight, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                    ANTIALIASED_QUALITY, DEFAULT_PITCH, face);
  }
  ~Font() { DeleteObject(f); }
};

struct Painter {
  Canvas &c;
  explicit Painter(Canvas &cv) : c(cv) {}
  void Rect(int x, int y, int w, int h, uint32_t color, int r = 0) {
    HBRUSH b = CreateSolidBrush(C(color));
    HPEN p = CreatePen(PS_SOLID, 1, C(color));
    HGDIOBJ ob = SelectObject(c.dc, b), op = SelectObject(c.dc, p);
    if (r) RoundRect(c.dc, x * SS, y * SS, (x + w) * SS, (y + h) * SS, r * 2 * SS, r * 2 * SS);
    else Rectangle(c.dc, x * SS, y * SS, (x + w) * SS, (y + h) * SS);
    SelectObject(c.dc, ob); SelectObject(c.dc, op);
    DeleteObject(b); DeleteObject(p);
  }
  void Outline(int x, int y, int w, int h, uint32_t color, int r, int width = 1) {
    HPEN p = CreatePen(PS_SOLID, width * SS, C(color));
    HGDIOBJ ob = SelectObject(c.dc, GetStockObject(NULL_BRUSH)), op = SelectObject(c.dc, p);
    RoundRect(c.dc, x * SS, y * SS, (x + w) * SS, (y + h) * SS, r * 2 * SS, r * 2 * SS);
    SelectObject(c.dc, ob); SelectObject(c.dc, op);
    DeleteObject(p);
  }
  void Dot(int cx, int cy, int r, uint32_t color) { Rect(cx - r, cy - r, 2 * r, 2 * r, color, r); }
  // text; align 0 left, 1 center, 2 right (x is the anchor). returns the width
  int Text(int x, int y, const std::wstring &s, const Font &f, uint32_t color, int align = 0, int maxw = 0) {
    HGDIOBJ of = SelectObject(c.dc, f.f);
    SetTextColor(c.dc, C(color));
    SIZE sz;
    std::wstring t = s;
    GetTextExtentPoint32W(c.dc, t.c_str(), (int)t.size(), &sz);
    if (maxw > 0)
      while (sz.cx > maxw * SS && t.size() > 2) {
        t = t.substr(0, t.size() - 2) + L"…";
        GetTextExtentPoint32W(c.dc, t.c_str(), (int)t.size(), &sz);
      }
    int px = x * SS - (align == 1 ? sz.cx / 2 : align == 2 ? sz.cx : 0);
    TextOutW(c.dc, px, y * SS, t.c_str(), (int)t.size());
    SelectObject(c.dc, of);
    return sz.cx / SS;
  }
};

static std::wstring Wide(const char *s) {
  int n = MultiByteToWideChar(CP_UTF8, 0, s, -1, nullptr, 0);
  std::wstring w(n ? n - 1 : 0, L'\0');
  if (n > 1) MultiByteToWideChar(CP_UTF8, 0, s, -1, w.data(), n);
  return w;
}

static std::wstring F(const wchar_t *fmt, ...) {
  wchar_t b[512];
  va_list ap;
  va_start(ap, fmt);
  _vsnwprintf_s(b, _countof(b), _TRUNCATE, fmt, ap);
  va_end(ap);
  return b;
}

static std::wstring Ago(double s) {
  if (s < 0) return L"not yet";
  if (s < 1.5) return L"now";
  if (s < 90) return F(L"%.0f s ago", s);
  if (s < 5400) return F(L"%.0f min ago", s / 60);
  return F(L"%.1f h ago", s / 3600);
}

static std::wstring Dur(double s) {
  if (s < 90) return F(L"%.0f s", s);
  if (s < 5400) return F(L"%.0f min", s / 60);
  return F(L"%.1f h", s / 3600);
}

// ---------------------------------------------------------------- page
struct Button { int x, y, w, h, cmd; std::wstring label; bool primary; };

struct Page {
  std::vector<Button> buttons;
  int hover = -1;
};

static void StateText(const QlhsStatus &s, bool stale, std::wstring &title, std::wstring &sub, uint32_t &color) {
  if (stale) { title = L"Driver not running"; sub = L"The QuestLHSync driver isn't updating. Restart SteamVR."; color = col::red; return; }
  switch (s.state) {
    case QLHS_NO_HMD:
      title = L"Waiting for a Quest";
      sub = s.hmd[0] ? L"SteamVR's headset isn't a Quest Pro, 3 or 3S, so the lighthouse space is left alone."
                     : L"Start your streaming app (Link, Air Link, Virtual Desktop, ALVR, Steam Link, ...).";
      color = col::grey;
      break;
    case QLHS_SEARCHING:
      title = L"Looking for the headset";
      sub = L"No Quest with the QuestLHSync module answers on the network. Is it awake and on the same Wi-Fi?";
      color = col::amber;
      break;
    case QLHS_CONNECTING: title = L"Connecting"; sub = L"Found the headset, opening the camera stream."; color = col::amber; break;
    case QLHS_NO_CAMERAS:
      title = L"No camera frames";
      sub = L"Connected, but the tracking cameras send nothing. Put the headset on (or wake it) and check the module.";
      color = col::red;
      break;
    case QLHS_NO_STATIONS:
      title = L"Waiting for base stations";
      sub = L"SteamVR shows base stations only while a lighthouse device (tracker, controller) is on. Turn one on.";
      color = col::amber;
      break;
    case QLHS_ACQUIRING:
      title = L"Finding the base stations";
      sub = L"Look around the room so the cameras catch both base stations' flashes.";
      color = col::amber;
      break;
    case QLHS_LOCKED:
      title = s.paused ? L"Locked, corrections paused" : L"Locked";
      sub = s.paused ? L"Lighthouse devices stay where they are until you resume."
                     : L"Lighthouse devices follow the Quest's own tracking.";
      color = s.paused ? col::amber : col::green;
      break;
    default: title = L"Starting"; sub = L""; color = col::grey;
  }
}

static void Draw(Canvas &cv, Page &pg, const QlhsStatus &s, bool stale) {
  Painter p(cv);
  Font h1(40, FW_SEMIBOLD, L"Segoe UI Semibold"), h2(22, FW_SEMIBOLD, L"Segoe UI Semibold"), body(21, FW_NORMAL),
      small(17, FW_NORMAL), label(16, FW_SEMIBOLD, L"Segoe UI Semibold"), big(28, FW_SEMIBOLD, L"Segoe UI Semibold"),
      mono(17, FW_NORMAL, L"Cascadia Mono");
  p.Rect(0, 0, W, H, col::bg);

  // header
  int tw = p.Text(48, 34, L"QuestLHSync", h1, col::text);
  p.Text(48 + tw + 14, 50, L"by CreoleVR", h2, col::faint);
  p.Text(50, 88, L"Lighthouse tracking aligned to your Quest, from its own cameras", small, col::dim);
  std::wstring title, sub;
  uint32_t sc;
  StateText(s, stale, title, sub, sc);
  {
    Font pill(22, FW_SEMIBOLD, L"Segoe UI Semibold");
    HGDIOBJ of = SelectObject(cv.dc, pill.f);
    SIZE sz;
    GetTextExtentPoint32W(cv.dc, title.c_str(), (int)title.size(), &sz);
    SelectObject(cv.dc, of);
    int tw = sz.cx / SS, pw = tw + 64, px = W - 48 - pw;
    p.Rect(px, 40, pw, 48, col::card2, 24);
    p.Dot(px + 26, 64, 7, sc);
    p.Text(px + 44, 50, title, pill, col::text);
  }

  // status line
  p.Rect(48, 124, W - 96, 52, col::card, 12);
  p.Rect(48, 124, 6, 52, sc, 3);
  p.Text(70, 137, sub, body, col::text, 0, W - 140);

  // cards
  int top = 196, ch = 300, gap = 20, cw = (W - 96 - 2 * gap) / 3;
  int x1 = 48, x2 = x1 + cw + gap, x3 = x2 + cw + gap;
  for (int x : {x1, x2, x3}) p.Rect(x, top, cw, ch, col::card, 14);

  auto row = [&](int x, int y, const wchar_t *k, const std::wstring &v, uint32_t vc = col::text) {
    p.Text(x + 22, y, k, small, col::dim);
    p.Text(x + cw - 22, y, v, small, vc, 2, cw - 150);
  };

  // headset card
  p.Text(x1 + 22, top + 18, L"HEADSET", label, col::faint);
  p.Text(x1 + 22, top + 44, s.headset[0] ? Wide(s.headset) : L"—", big, col::text, 0, cw - 44);
  row(x1, top + 100, L"Address", s.headset_addr[0] ? Wide(s.headset_addr) : L"—");
  row(x1, top + 134, L"Cameras", s.cam_fps > 0.5 ? F(L"%.0f frames/s", s.cam_fps) : L"—",
      s.cam_fps > 0.5 || s.state < QLHS_NO_CAMERAS ? col::text : col::red);
  row(x1, top + 168, L"Round trip", s.rtt_ms > 0 ? F(L"%.1f ms", s.rtt_ms) : L"—");
  row(x1, top + 202, L"Streamer", s.hmd_system[0] ? Wide(s.hmd_system) : L"—");
  row(x1, top + 236, L"Pose timing",
      s.headset_addr[0] ? F(L"%.1f ms %s", s.expo_ms, s.expo_learned ? L"(learned)" : L"(learning)") : L"—",
      s.expo_learned || !s.headset_addr[0] ? col::text : col::amber);

  // base stations card
  p.Text(x2 + 22, top + 18, L"BASE STATIONS", label, col::faint);
  if (s.nst == 0) {
    p.Text(x2 + 22, top + 52, L"None in SteamVR yet", body, col::dim);
    p.Text(x2 + 22, top + 84, L"They show up while a tracker or", small, col::faint);
    p.Text(x2 + 22, top + 108, L"controller is switched on.", small, col::faint);
  }
  for (int i = 0; i < std::min(s.nst, 4); i++) {
    const QlhsStation &st = s.st[i];
    int y = top + 50 + i * 60;
    bool fresh = st.last_seen >= 0 && st.last_seen < 15;
    p.Dot(x2 + 30, y + 15, 6, fresh ? col::green : st.last_seen >= 0 ? col::amber : col::faint);
    p.Text(x2 + 46, y, Wide(st.serial), body, col::text, 0, cw - 150);
    std::wstring role = st.anchor ? L"anchor" : st.measured ? L"measured" : L"";
    if (!role.empty()) p.Text(x2 + cw - 22, y + 3, role, small, col::blue, 2);
    std::wstring info = L"seen " + Ago(st.last_seen) + F(L"  ·  %d sightings", st.support);
    if (st.dist > 0) info += F(L"  ·  %.1f m", st.dist);
    p.Text(x2 + 46, y + 28, info, small, col::dim, 0, cw - 68);
  }

  // alignment card
  p.Text(x3 + 22, top + 18, L"ALIGNMENT", label, col::faint);
  bool has = s.locked || s.nfit > 0 || s.yaw_deg != 0;
  p.Text(x3 + 22, top + 44, has ? F(L"%.2f°", s.med_deg >= 0 ? s.med_deg : 0.0) : L"—", big,
         s.med_deg >= 0 && s.med_deg < 0.3 ? col::green : col::text);
  if (has) p.Text(x3 + 22 + 110, top + 56, L"median sighting error", small, col::dim);
  row(x3, top + 100, L"Yaw", has ? F(L"%+.2f°", s.yaw_deg) : L"—");
  row(x3, top + 134, L"Offset", has ? F(L"%.2f, %.2f, %.2f m", s.t[0], s.t[1], s.t[2]) : L"—");
  row(x3, top + 168, L"Sightings used", has ? F(L"%d", s.nfit) : L"—");
  row(x3, top + 202, L"Since acquired", s.locked_for >= 0 ? Dur(s.locked_for) : L"—");
  row(x3, top + 236, L"Settling", s.lag_cm > 0.05 ? F(L"%.1f cm to go", s.lag_cm) : has ? L"done" : L"—");

  // log
  int ly = top + ch + 20, lh = 150;
  p.Rect(48, ly, W - 96, lh, col::card, 14);
  p.Text(70, ly + 16, L"LOG", label, col::faint);
  uint32_t n = s.nlog;
  int shown = 0;
  for (uint32_t i = 0; i < 5 && i < n; i++) {
    uint32_t k = n - 1 - i;
    p.Text(70, ly + 44 + 20 * i, Wide(s.log[k % 8]), mono, i == 0 ? col::text : col::dim, 0, W - 150);
    shown++;
  }
  if (!shown) p.Text(70, ly + 44, L"—", mono, col::dim);

  // buttons
  pg.buttons.clear();
  int by = H - 66, bh = 46;
  pg.buttons.push_back({48, by, 200, bh, QLHS_CMD_REACQUIRE, L"Re-acquire", false});
  pg.buttons.push_back({264, by, 240, bh, s.paused ? QLHS_CMD_RESUME : QLHS_CMD_PAUSE,
                        s.paused ? L"Resume corrections" : L"Pause corrections", s.paused != 0});
  pg.buttons.push_back({520, by, 210, bh, s.recording ? QLHS_CMD_RECORD_OFF : QLHS_CMD_RECORD_ON,
                        s.recording ? L"Stop recording" : L"Record session", false});
  for (size_t i = 0; i < pg.buttons.size(); i++) {
    auto &b = pg.buttons[i];
    bool hov = (int)i == pg.hover;
    uint32_t fill = b.primary ? col::blue : hov ? col::line : col::card2;
    p.Rect(b.x, b.y, b.w, b.h, fill, 10);
    if (b.cmd == QLHS_CMD_RECORD_OFF) p.Dot(b.x + 26, b.y + bh / 2, 6, col::red);
    p.Text(b.x + b.w / 2 + (b.cmd == QLHS_CMD_RECORD_OFF ? 10 : 0), b.y + 11, b.label, h2, col::text, 1);
  }
  p.Text(W - 48, by + 14, L"Settings: steamvr.vrsettings → driver_questlhsync", small, col::faint, 2);
}

// 2x2 box filter + BGRX -> RGBA (alpha 255)
static void Downsample(const Canvas &big, std::vector<uint8_t> &rgba) {
  rgba.resize((size_t)W * H * 4);
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      unsigned r = 0, g = 0, b = 0;
      for (int dy = 0; dy < SS; dy++)
        for (int dx = 0; dx < SS; dx++) {
          uint32_t v = big.px[(size_t)(y * SS + dy) * big.w + x * SS + dx];
          r += (v >> 16) & 255; g += (v >> 8) & 255; b += v & 255;
        }
      uint8_t *o = &rgba[((size_t)y * W + x) * 4];
      o[0] = (uint8_t)(r / (SS * SS)); o[1] = (uint8_t)(g / (SS * SS)); o[2] = (uint8_t)(b / (SS * SS)); o[3] = 255;
    }
}

// dashboard thumbnail: a base station's laser fan reaching a headset
static void DrawIcon(std::vector<uint8_t> &rgba, int N) {
  Canvas cv(N * SS, N * SS);
  Painter p(cv);
  auto S = [&](int v) { return v * N / 256; };
  p.Rect(0, 0, N, N, col::bg);
  p.Rect(S(16), S(16), S(224), S(224), col::card2, S(48));
  p.Rect(S(54), S(66), S(70), S(70), col::text, S(18));      // base station
  p.Dot(S(89), S(101), S(20), col::bg);
  p.Dot(S(89), S(101), S(10), col::blue);
  HPEN pen = CreatePen(PS_SOLID, S(8) * SS, C(col::blue));
  HGDIOBJ op = SelectObject(cv.dc, pen);
  for (int i = 0; i < 3; i++) {  // laser lines
    MoveToEx(cv.dc, S(118) * SS, S(112 + i * 14) * SS, nullptr);
    LineTo(cv.dc, S(170) * SS, S(140 + i * 18) * SS);
  }
  SelectObject(cv.dc, op);
  DeleteObject(pen);
  p.Rect(S(140), S(150), S(86), S(52), col::green, S(20));   // headset visor
  p.Dot(S(166), S(176), S(9), col::bg);
  p.Dot(S(200), S(176), S(9), col::bg);
  rgba.resize((size_t)N * N * 4);
  for (int y = 0; y < N; y++)
    for (int x = 0; x < N; x++) {
      unsigned r = 0, g = 0, b = 0;
      for (int dy = 0; dy < SS; dy++)
        for (int dx = 0; dx < SS; dx++) {
          uint32_t v = cv.px[(size_t)(y * SS + dy) * cv.w + x * SS + dx];
          r += (v >> 16) & 255; g += (v >> 8) & 255; b += v & 255;
        }
      uint8_t *o = &rgba[((size_t)y * N + x) * 4];
      o[0] = (uint8_t)(r / 4); o[1] = (uint8_t)(g / 4); o[2] = (uint8_t)(b / 4); o[3] = 255;
    }
}

// ---------------------------------------------------------------- PNG (stored deflate) for --preview
static uint32_t Crc(const uint8_t *d, size_t n, uint32_t c = 0xFFFFFFFFu) {
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
  }
  return c;
}

static bool WritePng(const char *path, const std::vector<uint8_t> &rgba, int w, int h) {
  std::vector<uint8_t> raw;
  for (int y = 0; y < h; y++) {
    raw.push_back(0);
    raw.insert(raw.end(), rgba.begin() + (size_t)y * w * 4, rgba.begin() + (size_t)(y + 1) * w * 4);
  }
  std::vector<uint8_t> z = {0x78, 0x01};
  for (size_t o = 0; o < raw.size(); o += 65535) {
    size_t n = std::min<size_t>(65535, raw.size() - o);
    z.push_back((uint8_t)(o + n == raw.size()));
    z.push_back((uint8_t)(n & 255)); z.push_back((uint8_t)(n >> 8)); z.push_back((uint8_t)(~n & 255)); z.push_back((uint8_t)((~n >> 8) & 255));
    z.insert(z.end(), raw.begin() + o, raw.begin() + o + n);
  }
  uint32_t a = 1, b = 0;
  for (uint8_t v : raw) { a = (a + v) % 65521; b = (b + a) % 65521; }
  uint32_t ad = (b << 16) | a;
  for (int i = 3; i >= 0; i--) z.push_back((ad >> (8 * i)) & 255);
  FILE *f = fopen(path, "wb");
  if (!f) return false;
  auto be = [&](uint32_t v) { uint8_t q[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v}; fwrite(q, 1, 4, f); };
  auto chunk = [&](const char *t, const std::vector<uint8_t> &d) {
    be((uint32_t)d.size());
    std::vector<uint8_t> td(t, t + 4);
    td.insert(td.end(), d.begin(), d.end());
    fwrite(td.data(), 1, td.size(), f);
    be(Crc(td.data(), td.size()) ^ 0xFFFFFFFFu);
  };
  const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 13, 10, 26, 10};
  fwrite(sig, 1, 8, f);
  std::vector<uint8_t> ih = {(uint8_t)(w >> 24), (uint8_t)(w >> 16), (uint8_t)(w >> 8), (uint8_t)w,
                             (uint8_t)(h >> 24), (uint8_t)(h >> 16), (uint8_t)(h >> 8), (uint8_t)h, 8, 6, 0, 0, 0};
  chunk("IHDR", ih);
  chunk("IDAT", z);
  chunk("IEND", {});
  fclose(f);
  return true;
}

static QlhsStatus Sample(const char *kind) {
  QlhsStatus s{};
  s.magic = QLHS_MAGIC;
  s.version = QLHS_VERSION;
  strcpy(s.hmd, "Quest Pro (CreoleCast)");
  strcpy(s.hmd_system, "creolecast");
  strcpy(s.headset, "Quest Pro");
  strcpy(s.headset_addr, "192.168.1.50:47280");
  s.cam_fps = 75;
  s.rtt_ms = 4.2;
  s.expo_ms = 15.3;
  s.expo_learned = 1;
  s.nst = 2;
  strcpy(s.st[0].serial, "LHB-1A2B3C4D");
  s.st[0].anchor = 1; s.st[0].support = 432; s.st[0].last_seen = 0.4; s.st[0].dist = 3.4;
  strcpy(s.st[1].serial, "LHB-5E6F7A8B");
  s.st[1].measured = 1; s.st[1].support = 337; s.st[1].last_seen = 6.2; s.st[1].dist = 2.1;
  s.state = QLHS_LOCKED;
  s.locked = 1; s.cond = 1;
  s.yaw_deg = -5.76; s.t[0] = -1.071; s.t[1] = 2.124; s.t[2] = 2.432;
  s.med_deg = 0.14; s.nfit = 788; s.locked_for = 754; s.lag_cm = 0.3;
  const char *logs[] = {"04:53:10  seeded from saved stations: yaw -5.765 t [-1.0713, 2.1258, 2.4309] miss 0.0 cm",
                        "04:53:12  Quest Pro at 192.168.1.50: connected",
                        "04:53:12  camera calibration: online",
                        "04:53:41  timing for creolecast: 15.3 ms (best fit 15.3 ms on 212 fast-head sightings)",
                        "04:55:02  lighthouse frame: SteamVR's is 0.75 deg from the reference"};
  for (auto l : logs) { strcpy(s.log[s.nlog % 8], l); s.nlog++; }
  if (!strcmp(kind, "acquiring")) { s.state = QLHS_ACQUIRING; s.locked = 0; s.med_deg = -1; s.nfit = 0; s.yaw_deg = 0; s.locked_for = -1; s.st[1].last_seen = -1; s.st[1].support = 0; s.expo_learned = 0; s.expo_ms = 15; }
  if (!strcmp(kind, "searching")) { s.state = QLHS_SEARCHING; s.headset_addr[0] = 0; s.cam_fps = 0; s.rtt_ms = 0; s.locked = 0; s.nfit = 0; s.med_deg = -1; s.yaw_deg = 0; s.locked_for = -1; }
  if (!strcmp(kind, "nohmd")) { s.state = QLHS_NO_HMD; strcpy(s.hmd, "PlayStation VR2"); }
  if (!strcmp(kind, "paused")) s.paused = 1;
  return s;
}

// ---------------------------------------------------------------- main
static QlhsStatus *g_shm;

static bool ReadStatus(QlhsStatus &out) {
  if (!g_shm) {
    HANDLE m = OpenFileMappingW(FILE_MAP_ALL_ACCESS, FALSE, QLHS_SHM_NAME);
    if (!m) return false;
    g_shm = (QlhsStatus *)MapViewOfFile(m, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(QlhsStatus));
    if (!g_shm) return false;
  }
  for (int i = 0; i < 100; i++) {
    int32_t s = g_shm->seq;
    if (s & 1) { Sleep(0); continue; }
    MemoryBarrier();
    memcpy(&out, (const void *)g_shm, sizeof out);
    MemoryBarrier();
    if (g_shm->seq == s) return out.magic == QLHS_MAGIC && out.version == QLHS_VERSION;
  }
  return false;
}

static void SendCmd(int cmd) {
  if (!g_shm) return;
  g_shm->cmd = cmd;
  MemoryBarrier();
  InterlockedIncrement((volatile LONG *)&g_shm->cmd_seq);
}

// ---------------------------------------------------------------- desktop window: the same page, scaled to fit
static Canvas *g_page;
static Page *g_pg;
static bool g_dirty = true, g_ui = false;  // g_ui: hover/click, redraw now
static int g_down = -1;

static int HitAt(float mx, float my) {
  for (size_t i = 0; i < g_pg->buttons.size(); i++) {
    auto &b = g_pg->buttons[i];
    if (mx >= b.x && mx < b.x + b.w && my >= b.y && my < b.y + b.h) return (int)i;
  }
  return -1;
}

static int WinHit(HWND h, LPARAM l) {
  RECT r;
  GetClientRect(h, &r);
  if (r.right <= 0 || r.bottom <= 0) return -1;
  return HitAt((float)(short)LOWORD(l) * W / r.right, (float)(short)HIWORD(l) * H / r.bottom);
}

static LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM w, LPARAM l) {
  switch (m) {
    case WM_PAINT: {
      PAINTSTRUCT ps;
      HDC dc = BeginPaint(h, &ps);
      RECT r;
      GetClientRect(h, &r);
      SetStretchBltMode(dc, HALFTONE);
      SetBrushOrgEx(dc, 0, 0, nullptr);
      StretchBlt(dc, 0, 0, r.right, r.bottom, g_page->dc, 0, 0, g_page->w, g_page->h, SRCCOPY);
      EndPaint(h, &ps);
      return 0;
    }
    case WM_ERASEBKGND: return 1;
    case WM_SIZING: {  // keep the page's shape
      RECT wr, cr, *r = (RECT *)l;
      GetWindowRect(h, &wr);
      GetClientRect(h, &cr);
      int fw = (wr.right - wr.left) - cr.right, fh = (wr.bottom - wr.top) - cr.bottom;
      int cw = r->right - r->left - fw, chh = r->bottom - r->top - fh;
      if (w == WMSZ_TOP || w == WMSZ_BOTTOM) cw = chh * W / H;
      else chh = cw * H / W;
      if (w == WMSZ_TOP || w == WMSZ_TOPLEFT || w == WMSZ_TOPRIGHT) r->top = r->bottom - chh - fh;
      else r->bottom = r->top + chh + fh;
      if (w == WMSZ_LEFT || w == WMSZ_TOPLEFT || w == WMSZ_BOTTOMLEFT) r->left = r->right - cw - fw;
      else r->right = r->left + cw + fw;
      return TRUE;
    }
    case WM_SIZE: InvalidateRect(h, nullptr, FALSE); return 0;
    case WM_DPICHANGED: {
      RECT *r = (RECT *)l;
      SetWindowPos(h, nullptr, r->left, r->top, r->right - r->left, r->bottom - r->top, SWP_NOZORDER | SWP_NOACTIVATE);
      return 0;
    }
    case WM_MOUSEMOVE: {
      TRACKMOUSEEVENT t{sizeof t, TME_LEAVE, h, 0};
      TrackMouseEvent(&t);
      int hit = WinHit(h, l);
      if (hit != g_pg->hover) { g_pg->hover = hit; g_ui = true; }
      SetCursor(LoadCursor(nullptr, hit >= 0 ? IDC_HAND : IDC_ARROW));
      return 0;
    }
    case WM_MOUSELEAVE:
      if (g_pg->hover >= 0) { g_pg->hover = -1; g_ui = true; }
      return 0;
    case WM_SETCURSOR:
      if (LOWORD(l) == HTCLIENT) return TRUE;  // WM_MOUSEMOVE sets it
      break;
    case WM_LBUTTONDOWN: g_down = WinHit(h, l); SetCapture(h); return 0;
    case WM_LBUTTONUP: {
      ReleaseCapture();
      int hit = WinHit(h, l);
      if (hit >= 0 && hit == g_down) { SendCmd(g_pg->buttons[hit].cmd); g_ui = true; }
      g_down = -1;
      return 0;
    }
    case WM_CLOSE: ShowWindow(h, SW_HIDE); return 0;  // the dashboard page stays; the window is back next SteamVR start
  }
  return DefWindowProcW(h, m, w, l);
}

static HICON MakeIcon(int n) {
  std::vector<uint8_t> rgba;
  DrawIcon(rgba, n);
  std::vector<uint32_t> bgra((size_t)n * n);
  for (size_t i = 0; i < bgra.size(); i++)
    bgra[i] = 0xFF000000u | (rgba[i * 4] << 16) | (rgba[i * 4 + 1] << 8) | rgba[i * 4 + 2];
  std::vector<uint8_t> mask((size_t)((n + 15) / 16) * 2 * n, 0);
  ICONINFO ii{TRUE, 0, 0, CreateBitmap(n, n, 1, 1, mask.data()), CreateBitmap(n, n, 1, 32, bgra.data())};
  HICON ic = CreateIconIndirect(&ii);
  DeleteObject(ii.hbmMask);
  DeleteObject(ii.hbmColor);
  return ic;
}

static HWND MakeWindow(HINSTANCE inst) {
  WNDCLASSEXW wc{sizeof wc};
  wc.lpfnWndProc = WndProc;
  wc.hInstance = inst;
  wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
  wc.hIcon = MakeIcon(64);
  wc.hIconSm = MakeIcon(32);
  wc.lpszClassName = L"QuestLHSyncWindow";
  RegisterClassExW(&wc);
  UINT dpi = GetDpiForSystem();
  RECT r{0, 0, MulDiv(W * 3 / 4, dpi, 96), MulDiv(H * 3 / 4, dpi, 96)};
  AdjustWindowRectExForDpi(&r, WS_OVERLAPPEDWINDOW, FALSE, 0, dpi);
  HWND h = CreateWindowExW(0, wc.lpszClassName, L"QuestLHSync by CreoleVR", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT,
                           CW_USEDEFAULT, r.right - r.left, r.bottom - r.top, nullptr, nullptr, inst, nullptr);
  if (h) ShowWindow(h, SW_SHOWNOACTIVATE);  // SteamVR may be starting under a game: don't take the focus
  return h;
}

// ---------------------------------------------------------------- SteamVR dashboard page
struct Dash {
  vr::VROverlayHandle_t main = vr::k_ulOverlayHandleInvalid, thumb = vr::k_ulOverlayHandleInvalid;
  // the page goes up as a D3D11 texture, two of them in turn: SetOverlayRaw makes a new texture on every call and
  // the page blinks while SteamVR swaps it in
  ID3D11Device *dev = nullptr;
  ID3D11DeviceContext *ctx = nullptr;
  ID3D11Texture2D *tex[2] = {};
  int flip = 0;

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
    int32_t adapter = -1;
    vr::VRSystem()->GetDXGIOutputInfo(&adapter);
    IDXGIFactory1 *fac = nullptr;
    IDXGIAdapter1 *ad = nullptr;
    if (adapter >= 0 && SUCCEEDED(CreateDXGIFactory1(__uuidof(IDXGIFactory1), (void **)&fac))) fac->EnumAdapters1(adapter, &ad);
    D3D11CreateDevice(ad, ad ? D3D_DRIVER_TYPE_UNKNOWN : D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0,
                      D3D11_SDK_VERSION, &dev, nullptr, &ctx);
    if (ad) ad->Release();
    if (fac) fac->Release();
    D3D11_TEXTURE2D_DESC td{};
    td.Width = W;
    td.Height = H;
    td.MipLevels = td.ArraySize = 1;
    td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    td.SampleDesc.Count = 1;
    td.Usage = D3D11_USAGE_DEFAULT;
    td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    for (auto &t : tex)
      if (dev) dev->CreateTexture2D(&td, nullptr, &t);
    return true;
  }
  void Show(std::vector<uint8_t> &rgba) {
    if (tex[0] && tex[1]) {
      ID3D11Texture2D *t = tex[flip ^= 1];
      ctx->UpdateSubresource(t, 0, nullptr, rgba.data(), W * 4, 0);
      ctx->Flush();
      vr::Texture_t vt{t, vr::TextureType_DirectX, vr::ColorSpace_Gamma};
      vr::VROverlay()->SetOverlayTexture(main, &vt);
    } else {
      vr::VROverlay()->SetOverlayRaw(main, rgba.data(), W, H, 4);
    }
  }
  ~Dash() {
    for (auto t : tex)
      if (t) t->Release();
    if (ctx) ctx->Release();
    if (dev) dev->Release();
  }
};

int WINAPI wWinMain(HINSTANCE inst, HINSTANCE, PWSTR, int) {
  int argc = 0;
  LPWSTR *argv = CommandLineToArgvW(GetCommandLineW(), &argc);
  if (argc >= 3 && !wcscmp(argv[1], L"--preview")) {
    char path[MAX_PATH], kind[32] = "locked";
    WideCharToMultiByte(CP_UTF8, 0, argv[2], -1, path, sizeof path, nullptr, nullptr);
    if (argc >= 4) WideCharToMultiByte(CP_UTF8, 0, argv[3], -1, kind, sizeof kind, nullptr, nullptr);
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

  HANDLE mutex = CreateMutexW(nullptr, TRUE, QLHS_OVERLAY_MUTEX);
  if (GetLastError() == ERROR_ALREADY_EXISTS) return 0;
  SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  // The driver starts us while SteamVR is still coming up, maybe with no headset yet (it can connect much later).
  // We live as long as vrserver runs; the dashboard page joins once the driver reports a headset (VR_Init without
  // one is refused, and would start SteamVR if it weren't running).
  auto server_up = [] {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return true;
    PROCESSENTRY32W pe{sizeof pe};
    bool up = false;
    for (BOOL ok = Process32FirstW(snap, &pe); ok && !up; ok = Process32NextW(snap, &pe))
      up = !_wcsicmp(pe.szExeFile, L"vrserver.exe");
    CloseHandle(snap);
    return up;
  };

  Canvas big(W * SS, H * SS);
  Page pg;
  g_page = &big;
  g_pg = &pg;
  QlhsStatus st{}, prev{};
  ReadStatus(st);
  Draw(big, pg, st, false);
  HWND win = MakeWindow(inst);
  Dash dash;
  bool vr_up = false, quit = false;
  std::vector<uint8_t> rgba;
  double last_upd = 0;
  DWORD last_change = GetTickCount(), last_draw = 0, next_try = 0, last_check = 0;
  int down = -1;
  while (!quit) {
    MSG msg;
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
      TranslateMessage(&msg);
      DispatchMessageW(&msg);
    }
    DWORD now = GetTickCount();
    if (!vr_up && now - last_check >= 1000) {
      last_check = now;
      if (!server_up()) break;
    }
    bool have = ReadStatus(st);
    if (!have) memset(&st, 0, sizeof st);
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
    bool win_vis = win && IsWindowVisible(win) && !IsIconic(win);
    bool dash_vis = vr_up && vr::VROverlay()->IsOverlayVisible(dash.main);
    if ((win_vis || dash_vis) && (g_ui || ((g_dirty || now - last_draw > 1000) && now - last_draw >= 200))) {
      Draw(big, pg, st, stale);
      if (win_vis) InvalidateRect(win, nullptr, FALSE);
      if (dash_vis) {
        Downsample(big, rgba);
        dash.Show(rgba);
      }
      last_draw = now;
      g_dirty = g_ui = false;
    }
    MsgWaitForMultipleObjects(0, nullptr, FALSE, 30, QS_ALLINPUT);
  }
  if (vr_up) vr::VR_Shutdown();
  if (win) DestroyWindow(win);
  ReleaseMutex(mutex);
  return 0;
}
