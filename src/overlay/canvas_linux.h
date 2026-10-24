// Linux stand-in for the GDI canvas the dashboard page is drawn on: a 0x00RRGGBB pixel buffer, filled shapes (the
// page draws at 2x and downsamples, which anti-aliases them) and text through stb_truetype with fonts fontconfig picks.
#pragma once
#include <stdio.h>
#include <string.h>
#include <wchar.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <tuple>
#include <string>
#include <vector>

#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_STATIC
#include "stb_truetype.h"

#ifndef FW_NORMAL
#define FW_NORMAL 400
#define FW_SEMIBOLD 600
#endif

struct Canvas {
  int w, h;
  std::vector<uint32_t> buf;
  uint32_t *px;
  Canvas(int w_, int h_) : w(w_), h(h_), buf((size_t)w_ * h_), px(buf.data()) {}
};

namespace fonts {

struct Face {
  std::vector<uint8_t> data;
  stbtt_fontinfo info{};
  bool ok = false;
};

inline bool ReadAll(const std::string &path, std::vector<uint8_t> &out) {
  FILE *f = fopen(path.c_str(), "rb");
  if (!f) return false;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (n <= 0) { fclose(f); return false; }
  out.resize((size_t)n);
  bool ok = fread(out.data(), 1, (size_t)n, f) == (size_t)n;
  fclose(f);
  return ok;
}

inline bool Open(Face &fc, const std::string &path, int index) {
  if (path.empty() || !ReadAll(path, fc.data)) return false;
  int off = stbtt_GetFontOffsetForIndex(fc.data.data(), index);
  if (off < 0 || !stbtt_InitFont(&fc.info, fc.data.data(), off)) return false;
  return fc.ok = true;
}

// fontconfig's pick for a pattern: "file\nindex"
inline bool Match(const char *pattern, std::string &file, int &index) {
  std::string cmd = std::string("fc-match -f '%{file}\\n%{index}' '") + pattern + "' 2>/dev/null";
  FILE *p = popen(cmd.c_str(), "r");
  if (!p) return false;
  char line[1024] = "", idx[32] = "0";
  bool ok = fgets(line, sizeof line, p) != nullptr;
  if (ok && !fgets(idx, sizeof idx, p)) strcpy(idx, "0");
  pclose(p);
  if (!ok) return false;
  file = line;
  while (!file.empty() && (file.back() == '\n' || file.back() == '\r')) file.pop_back();
  index = atoi(idx);
  return !file.empty();
}

enum Kind { kRegular, kSemibold, kMono };

inline Face &Get(Kind k) {
  static std::unique_ptr<Face> faces[3];
  if (faces[k]) return *faces[k];
  faces[k] = std::make_unique<Face>();
  Face &fc = *faces[k];
  static const char *const pattern[] = {"sans-serif:weight=regular", "sans-serif:weight=semibold", "monospace"};
  static const char *const fallback[3][6] = {
      {"/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/TTF/DejaVuSans.ttf",
       "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf", "/usr/share/fonts/noto/NotoSans-Regular.ttf",
       "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf", "/usr/share/fonts/liberation/LiberationSans-Regular.ttf"},
      {"/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
       "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans-Bold.ttf", "/usr/share/fonts/noto/NotoSans-SemiBold.ttf",
       "/usr/share/fonts/truetype/noto/NotoSans-SemiBold.ttf", "/usr/share/fonts/liberation/LiberationSans-Bold.ttf"},
      {"/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
       "/usr/share/fonts/dejavu-sans-mono-fonts/DejaVuSansMono.ttf", "/usr/share/fonts/noto/NotoSansMono-Regular.ttf",
       "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
       "/usr/share/fonts/liberation/LiberationMono-Regular.ttf"}};
  std::string file;
  int index = 0;
  if (Match(pattern[k], file, index) && Open(fc, file, index)) return fc;
  for (const char *f : fallback[k])
    if (Open(fc, f, 0)) return fc;
  if (k != kRegular) {  // whatever there is
    Face &r = Get(kRegular);
    if (r.ok) { fc.data = r.data; fc.ok = stbtt_InitFont(&fc.info, fc.data.data(), stbtt_GetFontOffsetForIndex(fc.data.data(), 0)); }
  }
  return fc;
}

}  // namespace fonts

struct Font {
  int px;  // em size on the 2x canvas
  fonts::Kind kind;
  Font(int size, int weight, const wchar_t *face = L"") {
    px = size * 2;
    kind = wcsstr(face, L"Mono") ? fonts::kMono : weight >= FW_SEMIBOLD ? fonts::kSemibold : fonts::kRegular;
  }
};

struct Painter {
  static const int SSc = 2;  // the page's supersampling (overlay_main.cpp's SS)
  Canvas &c;
  explicit Painter(Canvas &cv) : c(cv) {}

  void Fill(int x0, int y0, int x1, int y1, uint32_t color, int r) {  // canvas px, [x0, x1)
    x0 = std::max(x0, 0); y0 = std::max(y0, 0);
    int cx1 = std::min(x1, c.w), cy1 = std::min(y1, c.h);
    r = std::min({r, (x1 - x0) / 2, (y1 - y0) / 2});
    for (int y = y0; y < cy1; y++) {
      for (int x = x0; x < cx1; x++) {
        if (r > 0) {
          double px = x + 0.5, py = y + 0.5, dx = 0, dy = 0;
          if (px < x0 + r) dx = x0 + r - px;
          else if (px > x1 - r) dx = px - (x1 - r);
          if (py < y0 + r) dy = y0 + r - py;
          else if (py > y1 - r) dy = py - (y1 - r);
          if (dx * dx + dy * dy > (double)r * r) continue;
        }
        c.px[(size_t)y * c.w + x] = color;
      }
    }
  }
  void Rect(int x, int y, int w, int h, uint32_t color, int r = 0) {
    Fill(x * SSc, y * SSc, (x + w) * SSc, (y + h) * SSc, color, r * SSc);
  }
  void Outline(int x, int y, int w, int h, uint32_t color, int r, int width = 1) {
    int t = width * SSc, X0 = x * SSc, Y0 = y * SSc, X1 = (x + w) * SSc, Y1 = (y + h) * SSc, R = r * SSc;
    for (int yy = std::max(Y0, 0); yy < std::min(Y1, c.h); yy++)
      for (int xx = std::max(X0, 0); xx < std::min(X1, c.w); xx++) {
        auto inside = [&](int a0, int b0, int a1, int b1, int rr) {
          double px = xx + 0.5, py = yy + 0.5, dx = 0, dy = 0;
          if (px < a0 || px >= a1 || py < b0 || py >= b1) return false;
          if (px < a0 + rr) dx = a0 + rr - px; else if (px > a1 - rr) dx = px - (a1 - rr);
          if (py < b0 + rr) dy = b0 + rr - py; else if (py > b1 - rr) dy = py - (b1 - rr);
          return dx * dx + dy * dy <= (double)rr * rr;
        };
        if (inside(X0, Y0, X1, Y1, R) && !inside(X0 + t, Y0 + t, X1 - t, Y1 - t, std::max(R - t, 0)))
          c.px[(size_t)yy * c.w + xx] = color;
      }
  }
  void Dot(int cx, int cy, int r, uint32_t color) { Rect(cx - r, cy - r, 2 * r, 2 * r, color, r); }
  // a straight stroke on the canvas (canvas px), width w
  void LineC(double x0, double y0, double x1, double y1, double w, uint32_t color) {
    int ax = (int)std::floor(std::min(x0, x1) - w), bx = (int)std::ceil(std::max(x0, x1) + w);
    int ay = (int)std::floor(std::min(y0, y1) - w), by = (int)std::ceil(std::max(y0, y1) + w);
    double dx = x1 - x0, dy = y1 - y0, ll = dx * dx + dy * dy;
    for (int y = std::max(ay, 0); y < std::min(by, c.h); y++)
      for (int x = std::max(ax, 0); x < std::min(bx, c.w); x++) {
        double px = x + 0.5 - x0, py = y + 0.5 - y0;
        double t = ll > 0 ? std::max(0.0, std::min(1.0, (px * dx + py * dy) / ll)) : 0;
        double ex = px - t * dx, ey = py - t * dy;
        if (ex * ex + ey * ey <= w * w / 4) c.px[(size_t)y * c.w + x] = color;
      }
  }

  // text: the face's metrics at the font's em size
  struct Glyph { int w = 0, h = 0, xo = 0, yo = 0; std::vector<uint8_t> a; };
  static Glyph &Bitmap(const fonts::Face &fc, int kind, int px, int cp) {
    static std::map<std::tuple<int, int, int>, Glyph> cache;
    auto key = std::make_tuple(kind, px, cp);
    auto it = cache.find(key);
    if (it != cache.end()) return it->second;
    Glyph &g = cache[key];
    if (!fc.ok) return g;
    float s = stbtt_ScaleForMappingEmToPixels(&fc.info, (float)px);
    int x0, y0, x1, y1;
    stbtt_GetCodepointBitmapBox(&fc.info, cp, s, s, &x0, &y0, &x1, &y1);
    g.w = x1 - x0; g.h = y1 - y0; g.xo = x0; g.yo = y0;
    if (g.w > 0 && g.h > 0) {
      g.a.resize((size_t)g.w * g.h);
      stbtt_MakeCodepointBitmap(&fc.info, g.a.data(), g.w, g.h, g.w, s, s, cp);
    }
    return g;
  }
  // the face that has the character (the font's, else the regular one)
  static const fonts::Face &FaceFor(const Font &f, int cp, int &kind) {
    const fonts::Face &a = fonts::Get(f.kind);
    kind = f.kind;
    if (!a.ok || stbtt_FindGlyphIndex(&a.info, cp)) return a;
    for (int k : {fonts::kRegular, fonts::kSemibold, fonts::kMono}) {
      const fonts::Face &b = fonts::Get((fonts::Kind)k);
      if (b.ok && stbtt_FindGlyphIndex(&b.info, cp)) { kind = k; return b; }
    }
    return a;
  }
  static double Advance(const Font &f, int cp, int next) {
    int kind;
    const fonts::Face &fc = FaceFor(f, cp, kind);
    if (!fc.ok) return 0;
    float s = stbtt_ScaleForMappingEmToPixels(&fc.info, (float)f.px);
    int adv, lsb;
    stbtt_GetCodepointHMetrics(&fc.info, cp, &adv, &lsb);
    double a = adv * s;
    if (next && kind == f.kind) a += stbtt_GetCodepointKernAdvance(&fc.info, cp, next) * s;
    return a;
  }
  // width on the canvas (2x px)
  static int MeasureC(const std::wstring &t, const Font &f) {
    double x = 0;
    for (size_t i = 0; i < t.size(); i++) x += Advance(f, (int)t[i], i + 1 < t.size() ? (int)t[i + 1] : 0);
    return (int)std::lround(x);
  }
  int Measure(const std::wstring &t, const Font &f) { return MeasureC(t, f); }

  void Blend(int x, int y, uint32_t color, unsigned a) {
    if (x < 0 || y < 0 || x >= c.w || y >= c.h || !a) return;
    uint32_t &d = c.px[(size_t)y * c.w + x];
    auto mix = [&](int sh) {
      unsigned s = (color >> sh) & 255, o = (d >> sh) & 255;
      return ((o * (255 - a) + s * a + 127) / 255) << sh;
    };
    d = mix(16) | mix(8) | mix(0);
  }

  // text; align 0 left, 1 center, 2 right (x is the anchor). returns the width; y is the top of the line, as GDI's
  int Text(int x, int y, const std::wstring &s, const Font &f, uint32_t color, int align = 0, int maxw = 0) {
    std::wstring t = s;
    int w = MeasureC(t, f);
    if (maxw > 0)
      while (w > maxw * SSc && t.size() > 2) {
        t = t.substr(0, t.size() - 2) + L"…";
        w = MeasureC(t, f);
      }
    const fonts::Face &main = fonts::Get(f.kind);
    if (!main.ok) return w / SSc;
    float sc = stbtt_ScaleForMappingEmToPixels(&main.info, (float)f.px);
    int asc, desc, gap;
    stbtt_GetFontVMetrics(&main.info, &asc, &desc, &gap);
    int16_t win_asc = 0;
    {  // GDI puts the top of the cell at y: the OS/2 winAscent above the baseline
      int a, d, l;
      if (stbtt_GetFontVMetricsOS2(&main.info, &a, &d, &l)) {
        uint32_t os2 = stbtt__find_table(main.info.data, main.info.fontstart, "OS/2");
        if (os2) win_asc = (int16_t)ttUSHORT(main.info.data + os2 + 74);
      }
    }
    double base = y * SSc + (win_asc > 0 ? win_asc : asc) * sc;
    double pen = x * SSc - (align == 1 ? w / 2 : align == 2 ? w : 0);
    for (size_t i = 0; i < t.size(); i++) {
      int cp = (int)t[i], kind;
      const fonts::Face &fc = FaceFor(f, cp, kind);
      Glyph &g = Bitmap(fc, kind, f.px, cp);
      int gx = (int)std::lround(pen) + g.xo, gy = (int)std::lround(base) + g.yo;
      for (int yy = 0; yy < g.h; yy++)
        for (int xx = 0; xx < g.w; xx++) Blend(gx + xx, gy + yy, color, g.a[(size_t)yy * g.w + xx]);
      pen += Advance(f, cp, i + 1 < t.size() ? (int)t[i + 1] : 0);
    }
    return w / SSc;
  }
};
