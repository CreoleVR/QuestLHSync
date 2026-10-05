// QuestLHSync-steamvr-installer.exe: installs and updates the SteamVR driver from the latest GitHub release.
// Looks like the dashboard page (same colours, fonts and layout), drawn with GDI at the screen's own resolution. Downloads
// releases/download/<tag>/QuestLHSync-steamvr-<ver>.zip with WinHTTP, unpacks its questlhsync/ folder to
// %LOCALAPPDATA%\QuestLHSync\questlhsync, registers it with SteamVR (vrpathreg) and records the installed tag in
// %LOCALAPPDATA%\QuestLHSync\installed.json so it can offer updates.
#include <windows.h>
#include <dwmapi.h>
#include <tlhelp32.h>
#include <winhttp.h>
#undef small  // rpcndr.h (via windows.h): "#define small char"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdarg>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

static const int W = 960, H = 660;  // page size at 96 dpi
static double g_k = 1;                      // dpi scale
static const wchar_t *RELEASES = L"https://github.com/CreoleVR/QuestLHSync/releases";

// ---------------------------------------------------------------- drawing (same look as the dashboard page)
struct Canvas {
  int w, h;
  HDC dc;
  HBITMAP bmp;
  Canvas(int w_, int h_) : w(w_), h(h_) {
    BITMAPINFO bi{};
    bi.bmiHeader.biSize = sizeof bi.bmiHeader;
    bi.bmiHeader.biWidth = w;
    bi.bmiHeader.biHeight = -h;
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    void *px;
    dc = CreateCompatibleDC(nullptr);
    bmp = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &px, nullptr, 0);
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
    f = CreateFontW(-(int)lround(px * g_k), 0, 0, 0, weight, 0, 0, 0, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                    CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, face);
  }
  ~Font() { DeleteObject(f); }
};

struct Painter {
  Canvas &c;
  explicit Painter(Canvas &cv) : c(cv) {}
  static int U(int v) { return (int)lround(v * g_k); }
  void Rect(int x, int y, int w, int h, uint32_t color, int r = 0) {
    HBRUSH b = CreateSolidBrush(C(color));
    HPEN p = CreatePen(PS_SOLID, 1, C(color));
    HGDIOBJ ob = SelectObject(c.dc, b), op = SelectObject(c.dc, p);
    if (r) RoundRect(c.dc, U(x), U(y), U(x + w), U(y + h), U(r) * 2, U(r) * 2);
    else Rectangle(c.dc, U(x), U(y), U(x + w), U(y + h));
    SelectObject(c.dc, ob); SelectObject(c.dc, op);
    DeleteObject(b); DeleteObject(p);
  }
  void Dot(int cx, int cy, int r, uint32_t color) { Rect(cx - r, cy - r, 2 * r, 2 * r, color, r); }
  // text; align 0 left, 1 center, 2 right (x is the anchor). returns the width in page pixels
  int Text(int x, int y, const std::wstring &s, const Font &f, uint32_t color, int align = 0, int maxw = 0) {
    HGDIOBJ of = SelectObject(c.dc, f.f);
    SetTextColor(c.dc, C(color));
    SIZE sz;
    std::wstring t = s;
    GetTextExtentPoint32W(c.dc, t.c_str(), (int)t.size(), &sz);
    if (maxw > 0)
      while (sz.cx > U(maxw) && t.size() > 2) {
        t = t.substr(0, t.size() - 2) + L"…";
        GetTextExtentPoint32W(c.dc, t.c_str(), (int)t.size(), &sz);
      }
    int px = U(x) - (align == 1 ? sz.cx / 2 : align == 2 ? sz.cx : 0);
    TextOutW(c.dc, px, U(y), t.c_str(), (int)t.size());
    SelectObject(c.dc, of);
    return (int)(sz.cx / g_k);
  }
};

// ---------------------------------------------------------------- strings
static std::wstring Wide(const std::string &s) {
  if (s.empty()) return L"";
  int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), nullptr, 0);
  std::wstring w(n, L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), w.data(), n);
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

struct Fail { std::wstring msg; };

static std::vector<int> VKey(const std::wstring &v) {
  std::vector<int> k;
  for (size_t i = 0; i < v.size();) {
    if (iswdigit(v[i])) {
      int n = 0;
      while (i < v.size() && iswdigit(v[i])) n = n * 10 + (v[i++] - L'0');
      k.push_back(n);
    } else i++;
  }
  return k;
}

static bool Newer(const std::wstring &a, const std::wstring &b) {  // a newer than b
  auto x = VKey(a), y = VKey(b);
  size_t n = std::max(x.size(), y.size());
  x.resize(n); y.resize(n);
  return x > y;
}

// ---------------------------------------------------------------- inflate (RFC 1951, after zlib's puff.c)
namespace inflate {
struct Huff { short count[16]; short symbol[288]; };

struct State {
  const uint8_t *in; size_t inlen, incnt = 0;
  uint32_t bitbuf = 0; int bitcnt = 0;
  uint8_t *out; size_t outlen, outcnt = 0;

  int Bits(int need) {
    uint32_t val = bitbuf;
    while (bitcnt < need) {
      if (incnt == inlen) throw Fail{L"release zip is truncated"};
      val |= (uint32_t)in[incnt++] << bitcnt;
      bitcnt += 8;
    }
    bitbuf = val >> need;
    bitcnt -= need;
    return (int)(val & ((1u << need) - 1));
  }
};

static int Construct(Huff &h, const short *length, int n) {
  for (int l = 0; l <= 15; l++) h.count[l] = 0;
  for (int s = 0; s < n; s++) h.count[length[s]]++;
  if (h.count[0] == n) return 0;
  int left = 1;
  for (int l = 1; l <= 15; l++) {
    left <<= 1;
    left -= h.count[l];
    if (left < 0) return left;
  }
  short offs[16];
  offs[1] = 0;
  for (int l = 1; l < 15; l++) offs[l + 1] = offs[l] + h.count[l];
  for (int s = 0; s < n; s++)
    if (length[s]) h.symbol[offs[length[s]]++] = (short)s;
  return left;
}

static int Decode(State &s, const Huff &h) {
  int code = 0, first = 0, index = 0;
  for (int len = 1; len <= 15; len++) {
    code |= s.Bits(1);
    int count = h.count[len];
    if (code - count < first) return h.symbol[index + (code - first)];
    index += count;
    first += count;
    first <<= 1;
    code <<= 1;
  }
  return -1;
}

static void Codes(State &s, const Huff &lencode, const Huff &distcode) {
  static const short lens[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
                                 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
  static const short lext[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
  static const short dists[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513,
                                  769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
  static const short dext[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
  for (;;) {
    int sym = Decode(s, lencode);
    if (sym < 0) throw Fail{L"release zip is corrupt"};
    if (sym < 256) {
      if (s.outcnt == s.outlen) throw Fail{L"release zip is corrupt"};
      s.out[s.outcnt++] = (uint8_t)sym;
    } else if (sym == 256) {
      return;
    } else {
      sym -= 257;
      if (sym >= 29) throw Fail{L"release zip is corrupt"};
      size_t len = lens[sym] + s.Bits(lext[sym]);
      int ds = Decode(s, distcode);
      if (ds < 0 || ds >= 30) throw Fail{L"release zip is corrupt"};
      size_t dist = dists[ds] + s.Bits(dext[ds]);
      if (dist > s.outcnt || s.outcnt + len > s.outlen) throw Fail{L"release zip is corrupt"};
      for (; len; len--, s.outcnt++) s.out[s.outcnt] = s.out[s.outcnt - dist];
    }
  }
}

static void Stored(State &s) {
  s.bitbuf = 0;
  s.bitcnt = 0;
  if (s.incnt + 4 > s.inlen) throw Fail{L"release zip is truncated"};
  unsigned len = s.in[s.incnt] | s.in[s.incnt + 1] << 8;
  unsigned nlen = s.in[s.incnt + 2] | s.in[s.incnt + 3] << 8;
  s.incnt += 4;
  if (len != (~nlen & 0xffff)) throw Fail{L"release zip is corrupt"};
  if (s.incnt + len > s.inlen || s.outcnt + len > s.outlen) throw Fail{L"release zip is corrupt"};
  memcpy(s.out + s.outcnt, s.in + s.incnt, len);
  s.incnt += len;
  s.outcnt += len;
}

static void Fixed(State &s) {
  static Huff lencode, distcode;
  static bool built = false;
  if (!built) {
    short lengths[288];
    int i = 0;
    for (; i < 144; i++) lengths[i] = 8;
    for (; i < 256; i++) lengths[i] = 9;
    for (; i < 280; i++) lengths[i] = 7;
    for (; i < 288; i++) lengths[i] = 8;
    Construct(lencode, lengths, 288);
    for (i = 0; i < 30; i++) lengths[i] = 5;
    Construct(distcode, lengths, 30);
    built = true;
  }
  Codes(s, lencode, distcode);
}

static void Dynamic(State &s) {
  static const short order[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
  short lengths[320];
  int nlen = s.Bits(5) + 257, ndist = s.Bits(5) + 1, ncode = s.Bits(4) + 4;
  if (nlen > 286 || ndist > 30) throw Fail{L"release zip is corrupt"};
  int i = 0;
  for (; i < ncode; i++) lengths[order[i]] = (short)s.Bits(3);
  for (; i < 19; i++) lengths[order[i]] = 0;
  Huff lencode, distcode;
  if (Construct(lencode, lengths, 19) != 0) throw Fail{L"release zip is corrupt"};
  i = 0;
  while (i < nlen + ndist) {
    int sym = Decode(s, lencode);
    if (sym < 0) throw Fail{L"release zip is corrupt"};
    if (sym < 16) {
      lengths[i++] = (short)sym;
    } else {
      short len = 0;
      if (sym == 16) {
        if (i == 0) throw Fail{L"release zip is corrupt"};
        len = lengths[i - 1];
        sym = 3 + s.Bits(2);
      } else if (sym == 17) sym = 3 + s.Bits(3);
      else sym = 11 + s.Bits(7);
      if (i + sym > nlen + ndist) throw Fail{L"release zip is corrupt"};
      while (sym--) lengths[i++] = len;
    }
  }
  if (lengths[256] == 0) throw Fail{L"release zip is corrupt"};
  if (Construct(lencode, lengths, nlen) < 0 || Construct(distcode, lengths + nlen, ndist) < 0)
    throw Fail{L"release zip is corrupt"};
  Codes(s, lencode, distcode);
}

// inflates exactly outlen bytes
static void Run(const uint8_t *in, size_t inlen, uint8_t *out, size_t outlen) {
  State s{in, inlen};
  s.out = out;
  s.outlen = outlen;
  int last;
  do {
    last = s.Bits(1);
    int type = s.Bits(2);
    if (type == 0) Stored(s);
    else if (type == 1) Fixed(s);
    else if (type == 2) Dynamic(s);
    else throw Fail{L"release zip is corrupt"};
  } while (!last);
  if (s.outcnt != outlen) throw Fail{L"release zip is corrupt"};
}
}  // namespace inflate

static uint32_t Crc32(const uint8_t *d, size_t n) {
  static uint32_t table[256];
  if (!table[1])
    for (uint32_t i = 0; i < 256; i++) {
      uint32_t c = i;
      for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1)));
      table[i] = c;
    }
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) c = table[(c ^ d[i]) & 255] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

// ---------------------------------------------------------------- paths and installed state
static fs::path g_root, g_driver, g_state;

static void InitPaths() {
  wchar_t b[MAX_PATH * 2];
  DWORD n = GetEnvironmentVariableW(L"LOCALAPPDATA", b, _countof(b));
  g_root = fs::path(n ? std::wstring(b, n) : L".") / L"QuestLHSync";
  g_driver = g_root / L"questlhsync";
  g_state = g_root / L"installed.json";
}

static std::string Slurp(const fs::path &p) {
  std::ifstream f(p, std::ios::binary);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

static std::wstring ReadInstalled() {
  std::error_code ec;
  if (!fs::exists(g_driver / L"bin" / L"win64" / L"driver_questlhsync.dll", ec)) return L"";
  std::string t = Slurp(g_state);
  size_t k = t.find("\"version\"");
  if (k == std::string::npos) return L"";
  size_t a = t.find('"', t.find(':', k) + 1), b = a == std::string::npos ? a : t.find('"', a + 1);
  return b == std::string::npos ? L"" : Wide(t.substr(a + 1, b - a - 1));
}

// ---------------------------------------------------------------- network
struct Handle {
  HINTERNET h;
  explicit Handle(HINTERNET h_) : h(h_) {}
  ~Handle() { if (h) WinHttpCloseHandle(h); }
};

struct HttpResult { DWORD status = 0; std::wstring location; };

// GET url. follow=false returns the first response (used to read /releases/latest's redirect). sink gets the body.
static HttpResult Http(const std::wstring &url, bool follow,
                       const std::function<void(const uint8_t *, size_t, uint64_t)> &sink = {}) {
  wchar_t host[256], path[2048];
  URL_COMPONENTS uc{};
  uc.dwStructSize = sizeof uc;
  uc.lpszHostName = host; uc.dwHostNameLength = _countof(host);
  uc.lpszUrlPath = path; uc.dwUrlPathLength = _countof(path);
  if (!WinHttpCrackUrl(url.c_str(), 0, 0, &uc)) throw Fail{L"bad URL " + url};
  Handle s(WinHttpOpen(L"QuestLHSync-Installer", WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME,
                       WINHTTP_NO_PROXY_BYPASS, 0));
  if (!s.h) throw Fail{F(L"network error %lu", GetLastError())};
  WinHttpSetTimeouts(s.h, 15000, 15000, 30000, 30000);
  Handle c(WinHttpConnect(s.h, host, uc.nPort, 0));
  if (!c.h) throw Fail{F(L"network error %lu", GetLastError())};
  Handle r(WinHttpOpenRequest(c.h, L"GET", path, nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES,
                              uc.nScheme == INTERNET_SCHEME_HTTPS ? WINHTTP_FLAG_SECURE : 0));
  if (!r.h) throw Fail{F(L"network error %lu", GetLastError())};
  if (!follow) {
    DWORD o = WINHTTP_DISABLE_REDIRECTS;
    WinHttpSetOption(r.h, WINHTTP_OPTION_DISABLE_FEATURE, &o, sizeof o);
  }
  if (!WinHttpSendRequest(r.h, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) ||
      !WinHttpReceiveResponse(r.h, nullptr))
    throw Fail{F(L"can't reach github.com (network error %lu)", GetLastError())};
  HttpResult res;
  DWORD sz = sizeof res.status;
  WinHttpQueryHeaders(r.h, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX,
                      &res.status, &sz, WINHTTP_NO_HEADER_INDEX);
  wchar_t loc[2048];
  sz = sizeof loc;
  if (WinHttpQueryHeaders(r.h, WINHTTP_QUERY_LOCATION, WINHTTP_HEADER_NAME_BY_INDEX, loc, &sz, WINHTTP_NO_HEADER_INDEX))
    res.location = loc;
  if (sink && res.status == 200) {
    DWORD len = 0;
    sz = sizeof len;
    uint64_t total = WinHttpQueryHeaders(r.h, WINHTTP_QUERY_CONTENT_LENGTH | WINHTTP_QUERY_FLAG_NUMBER,
                                         WINHTTP_HEADER_NAME_BY_INDEX, &len, &sz, WINHTTP_NO_HEADER_INDEX) ? len : 0;
    std::vector<uint8_t> buf(64 * 1024);
    for (;;) {
      DWORD got = 0;
      if (!WinHttpReadData(r.h, buf.data(), (DWORD)buf.size(), &got)) throw Fail{F(L"download failed (error %lu)", GetLastError())};
      if (!got) break;
      sink(buf.data(), got, total);
    }
  }
  return res;
}

static std::wstring LatestTag() {
  HttpResult r = Http(std::wstring(RELEASES) + L"/latest", false);
  size_t k = r.location.rfind(L"/releases/tag/");
  if (r.status / 100 != 3 || k == std::wstring::npos) throw Fail{F(L"couldn't read the latest release (HTTP %lu)", r.status)};
  std::wstring tag = r.location.substr(k + 14);
  tag = tag.substr(0, tag.find_first_of(L"/?#"));
  if (tag.empty()) throw Fail{L"couldn't read the latest release"};
  return tag;
}

static std::wstring PackageName(const std::wstring &tag) {
  return L"QuestLHSync-steamvr-" + tag.substr(tag.size() && (tag[0] == L'v' || tag[0] == L'V') ? 1 : 0) + L".zip";
}

// ---------------------------------------------------------------- unpack the driver folder
static void ExtractDriver(const std::vector<uint8_t> &z) {
  const Fail bad{L"release zip is corrupt"};
  auto u16 = [&](size_t o) -> uint32_t {
    if (o + 2 > z.size()) throw bad;
    return z[o] | z[o + 1] << 8;
  };
  auto u32 = [&](size_t o) -> uint32_t { return u16(o) | u16(o + 2) << 16; };

  size_t eocd = std::string::npos;
  for (size_t back = 22; back <= z.size() && back <= 22 + 65535; back++)
    if (u32(z.size() - back) == 0x06054b50) { eocd = z.size() - back; break; }
  if (eocd == std::string::npos) throw bad;
  uint32_t count = u16(eocd + 10);
  size_t p = u32(eocd + 16);

  fs::path nw = g_driver;
  nw += L".new";
  std::error_code ec;
  fs::remove_all(nw, ec);
  int files = 0;
  for (uint32_t i = 0; i < count; i++) {
    if (u32(p) != 0x02014b50) throw bad;
    uint32_t method = u16(p + 10), crc = u32(p + 16), cs = u32(p + 20), us = u32(p + 24);
    uint32_t nl = u16(p + 28), el = u16(p + 30), cl = u16(p + 32);
    size_t lo = u32(p + 42);
    if (p + 46 + nl > z.size()) throw bad;
    std::string name((const char *)&z[p + 46], nl);
    p += 46 + nl + el + cl;
    if (name.rfind("questlhsync/", 0) != 0 || name.back() == '/') continue;
    std::string rel = name.substr(12);
    // refuse anything that could leave the folder
    if (rel.empty() || rel[0] == '/' || rel.find_first_of("\\:") != std::string::npos) throw Fail{L"unsafe path in release zip"};
    for (size_t a = 0; a <= rel.size();) {
      size_t b = rel.find('/', a);
      if (b == std::string::npos) b = rel.size();
      std::string seg = rel.substr(a, b - a);
      if (seg.empty() || seg == "." || seg == "..") throw Fail{L"unsafe path in release zip"};
      a = b + 1;
    }
    if (cs == 0xFFFFFFFFu || us == 0xFFFFFFFFu) throw Fail{L"zip64 release zips aren't supported"};
    if (u32(lo) != 0x04034b50) throw bad;
    size_t data = lo + 30 + u16(lo + 26) + u16(lo + 28);
    if (data + cs > z.size()) throw bad;
    std::vector<uint8_t> out(us);
    if (method == 0) {
      if (cs != us) throw bad;
      if (us) memcpy(out.data(), &z[data], us);
    } else if (method == 8) {
      inflate::Run(&z[data], cs, out.data(), us);
    } else throw Fail{L"release zip uses an unsupported compression"};
    if (Crc32(out.data(), us) != crc) throw bad;
    fs::path dest = nw / Wide(rel);
    fs::create_directories(dest.parent_path(), ec);
    std::ofstream f(dest, std::ios::binary);
    f.write((const char *)out.data(), us);
    f.close();
    if (!f) throw Fail{L"couldn't write " + dest.wstring()};
    files++;
  }
  if (!files) throw Fail{L"release zip has no questlhsync/ driver folder"};
  // swapped by renames: a file still in use leaves the old driver whole instead of half deleted
  fs::path old = g_driver;
  old += L".old";
  fs::remove_all(old, ec);
  if (fs::exists(g_driver, ec)) {
    fs::rename(g_driver, old, ec);
    if (ec) {
      fs::remove_all(nw, ec);
      throw Fail{L"couldn't replace the old driver files (is SteamVR running?)"};
    }
  }
  fs::rename(nw, g_driver, ec);
  if (ec) {
    fs::rename(old, g_driver, ec);
    throw Fail{L"couldn't move the driver into place"};
  }
  fs::remove_all(old, ec);  // what's still in use goes at the next install
}

// ---------------------------------------------------------------- SteamVR
// the paths in openvrpaths.vrpath's `key` array ("runtime", "external_drivers")
static std::vector<fs::path> VrPaths(const char *key) {
  wchar_t b[MAX_PATH * 2];
  DWORD n = GetEnvironmentVariableW(L"LOCALAPPDATA", b, _countof(b));
  std::string t = Slurp(fs::path(std::wstring(b, n)) / L"openvr" / L"openvrpaths.vrpath");
  std::vector<fs::path> v;
  size_t k = t.find("\"" + std::string(key) + "\"");
  size_t a = k == std::string::npos ? k : t.find('[', k), e = a == std::string::npos ? a : t.find(']', a);
  if (e != std::string::npos) {
    for (size_t i = a; i < e; i++) {
      if (t[i] != '"') continue;
      std::string s;
      for (i++; i < e && t[i] != '"'; i++) {
        if (t[i] == '\\' && i + 1 < e) i++;
        s += t[i];
      }
      v.push_back(fs::path(Wide(s)));
    }
  }
  return v;
}

static fs::path VrPathReg() {
  for (const fs::path &p : VrPaths("runtime")) {
    fs::path exe = p / L"bin" / L"win64" / L"vrpathreg.exe";
    std::error_code ec;
    if (fs::exists(exe, ec)) return exe;
  }
  throw Fail{L"SteamVR not found: install and run it once, then try again"};
}

// other QuestLHSync driver folders registered with SteamVR (installed by hand before the installer)
static std::vector<fs::path> OtherCopies() {
  std::vector<fs::path> v;
  for (const fs::path &p : VrPaths("external_drivers")) {
    std::error_code ec;
    if (fs::exists(p / L"bin" / L"win64" / L"driver_questlhsync.dll", ec) && !fs::equivalent(p, g_driver, ec))
      v.push_back(p);
  }
  return v;
}

static bool SteamVrRunning() {
  HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (snap == INVALID_HANDLE_VALUE) return false;
  PROCESSENTRY32W pe{sizeof pe};
  bool found = false;
  for (BOOL ok = Process32FirstW(snap, &pe); ok && !found; ok = Process32NextW(snap, &pe))
    found = _wcsicmp(pe.szExeFile, L"vrserver.exe") == 0;
  CloseHandle(snap);
  return found;
}

static void RunReg(const wchar_t *action, const fs::path &dir = g_driver) {
  std::wstring cmd = L"\"" + VrPathReg().wstring() + L"\" " + action + L" \"" + dir.wstring() + L"\"";
  SECURITY_ATTRIBUTES sa{sizeof sa, nullptr, TRUE};
  HANDLE rd, wr;
  if (!CreatePipe(&rd, &wr, &sa, 0)) throw Fail{L"couldn't start vrpathreg"};
  SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);
  STARTUPINFOW si{sizeof si};
  si.dwFlags = STARTF_USESTDHANDLES;
  si.hStdOutput = si.hStdError = wr;
  PROCESS_INFORMATION pi{};
  BOOL ok = CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
  CloseHandle(wr);
  if (!ok) { CloseHandle(rd); throw Fail{L"couldn't start vrpathreg"}; }
  std::string output;
  char buf[512];
  DWORD got;
  while (ReadFile(rd, buf, sizeof buf, &got, nullptr) && got) output.append(buf, got);
  CloseHandle(rd);
  WaitForSingleObject(pi.hProcess, 15000);
  DWORD code = 1;
  GetExitCodeProcess(pi.hProcess, &code);
  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);
  if (code != 0) {
    while (!output.empty() && (output.back() == '\n' || output.back() == '\r' || output.back() == ' ')) output.pop_back();
    throw Fail{F(L"vrpathreg %s failed (%lu) ", action, code) + Wide(output)};
  }
}

// ---------------------------------------------------------------- shared state (UI thread <-> workers)
struct Shared {
  std::mutex m;
  std::wstring latest, installed, msg = L"Checking for updates…";
  uint32_t accent = 0;  // banner bar colour, 0 = follow the state
  std::vector<std::pair<std::wstring, std::wstring>> logs;
  int pct = 0;
  bool busy = false, err = false;
} S;

static HWND g_hwnd;
static std::atomic<bool> g_dirty{true};

static void Touch() {
  g_dirty = true;
  if (g_hwnd) InvalidateRect(g_hwnd, nullptr, FALSE);
}

static void Say(const std::wstring &m, uint32_t accent = 0) {
  SYSTEMTIME t;
  GetLocalTime(&t);
  {
    std::lock_guard<std::mutex> g(S.m);
    S.msg = m;
    S.accent = accent;
    S.logs.push_back({F(L"%02d:%02d:%02d", t.wHour, t.wMinute, t.wSecond), m});
  }
  Touch();
}

static void SetProgress(int pct) {
  {
    std::lock_guard<std::mutex> g(S.m);
    if (S.pct == pct) return;
    S.pct = pct;
  }
  Touch();
}

static void RunBg(std::function<void()> fn) {
  {
    std::lock_guard<std::mutex> g(S.m);
    if (S.busy) return;
    S.busy = true;
    S.err = false;
    S.pct = 0;
  }
  Touch();
  std::thread([fn] {
    std::wstring err;
    try { fn(); }
    catch (const Fail &f) { err = f.msg; }
    catch (const std::exception &e) { err = Wide(e.what()); }
    std::wstring inst = ReadInstalled();
    {
      std::lock_guard<std::mutex> g(S.m);
      S.installed = inst;
      S.busy = false;
      S.err = !err.empty();
    }
    if (!err.empty()) Say(L"Error: " + err, col::red);
    else Touch();
  }).detach();
}

static BOOL CALLBACK CloseVrMonitor(HWND w, LPARAM pids) {
  DWORD pid = 0;
  GetWindowThreadProcessId(w, &pid);
  for (DWORD p : *(std::vector<DWORD> *)pids)
    if (p == pid) PostMessageW(w, WM_CLOSE, 0, 0);
  return TRUE;
}

// Stops SteamVR and our dashboard app (they hold the driver folder's files open) and waits for them to exit. SteamVR
// is asked to close first, like closing its window: killed outright, Steam can go on thinking SteamVR runs and refuse
// to start it again. The dashboard app leaves with it.
static void StopSteamVr() {
  static const wchar_t *names[] = {L"vrserver.exe", L"vrmonitor.exe", L"vrcompositor.exe", L"vrdashboard.exe",
                                   L"vrwebhelper.exe", L"vrstartup.exe", L"QuestLHSync.exe"};
  std::vector<HANDLE> procs;
  std::vector<DWORD> monitor;
  HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (snap == INVALID_HANDLE_VALUE) return;
  PROCESSENTRY32W pe{sizeof pe};
  for (BOOL ok = Process32FirstW(snap, &pe); ok; ok = Process32NextW(snap, &pe))
    for (const wchar_t *n : names)
      if (_wcsicmp(pe.szExeFile, n) == 0) {
        if (HANDLE h = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE, FALSE, pe.th32ProcessID)) procs.push_back(h);
        if (_wcsicmp(n, L"vrmonitor.exe") == 0) monitor.push_back(pe.th32ProcessID);
        break;
      }
  CloseHandle(snap);
  if (procs.empty()) return;
  Say(L"Stopping SteamVR…");
  if (!monitor.empty()) {
    EnumWindows(CloseVrMonitor, (LPARAM)&monitor);
    for (int i = 0; i < 150; i++) {
      bool left = false;
      for (HANDLE h : procs) left = left || WaitForSingleObject(h, 0) == WAIT_TIMEOUT;
      if (!left) break;
      Sleep(100);
    }
  }
  for (HANDLE h : procs) TerminateProcess(h, 1);  // what didn't close in 15 s
  for (HANDLE h : procs) {
    WaitForSingleObject(h, 5000);
    CloseHandle(h);
  }
  for (int i = 0; i < 50 && SteamVrRunning(); i++) Sleep(100);
  if (SteamVrRunning()) throw Fail{L"couldn't stop SteamVR (vrserver.exe is still running)"};
}

static void Check() {
  RunBg([] {
    Say(L"Checking for updates…");
    std::wstring tag = LatestTag(), cur;
    {
      std::lock_guard<std::mutex> g(S.m);
      S.latest = tag;
      cur = S.installed;
    }
    if (cur.empty()) Say(L"Latest release is " + tag + L": ready to install.");
    else if (Newer(tag, cur)) Say(L"Update available: " + cur + L" → " + tag, col::amber);
    else Say(L"QuestLHSync is up to date.", col::green);
  });
}

static void Install(const std::wstring &tag) {
  RunBg([tag] {
    VrPathReg();  // fail early if SteamVR is missing
    Say(L"Downloading " + tag + L"…");
    std::vector<uint8_t> zip;
    Http(std::wstring(RELEASES) + L"/download/" + tag + L"/" + PackageName(tag), true,
         [&](const uint8_t *d, size_t n, uint64_t total) {
           zip.insert(zip.end(), d, d + n);
           if (total) SetProgress((int)std::min<uint64_t>(99, zip.size() * 100 / total));
         });
    if (zip.empty()) throw Fail{L"download failed: " + PackageName(tag) + L" not found"};
    StopSteamVr();
    Say(L"Installing…");
    std::error_code ec;
    fs::create_directories(g_root, ec);
    ExtractDriver(zip);
    RunReg(L"adddriver");
    for (const fs::path &old : OtherCopies()) {  // SteamVR would load one of the two: this one only from now on
      RunReg(L"removedriver", old);
      Say(L"Unregistered the older copy in " + old.wstring() + L" (its files are left as they are)");
    }
    std::string utf8(WideCharToMultiByte(CP_UTF8, 0, tag.c_str(), (int)tag.size(), nullptr, 0, nullptr, nullptr), '\0');
    WideCharToMultiByte(CP_UTF8, 0, tag.c_str(), (int)tag.size(), utf8.data(), (int)utf8.size(), nullptr, nullptr);
    std::ofstream(g_state) << "{\"version\":\"" << utf8 << "\"}";
    SetProgress(100);
    Say(L"Installed " + tag + L". Start SteamVR.", col::green);
  });
}

static void Uninstall() {
  RunBg([] {
    StopSteamVr();
    RunReg(L"removedriver");
    std::error_code ec;
    fs::remove_all(g_driver, ec);
    fs::remove(g_state, ec);
    Say(L"QuestLHSync driver uninstalled.");
  });
}

// ---------------------------------------------------------------- page
struct Button { int x, y, w, h, id; std::wstring label; bool primary, enabled; };

static std::vector<Button> g_buttons;
static int g_hover = -1;
static bool g_svr = false;
static std::unique_ptr<Canvas> g_canvas;
static int g_cw, g_ch;

static void Draw() {
  std::wstring latest, installed, msg;
  std::vector<std::pair<std::wstring, std::wstring>> logs;
  uint32_t accent;
  int pct;
  bool busy, err;
  {
    std::lock_guard<std::mutex> g(S.m);
    latest = S.latest; installed = S.installed; msg = S.msg; accent = S.accent;
    logs = S.logs; pct = S.pct; busy = S.busy; err = S.err;
  }
  bool outdated = !installed.empty() && !latest.empty() && Newer(latest, installed);

  Canvas &cv = *g_canvas;
  Painter p(cv);
  Font h1(40, FW_SEMIBOLD, L"Segoe UI Semibold"), h2(22, FW_SEMIBOLD, L"Segoe UI Semibold"), body(21, FW_NORMAL),
      small(17, FW_NORMAL), label(16, FW_SEMIBOLD, L"Segoe UI Semibold"), big(28, FW_SEMIBOLD, L"Segoe UI Semibold"),
      mono(17, FW_NORMAL, L"Cascadia Mono");
  p.Rect(0, 0, W, H, col::bg);

  // header
  int tw = p.Text(48, 34, L"QuestLHSync", h1, col::text);
  p.Text(48 + tw + 14, 50, L"by CreoleVR", h2, col::faint);
  p.Text(50, 88, L"Installer for the SteamVR driver", small, col::dim);

  std::wstring title;
  uint32_t sc;
  if (busy) { title = L"Working"; sc = col::amber; }
  else if (err) { title = L"Error"; sc = col::red; }
  else if (outdated) { title = L"Update available"; sc = col::amber; }
  else if (!installed.empty()) { title = L"Installed"; sc = col::green; }
  else { title = L"Not installed"; sc = col::grey; }
  {
    Font pill(22, FW_SEMIBOLD, L"Segoe UI Semibold");
    HGDIOBJ of = SelectObject(cv.dc, pill.f);
    SIZE sz;
    GetTextExtentPoint32W(cv.dc, title.c_str(), (int)title.size(), &sz);
    SelectObject(cv.dc, of);
    int pw = (int)(sz.cx / g_k) + 64, px = W - 48 - pw;
    p.Rect(px, 40, pw, 48, col::card2, 24);
    p.Dot(px + 26, 64, 7, sc);
    p.Text(px + 44, 49, title, pill, col::text);
  }

  // status line
  p.Rect(48, 124, W - 96, 52, col::card, 12);
  p.Rect(48, 124, 6, 52, accent ? accent : sc, 3);
  p.Text(70, 137, msg, body, col::text, 0, W - 140);

  // cards
  int top = 196, ch = 204, gap = 20, cw = (W - 96 - gap) / 2;
  int x1 = 48, x2 = x1 + cw + gap;
  for (int x : {x1, x2}) p.Rect(x, top, cw, ch, col::card, 14);
  auto row = [&](int x, int y, const wchar_t *k, const std::wstring &v, uint32_t vc = col::text) {
    p.Text(x + 22, y, k, small, col::dim);
    p.Text(x + cw - 22, y, v, small, vc, 2, cw - 150);
  };
  const std::wstring dash = L"—";

  p.Text(x1 + 22, top + 18, L"INSTALLED", label, col::faint);
  p.Text(x1 + 22, top + 44, installed.empty() ? dash : installed, big, col::text, 0, cw - 44);
  row(x1, top + 100, L"Folder", installed.empty() ? dash : L"%LOCALAPPDATA%\\QuestLHSync");
  row(x1, top + 134, L"SteamVR", g_svr ? L"running" : L"closed", g_svr ? col::amber : col::text);
  row(x1, top + 168, L"Status", installed.empty() ? L"not installed" : outdated ? L"update available" : latest.empty() ? dash : L"up to date",
      outdated ? col::amber : col::text);

  p.Text(x2 + 22, top + 18, L"LATEST RELEASE", label, col::faint);
  p.Text(x2 + 22, top + 44, latest.empty() ? dash : latest, big, !latest.empty() && !outdated ? col::green : col::text, 0, cw - 44);
  row(x2, top + 100, L"Source", L"GitHub releases");
  row(x2, top + 134, L"Package", latest.empty() ? dash : PackageName(latest));
  row(x2, top + 168, L"Registers via", L"vrpathreg");

  // log
  int ly = top + ch + 20, lh = 150;
  p.Rect(48, ly, W - 96, lh, col::card, 14);
  p.Text(70, ly + 16, L"LOG", label, col::faint);
  if (busy || pct) {
    int bw = 220, bx = W - 48 - 22 - bw;
    p.Rect(bx, ly + 22, bw, 8, col::card2, 4);
    if (pct) p.Rect(bx, ly + 22, std::max(8, bw * pct / 100), 8, col::blue, 4);
  }
  size_t n = logs.size();
  for (size_t i = 0; i < 5 && i < n; i++) {
    const auto &l = logs[n - 1 - i];
    p.Text(70, ly + 44 + 20 * (int)i, l.first + L"  " + l.second, mono, i == 0 ? col::text : col::dim, 0, W - 150);
  }
  if (!n) p.Text(70, ly + 44, dash, mono, col::dim);

  // buttons
  g_buttons.clear();
  int by = H - 66, bh = 46;
  std::wstring main_label = installed.empty() ? L"Install" : outdated ? L"Update to " + latest : L"Reinstall";
  g_buttons.push_back({48, by, outdated ? 260 : 200, bh, 0, main_label, installed.empty() || outdated, !busy && !latest.empty()});
  int x = 48 + g_buttons[0].w + 16;
  g_buttons.push_back({x, by, 170, bh, 1, L"Uninstall", false, !busy && !installed.empty()});
  x += 170 + 16;
  g_buttons.push_back({x, by, 250, bh, 2, L"Check for updates", false, !busy});
  for (size_t i = 0; i < g_buttons.size(); i++) {
    auto &b = g_buttons[i];
    uint32_t fill = !b.enabled ? col::card : b.primary ? (int)i == g_hover ? 0x7aa0ff : col::blue : (int)i == g_hover ? col::line : col::card2;
    p.Rect(b.x, b.y, b.w, b.h, fill, 10);
    p.Text(b.x + b.w / 2, b.y + 8, b.label, h2, b.enabled ? col::text : col::faint, 1);  // Segoe UI sits low in its line box
  }
}

static int HitTest(LPARAM lp) {
  double x = (short)LOWORD(lp) / g_k, y = (short)HIWORD(lp) / g_k;
  for (size_t i = 0; i < g_buttons.size(); i++) {
    auto &b = g_buttons[i];
    if (b.enabled && x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) return (int)i;
  }
  return -1;
}

static void Click(int id) {
  if (id == 2) return Check();
  if (SteamVrRunning()) g_svr = true;  // Install/Uninstall stop it themselves
  if (id == 0) {
    std::wstring tag;
    {
      std::lock_guard<std::mutex> g(S.m);
      tag = S.latest;
    }
    return Install(tag);
  }
  if (MessageBoxW(g_hwnd, L"Uninstall the QuestLHSync driver?", L"QuestLHSync", MB_YESNO | MB_ICONQUESTION) == IDYES)
    Uninstall();
}

static LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM wp, LPARAM lp) {
  switch (m) {
    case WM_PAINT: {
      PAINTSTRUCT ps;
      HDC dc = BeginPaint(h, &ps);
      if (g_dirty.exchange(false)) Draw();
      BitBlt(dc, 0, 0, g_cw, g_ch, g_canvas->dc, 0, 0, SRCCOPY);
      EndPaint(h, &ps);
      return 0;
    }
    case WM_ERASEBKGND: return 1;
    case WM_MOUSEMOVE: {
      TRACKMOUSEEVENT t{sizeof t, TME_LEAVE, h, 0};
      TrackMouseEvent(&t);
      int hv = HitTest(lp);
      if (hv != g_hover) { g_hover = hv; Touch(); }
      return 0;
    }
    case WM_MOUSELEAVE:
      if (g_hover != -1) { g_hover = -1; Touch(); }
      return 0;
    case WM_SETCURSOR:
      if (LOWORD(lp) == HTCLIENT) {
        SetCursor(LoadCursorW(nullptr, g_hover >= 0 ? IDC_HAND : IDC_ARROW));
        return TRUE;
      }
      break;
    case WM_LBUTTONUP: {
      int hv = HitTest(lp);
      if (hv >= 0) Click(g_buttons[hv].id);
      return 0;
    }
    case WM_TIMER: {
      bool r = SteamVrRunning();
      if (r != g_svr) { g_svr = r; Touch(); }
      return 0;
    }
    case WM_DESTROY: PostQuitMessage(0); return 0;
  }
  return DefWindowProcW(h, m, wp, lp);
}

int WINAPI wWinMain(HINSTANCE hi, HINSTANCE, PWSTR, int show) {
  SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_SYSTEM_AWARE);
  g_k = GetDpiForSystem() / 96.0;
  InitPaths();
  S.installed = ReadInstalled();
  g_svr = SteamVrRunning();

  g_cw = (int)lround(W * g_k);
  g_ch = (int)lround(H * g_k);
  g_canvas = std::make_unique<Canvas>(g_cw, g_ch);

  WNDCLASSW wc{};
  wc.lpfnWndProc = WndProc;
  wc.hInstance = hi;
  wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
  wc.hbrBackground = CreateSolidBrush(C(col::bg));
  wc.lpszClassName = L"QuestLHSyncInstaller";
  RegisterClassW(&wc);
  DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
  RECT rc{0, 0, g_cw, g_ch};
  AdjustWindowRectExForDpi(&rc, style, FALSE, 0, GetDpiForSystem());
  int ww = rc.right - rc.left, wh = rc.bottom - rc.top;
  g_hwnd = CreateWindowW(wc.lpszClassName, L"QuestLHSync Installer", style, (GetSystemMetrics(SM_CXSCREEN) - ww) / 2,
                         (GetSystemMetrics(SM_CYSCREEN) - wh) / 2, ww, wh, nullptr, nullptr, hi, nullptr);
  BOOL dark = TRUE;
  DwmSetWindowAttribute(g_hwnd, 20 /* DWMWA_USE_IMMERSIVE_DARK_MODE */, &dark, sizeof dark);
  SetTimer(g_hwnd, 1, 1000, nullptr);
  ShowWindow(g_hwnd, show);
  Check();

  MSG msg;
  while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
    TranslateMessage(&msg);
    DispatchMessageW(&msg);
  }
  return 0;
}
