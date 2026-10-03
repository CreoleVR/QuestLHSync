// minimal JSON reader for the camera calibration and QuestLHSync's own state files
#pragma once
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

struct JVal {
  enum Type { Null, Bool, Num, Str, Arr, Obj } t = Null;
  double n = 0;
  bool b = false;
  std::string s;
  std::vector<JVal> a;
  std::vector<std::pair<std::string, JVal>> o;

  const JVal *get(const char *k) const {
    if (t != Obj) return nullptr;
    for (auto &kv : o)
      if (kv.first == k) return &kv.second;
    return nullptr;
  }
  double num(double def = 0) const { return t == Num ? n : def; }
  // numeric array -> doubles (empty if not one)
  std::vector<double> nums() const {
    std::vector<double> v;
    if (t == Arr)
      for (auto &e : a) v.push_back(e.num());
    return v;
  }
};

class JParser {
 public:
  JParser(const char *p, size_t n) : p_(p), e_(p + n) {}
  bool Parse(JVal &v) {
    if (!Value(v, 0)) return false;
    Ws();
    return p_ == e_;
  }

 private:
  const char *p_, *e_;
  void Ws() {
    while (p_ < e_ && (*p_ == ' ' || *p_ == '\t' || *p_ == '\n' || *p_ == '\r')) p_++;
  }
  bool Lit(const char *w) {
    size_t n = strlen(w);
    if ((size_t)(e_ - p_) < n || memcmp(p_, w, n)) return false;
    p_ += n;
    return true;
  }
  bool String(std::string &s) {
    if (p_ >= e_ || *p_ != '"') return false;
    p_++;
    while (p_ < e_ && *p_ != '"') {
      char c = *p_++;
      if (c != '\\') { s += c; continue; }
      if (p_ >= e_) return false;
      c = *p_++;
      switch (c) {
        case 'n': s += '\n'; break;
        case 't': s += '\t'; break;
        case 'r': s += '\r'; break;
        case 'b': s += '\b'; break;
        case 'f': s += '\f'; break;
        case 'u': {
          if (e_ - p_ < 4) return false;
          unsigned u = (unsigned)strtoul(std::string(p_, 4).c_str(), nullptr, 16);
          p_ += 4;
          if (u < 0x80) s += (char)u;
          else if (u < 0x800) { s += (char)(0xC0 | (u >> 6)); s += (char)(0x80 | (u & 63)); }
          else { s += (char)(0xE0 | (u >> 12)); s += (char)(0x80 | ((u >> 6) & 63)); s += (char)(0x80 | (u & 63)); }
          break;
        }
        default: s += c;
      }
    }
    if (p_ >= e_) return false;
    p_++;
    return true;
  }
  bool Value(JVal &v, int depth) {
    if (depth > 64) return false;
    Ws();
    if (p_ >= e_) return false;
    char c = *p_;
    if (c == '{') {
      v.t = JVal::Obj;
      p_++;
      Ws();
      if (p_ < e_ && *p_ == '}') { p_++; return true; }
      for (;;) {
        Ws();
        std::string k;
        if (!String(k)) return false;
        Ws();
        if (p_ >= e_ || *p_++ != ':') return false;
        v.o.emplace_back(std::move(k), JVal());
        if (!Value(v.o.back().second, depth + 1)) return false;
        Ws();
        if (p_ < e_ && *p_ == ',') { p_++; continue; }
        if (p_ < e_ && *p_ == '}') { p_++; return true; }
        return false;
      }
    }
    if (c == '[') {
      v.t = JVal::Arr;
      p_++;
      Ws();
      if (p_ < e_ && *p_ == ']') { p_++; return true; }
      for (;;) {
        v.a.emplace_back();
        if (!Value(v.a.back(), depth + 1)) return false;
        Ws();
        if (p_ < e_ && *p_ == ',') { p_++; continue; }
        if (p_ < e_ && *p_ == ']') { p_++; return true; }
        return false;
      }
    }
    if (c == '"') { v.t = JVal::Str; return String(v.s); }
    if (Lit("true")) { v.t = JVal::Bool; v.b = true; return true; }
    if (Lit("false")) { v.t = JVal::Bool; return true; }
    if (Lit("null")) return true;
    char tmp[64];
    size_t n = 0;
    while (p_ < e_ && n < sizeof tmp - 1 && strchr("+-0123456789.eE", *p_)) tmp[n++] = *p_++;
    if (!n) return false;
    tmp[n] = 0;
    v.t = JVal::Num;
    v.n = strtod(tmp, nullptr);
    return true;
  }
};

inline bool JParse(const std::string &text, JVal &v) { return JParser(text.data(), text.size()).Parse(v); }
