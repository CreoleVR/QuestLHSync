// small 3D math for the solver: vectors, row-major 3x3 matrices, unit quaternions (w x y z)
#pragma once
#include <cmath>

struct V3 {
  double x = 0, y = 0, z = 0;
  double &operator[](int i) { return (&x)[i]; }
  double operator[](int i) const { return (&x)[i]; }
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline V3 operator*(double s, V3 a) { return a * s; }
inline V3 &operator+=(V3 &a, V3 b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double norm(V3 a) { return std::sqrt(dot(a, a)); }

struct M3 {
  double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  V3 col(int c) const { return {m[0][c], m[1][c], m[2][c]}; }
};
inline V3 operator*(const M3 &R, V3 v) {
  return {R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z, R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
          R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z};
}
inline M3 operator*(const M3 &a, const M3 &b) {
  M3 r;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
  return r;
}
inline M3 T(const M3 &a) {
  M3 r;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) r.m[i][j] = a.m[j][i];
  return r;
}
// rotation about +y (gravity): the 4-DOF alignment's yaw
inline M3 Ry(double a) {
  double c = std::cos(a), s = std::sin(a);
  M3 r;
  r.m[0][0] = c; r.m[0][1] = 0; r.m[0][2] = s;
  r.m[1][0] = 0; r.m[1][1] = 1; r.m[1][2] = 0;
  r.m[2][0] = -s; r.m[2][1] = 0; r.m[2][2] = c;
  return r;
}
inline V3 RyMul(double c, double s, V3 v) { return {c * v.x + s * v.z, v.y, -s * v.x + c * v.z}; }
// rotation by |(ax, 0, az)| rad about the horizontal axis along (ax, 0, az): a tilt, no turn about the vertical
inline M3 Tilt(double ax, double az) {
  double th = std::hypot(ax, az);
  M3 r;
  if (th < 1e-15) return r;
  double x = ax / th, z = az / th, c = std::cos(th), s = std::sin(th), k = 1 - c;
  r.m[0][0] = c + k * x * x; r.m[0][1] = -s * z;  r.m[0][2] = k * x * z;
  r.m[1][0] = s * z;         r.m[1][1] = c;       r.m[1][2] = -s * x;
  r.m[2][0] = k * x * z;     r.m[2][1] = s * x;   r.m[2][2] = c + k * z * z;
  return r;
}
// angle of a rotation matrix (deg)
inline double RotDeg(const M3 &R) {  // atan2 form: acos loses small angles
  double c = (R.m[0][0] + R.m[1][1] + R.m[2][2] - 1) / 2;
  double sx = R.m[2][1] - R.m[1][2], sy = R.m[0][2] - R.m[2][0], sz = R.m[1][0] - R.m[0][1];
  return std::atan2(std::sqrt(sx * sx + sy * sy + sz * sz) / 2, c) * 57.29577951308232;
}
// how far a rotation matrix tilts the vertical (deg)
inline double TiltDeg(const M3 &R) {
  return std::atan2(std::hypot(R.m[0][1], R.m[2][1]), R.m[1][1]) * 57.29577951308232;
}
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = 180.0 / kPi;
inline double Wrap(double a) {  // [-pi, pi)
  a = std::fmod(a + kPi, 2 * kPi);
  if (a < 0) a += 2 * kPi;
  return a - kPi;
}

struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
};
inline Quat operator*(const Quat &a, const Quat &b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline M3 ToM3(const Quat &q0) {  // normalizes (logged quaternions are rounded)
  double n = std::sqrt(q0.w * q0.w + q0.x * q0.x + q0.y * q0.y + q0.z * q0.z);
  double w = q0.w / n, x = q0.x / n, y = q0.y / n, z = q0.z / n;
  M3 r;
  r.m[0][0] = 1 - 2 * (y * y + z * z); r.m[0][1] = 2 * (x * y - z * w); r.m[0][2] = 2 * (x * z + y * w);
  r.m[1][0] = 2 * (x * y + z * w); r.m[1][1] = 1 - 2 * (x * x + z * z); r.m[1][2] = 2 * (y * z - x * w);
  r.m[2][0] = 2 * (x * z - y * w); r.m[2][1] = 2 * (y * z + x * w); r.m[2][2] = 1 - 2 * (x * x + y * y);
  return r;
}
inline Quat ToQuat(const M3 &R) {
  const auto &m = R.m;
  double tr = m[0][0] + m[1][1] + m[2][2];
  Quat q;
  if (tr > 0) {
    double s = std::sqrt(tr + 1.0) * 2;
    q = {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
    q = {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
  } else if (m[1][1] > m[2][2]) {
    double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
    q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s};
  } else {
    double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
    q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s};
  }
  double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}
inline Quat Slerp(Quat a, Quat b, double s) {
  double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (d < 0) { b = {-b.w, -b.x, -b.y, -b.z}; d = -d; }
  double ka, kb;
  if (d > 0.9999995) { ka = 1 - s; kb = s; }
  else {
    double th = std::acos(d), sn = std::sin(th);
    ka = std::sin((1 - s) * th) / sn; kb = std::sin(s * th) / sn;
  }
  Quat q{ka * a.w + kb * b.w, ka * a.x + kb * b.x, ka * a.y + kb * b.y, ka * a.z + kb * b.z};
  double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}
// angle between two unit quaternions (deg)
inline double QuatDeg(const Quat &a, const Quat &b) {  // unit quaternions; atan2 form for small angles
  double w = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;  // (conj(a) * b).w
  double x = a.w * b.x - a.x * b.w - a.y * b.z + a.z * b.y;
  double y = a.w * b.y + a.x * b.z - a.y * b.w - a.z * b.x;
  double z = a.w * b.z - a.x * b.y + a.y * b.x - a.z * b.w;
  return 2 * std::atan2(std::sqrt(x * x + y * y + z * z), std::fabs(w)) * 57.29577951308232;
}
