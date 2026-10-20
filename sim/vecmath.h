// Small vector / quaternion / matrix types for the simulation. Doubles throughout: the sim runs at 1 kHz with stiff
// springs, and doubles keep it accurate and deterministic. Y is up, right-handed.
#pragma once
#include <cmath>

namespace tws {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;

inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
};
inline Vec3 operator*(double s, const Vec3& v) { return v * s; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) {
    double l = length(v);
    return l > 1e-12 ? v / l : Vec3{0, 0, 0};
}
inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }

struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
    static Quat axisAngle(const Vec3& axis, double angle) {
        Vec3 a = normalize(axis);
        double s = std::sin(angle * 0.5);
        return {std::cos(angle * 0.5), a.x * s, a.y * s, a.z * s};
    }
    Quat operator*(const Quat& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
    }
    Quat conj() const { return {w, -x, -y, -z}; }
    Vec3 rotate(const Vec3& v) const {
        // v' = v + 2w(q x v) + 2 q x (q x v)
        Vec3 q{x, y, z};
        Vec3 t = cross(q, v) * 2.0;
        return v + t * w + cross(q, t);
    }
    Quat normalized() const {
        double l = std::sqrt(w * w + x * x + y * y + z * z);
        return {w / l, x / l, y / l, z / l};
    }
    // Integrate by a world-space angular velocity over dt (exact for constant omega).
    Quat integrated(const Vec3& omega, double dt) const {
        double ang = length(omega) * dt;
        if (ang < 1e-15) return *this;
        return (axisAngle(omega, ang) * (*this)).normalized();
    }
};

inline Quat nlerp(Quat a, const Quat& b, double t) {
    if (a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z < 0) a = {-a.w, -a.x, -a.y, -a.z};
    Quat r{a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
    return r.normalized();
}

// Any unit vector perpendicular to n.
inline Vec3 anyPerpendicular(const Vec3& n) {
    Vec3 a = std::fabs(n.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    return normalize(cross(n, a));
}

}  // namespace tws
