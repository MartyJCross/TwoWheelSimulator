#include "articulated.h"

#include <cmath>

namespace tws {

// --- 3x3 ------------------------------------------------------------------------------------------------------

Mat3 Mat3::fromQuat(const Quat& q) {
    Mat3 r;
    double w = q.w, x = q.x, y = q.y, z = q.z;
    r.m[0][0] = 1 - 2 * (y * y + z * z); r.m[0][1] = 2 * (x * y - w * z);     r.m[0][2] = 2 * (x * z + w * y);
    r.m[1][0] = 2 * (x * y + w * z);     r.m[1][1] = 1 - 2 * (x * x + z * z); r.m[1][2] = 2 * (y * z - w * x);
    r.m[2][0] = 2 * (x * z - w * y);     r.m[2][1] = 2 * (y * z + w * x);     r.m[2][2] = 1 - 2 * (x * x + y * y);
    return r;
}

Mat3 Mat3::diag(double a, double b, double c) {
    Mat3 r;
    r.m[0][0] = a; r.m[1][1] = b; r.m[2][2] = c;
    r.m[0][1] = r.m[0][2] = r.m[1][0] = r.m[1][2] = r.m[2][0] = r.m[2][1] = 0;
    return r;
}

Vec3 Mat3::operator*(const Vec3& v) const {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
}

Mat3 Mat3::operator*(const Mat3& o) const {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
    return r;
}

Mat3 Mat3::transposed() const {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
    return r;
}

// --- 6x6 ------------------------------------------------------------------------------------------------------

namespace {

inline double sget(const SVec& v, int i) {
    return i < 3 ? (i == 0 ? v.ang.x : i == 1 ? v.ang.y : v.ang.z) : (i == 3 ? v.lin.x : i == 4 ? v.lin.y : v.lin.z);
}
inline void sset(SVec& v, int i, double x) {
    if (i < 3) (i == 0 ? v.ang.x : i == 1 ? v.ang.y : v.ang.z) = x;
    else (i == 3 ? v.lin.x : i == 4 ? v.lin.y : v.lin.z) = x;
}

// Motion cross product: v x m.
SVec crm(const SVec& v, const SVec& m) { return {cross(v.ang, m.ang), cross(v.ang, m.lin) + cross(v.lin, m.ang)}; }
// Force cross product: v x* f.
SVec crf(const SVec& v, const SVec& f) { return {cross(v.ang, f.ang) + cross(v.lin, f.lin), cross(v.ang, f.lin)}; }

Mat3 skew(const Vec3& v) {
    Mat3 r;
    r.m[0][0] = 0;     r.m[0][1] = -v.z; r.m[0][2] = v.y;
    r.m[1][0] = v.z;   r.m[1][1] = 0;    r.m[1][2] = -v.x;
    r.m[2][0] = -v.y;  r.m[2][1] = v.x;  r.m[2][2] = 0;
    return r;
}

Mat6 spatialInertia(double mass, const Vec3& c, const Mat3& ic) {
    Mat6 I;
    Mat3 cx = skew(c);
    Mat3 cxcxT = cx * cx.transposed();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            I.m[i][j] = ic.m[i][j] + mass * cxcxT.m[i][j];
            I.m[i][j + 3] = mass * cx.m[i][j];
            I.m[i + 3][j] = mass * cx.m[j][i];
            I.m[i + 3][j + 3] = (i == j) ? mass : 0.0;
        }
    return I;
}

Mat6 xformMatrix(const XForm& x) {
    // Motion transform [E 0; -E rx E].
    Mat6 X;
    Mat3 erx = x.E * skew(x.r);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            X.m[i][j] = x.E.m[i][j];
            X.m[i][j + 3] = 0;
            X.m[i + 3][j] = -erx.m[i][j];
            X.m[i + 3][j + 3] = x.E.m[i][j];
        }
    return X;
}

// Solve A x = b for 6x6 A (Gaussian elimination, partial pivoting; deterministic).
SVec solve6(Mat6 A, SVec bv) {
    double b[6];
    for (int i = 0; i < 6; ++i) b[i] = sget(bv, i);
    for (int c = 0; c < 6; ++c) {
        int piv = c;
        for (int r = c + 1; r < 6; ++r)
            if (std::fabs(A.m[r][c]) > std::fabs(A.m[piv][c])) piv = r;
        if (piv != c) {
            for (int k = 0; k < 6; ++k) std::swap(A.m[c][k], A.m[piv][k]);
            std::swap(b[c], b[piv]);
        }
        double d = A.m[c][c];
        for (int r = c + 1; r < 6; ++r) {
            double f = A.m[r][c] / d;
            for (int k = c; k < 6; ++k) A.m[r][k] -= f * A.m[c][k];
            b[r] -= f * b[c];
        }
    }
    double x[6];
    for (int r = 5; r >= 0; --r) {
        double s = b[r];
        for (int k = r + 1; k < 6; ++k) s -= A.m[r][k] * x[k];
        x[r] = s / A.m[r][r];
    }
    SVec out;
    for (int i = 0; i < 6; ++i) sset(out, i, x[i]);
    return out;
}

}  // namespace

SVec Mat6::operator*(const SVec& v) const {
    SVec r;
    for (int i = 0; i < 6; ++i) {
        double s = 0;
        for (int j = 0; j < 6; ++j) s += m[i][j] * sget(v, j);
        sset(r, i, s);
    }
    return r;
}

Mat6 Mat6::operator+(const Mat6& o) const {
    Mat6 r;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) r.m[i][j] = m[i][j] + o.m[i][j];
    return r;
}

Mat6 Mat6::operator-(const Mat6& o) const {
    Mat6 r;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) r.m[i][j] = m[i][j] - o.m[i][j];
    return r;
}

SVec XForm::motion(const SVec& v) const { return {E * v.ang, E * (v.lin - cross(r, v.ang))}; }

SVec XForm::forceT(const SVec& f) const {
    Mat3 Et = E.transposed();
    Vec3 fl = Et * f.lin;
    return {Et * f.ang + cross(r, fl), fl};
}

Mat6 XForm::inertiaToParent(const Mat6& I) const {
    Mat6 X = xformMatrix(*this);
    // X^T I X
    Mat6 IX, out;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            double s = 0;
            for (int k = 0; k < 6; ++k) s += I.m[i][k] * X.m[k][j];
            IX.m[i][j] = s;
        }
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            double s = 0;
            for (int k = 0; k < 6; ++k) s += X.m[k][i] * IX.m[k][j];
            out.m[i][j] = s;
        }
    return out;
}

// --- Articulated ----------------------------------------------------------------------------------------------

namespace {

XForm jointXForm(const Body& b) {
    // Parent -> joint frame (tree), then joint motion.
    XForm tree{Mat3::fromQuat(b.joint_rot).transposed(), b.joint_pos};
    Mat3 EJ;
    Vec3 rJ;
    if (b.type == JointType::Revolute) {
        EJ = Mat3::fromQuat(Quat::axisAngle(b.axis, b.q)).transposed();
    } else {
        rJ = b.axis * b.q;
    }
    XForm x;
    x.E = EJ * tree.E;
    x.r = tree.r + tree.E.transposed() * rJ;
    return x;
}

SVec jointS(const Body& b) {
    return b.type == JointType::Revolute ? SVec{b.axis, Vec3{}} : SVec{Vec3{}, b.axis};
}

}  // namespace

void Articulated::clearForces() { fext_.assign(bodies.size() + 1, SVec{}); }

void Articulated::addForce(int body, const Vec3& force, const Vec3& worldPoint) {
    if (fext_.size() != bodies.size() + 1) clearForces();
    Mat3 Rt = Mat3::fromQuat(bodyRot(body)).transposed();
    Vec3 f = Rt * force;
    Vec3 n = Rt * cross(worldPoint - bodyPos(body), force);
    fext_[body + 1].ang += n;
    fext_[body + 1].lin += f;
}

void Articulated::addTorque(int body, const Vec3& torque) {
    if (fext_.size() != bodies.size() + 1) clearForces();
    fext_[body + 1].ang += Mat3::fromQuat(bodyRot(body)).transposed() * torque;
}

void Articulated::updateKinematics() {
    for (Body& b : bodies) {
        XForm x = jointXForm(b);
        Vec3 ppos = b.parent < 0 ? base_pos : bodies[b.parent].pos;
        Quat prot = b.parent < 0 ? base_rot : bodies[b.parent].rot;
        SVec pvel = b.parent < 0 ? base_vel : bodies[b.parent].vel;
        Mat3 Rp = Mat3::fromQuat(prot);
        b.pos = ppos + Rp * x.r;
        // Child rotation (body -> world) = Rp * E^T. Build the quaternion by composing the tree and joint rotations.
        Quat local = b.joint_rot;
        if (b.type == JointType::Revolute) local = b.joint_rot * Quat::axisAngle(b.axis, b.q);
        b.rot = (prot * local).normalized();
        b.vel = x.motion(pvel) + jointS(b) * b.qd;
    }
}

void Articulated::step(double dt) {
    const int n = static_cast<int>(bodies.size());
    if (static_cast<int>(fext_.size()) != n + 1) clearForces();
    updateKinematics();

    // Gravity on every body at its centre of mass.
    auto addGravity = [&](int idx, double mass, const Vec3& com) {
        Mat3 Rt = Mat3::fromQuat(bodyRot(idx)).transposed();
        Vec3 g = Rt * (gravity * mass);
        fext_[idx + 1].ang += cross(com, g);
        fext_[idx + 1].lin += g;
    };
    addGravity(-1, base_mass, base_com);
    for (int i = 0; i < n; ++i) addGravity(i, bodies[i].mass, bodies[i].com);

    std::vector<XForm> X(n);
    std::vector<SVec> S(n), c(n), pA(n + 1), U(n);
    std::vector<Mat6> IA(n + 1);
    std::vector<double> D(n), u(n);

    Mat6 I0 = spatialInertia(base_mass, base_com, base_inertia);
    IA[0] = I0;
    pA[0] = crf(base_vel, I0 * base_vel) - fext_[0];
    for (int i = 0; i < n; ++i) {
        Body& b = bodies[i];
        X[i] = jointXForm(b);
        S[i] = jointS(b);
        SVec pv = b.parent < 0 ? base_vel : bodies[b.parent].vel;
        SVec vJ = S[i] * b.qd;
        b.vel = X[i].motion(pv) + vJ;
        c[i] = crm(b.vel, vJ);
        Mat6 I = spatialInertia(b.mass, b.com, b.inertia);
        IA[i + 1] = I;
        pA[i + 1] = crf(b.vel, I * b.vel) - fext_[i + 1];
    }
    for (int i = n - 1; i >= 0; --i) {
        U[i] = IA[i + 1] * S[i];
        D[i] = dot(S[i], U[i]);
        u[i] = bodies[i].tau - dot(S[i], pA[i + 1]);
        Mat6 Ia = IA[i + 1];
        for (int r = 0; r < 6; ++r)
            for (int k = 0; k < 6; ++k) Ia.m[r][k] -= sget(U[i], r) * sget(U[i], k) / D[i];
        SVec pa = pA[i + 1] + Ia * c[i] + U[i] * (u[i] / D[i]);
        int p = bodies[i].parent + 1;
        IA[p] = IA[p] + X[i].inertiaToParent(Ia);
        pA[p] = pA[p] + X[i].forceT(pa);
    }
    SVec a0 = solve6(IA[0], pA[0]) * -1.0;
    base_acc_ = a0;
    std::vector<SVec> a(n);
    for (int i = 0; i < n; ++i) {
        SVec ap = bodies[i].parent < 0 ? a0 : a[bodies[i].parent];
        SVec a1 = X[i].motion(ap) + c[i];
        bodies[i].qdd = (u[i] - dot(U[i], a1)) / D[i];
        a[i] = a1 + S[i] * bodies[i].qdd;
    }

    // Semi-implicit Euler: velocities first, then positions with the new velocities.
    base_vel = base_vel + a0 * dt;
    for (Body& b : bodies) {
        b.qd += b.qdd * dt;
        b.q += b.qd * dt;
    }
    base_pos += base_rot.rotate(base_vel.lin) * dt;
    double w = length(base_vel.ang);
    if (w > 1e-15) base_rot = (base_rot * Quat::axisAngle(base_vel.ang, w * dt)).normalized();

    clearForces();
    updateKinematics();
}

Vec3 Articulated::pointVelocity(int body, const Vec3& p) const {
    Quat r = bodyRot(body);
    SVec v = body < 0 ? base_vel : bodies[body].vel;
    Vec3 pb = r.conj().rotate(p - bodyPos(body));
    return r.rotate(v.lin + cross(v.ang, pb));
}

Vec3 Articulated::angularVelocity(int body) const {
    return bodyRot(body).rotate(body < 0 ? base_vel.ang : bodies[body].vel.ang);
}

double Articulated::totalMass() const {
    double m = base_mass;
    for (const Body& b : bodies) m += b.mass;
    return m;
}

Vec3 Articulated::centreOfMass() const {
    Vec3 s = (base_pos + base_rot.rotate(base_com)) * base_mass;
    for (const Body& b : bodies) s += (b.pos + b.rot.rotate(b.com)) * b.mass;
    return s / totalMass();
}

Vec3 Articulated::linearMomentum() const {
    Vec3 p = pointVelocity(-1, base_pos + base_rot.rotate(base_com)) * base_mass;
    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        const Body& b = bodies[i];
        p += pointVelocity(i, b.pos + b.rot.rotate(b.com)) * b.mass;
    }
    return p;
}

Vec3 Articulated::angularMomentum() const {
    auto one = [&](int idx, double mass, const Vec3& com, const Mat3& ic) {
        Quat r = bodyRot(idx);
        Vec3 pc = bodyPos(idx) + r.rotate(com);
        Vec3 vc = pointVelocity(idx, pc);
        Vec3 wb = idx < 0 ? base_vel.ang : bodies[idx].vel.ang;
        Vec3 Lspin = r.rotate(ic * wb);
        return Lspin + cross(pc, vc * mass);
    };
    Vec3 L = one(-1, base_mass, base_com, base_inertia);
    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) L += one(i, bodies[i].mass, bodies[i].com, bodies[i].inertia);
    return L;
}

double Articulated::kineticEnergy() const {
    double e = 0.5 * dot(base_vel, spatialInertia(base_mass, base_com, base_inertia) * base_vel);
    for (const Body& b : bodies) e += 0.5 * dot(b.vel, spatialInertia(b.mass, b.com, b.inertia) * b.vel);
    return e;
}

}  // namespace tws
