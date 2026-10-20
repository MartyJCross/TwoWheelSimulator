// Articulated rigid-body dynamics: Featherstone's Articulated-Body Algorithm (ABA) for a tree of rigid bodies on a
// floating base, with 1-DOF revolute and prismatic joints. Exact for trees (no constraint drift, no solver
// iterations), O(n), deterministic. Used for the bike: frame (base), steering, fork slider, wheels, swingarm, rider.
//
// Conventions (Featherstone, "Rigid Body Dynamics Algorithms", 2008):
//  - Spatial vectors are [angular; linear]. Motion: [omega; v], force: [torque; force].
//  - Each body has its own body-fixed frame. Its velocity/acceleration are kept in its own coordinates.
//  - The base's linear velocity is that of the point at the base frame's origin.
#pragma once
#include <vector>

#include "vecmath.h"

namespace tws {

struct Mat3 {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    static Mat3 fromQuat(const Quat& q);
    static Mat3 diag(double a, double b, double c);
    Vec3 operator*(const Vec3& v) const;
    Mat3 operator*(const Mat3& o) const;
    Mat3 transposed() const;
};

struct SVec {
    Vec3 ang, lin;
    SVec operator+(const SVec& o) const { return {ang + o.ang, lin + o.lin}; }
    SVec operator-(const SVec& o) const { return {ang - o.ang, lin - o.lin}; }
    SVec operator*(double s) const { return {ang * s, lin * s}; }
};
inline double dot(const SVec& a, const SVec& b) { return dot(a.ang, b.ang) + dot(a.lin, b.lin); }

struct Mat6 {
    double m[6][6] = {};
    SVec operator*(const SVec& v) const;
    Mat6 operator+(const Mat6& o) const;
    Mat6 operator-(const Mat6& o) const;
};

// Plücker transform from frame A to frame B: B's origin at r (A coords), E rotates A coords into B coords.
struct XForm {
    Mat3 E;
    Vec3 r;
    SVec motion(const SVec& v) const;          // A -> B, motion vector
    SVec forceT(const SVec& f) const;          // B -> A, force vector (X^T f)
    Mat6 inertiaToParent(const Mat6& I) const;  // X^T I X (B inertia expressed in A)
};

enum class JointType { Revolute, Prismatic };

struct Body {
    // Model.
    int parent = -1;          // -1 = attached to the base
    JointType type = JointType::Revolute;
    Vec3 axis{1, 0, 0};       // joint axis in the joint frame (unit)
    Vec3 joint_pos;           // joint frame origin in the parent's body frame
    Quat joint_rot;           // joint frame orientation relative to the parent's body frame
    double mass = 0;
    Vec3 com;                 // centre of mass in this body's frame
    Mat3 inertia = Mat3::diag(0, 0, 0);  // about the centre of mass, this body's axes

    // State.
    double q = 0, qd = 0;
    // Input for the next step: joint force/torque.
    double tau = 0;

    // Results of the last kinematics pass (world).
    Vec3 pos;                 // body frame origin
    Quat rot;                 // body frame -> world
    SVec vel;                 // body coords
    double qdd = 0;
};

class Articulated {
public:
    // Base (the bike frame).
    double base_mass = 0;
    Vec3 base_com;
    Mat3 base_inertia = Mat3::diag(0, 0, 0);
    Vec3 base_pos;
    Quat base_rot;
    SVec base_vel;  // body coords: [omega; velocity of the base origin]

    std::vector<Body> bodies;  // parents must come before children

    Vec3 gravity{0, -kGravity, 0};

    // External forces for the next step, world coords, applied at a world point. Index -1 = the base.
    void clearForces();
    void addForce(int body, const Vec3& force, const Vec3& worldPoint);
    void addTorque(int body, const Vec3& torque);

    // Recompute every body's world pose and velocity from the current state (no dynamics).
    void updateKinematics();
    // One step: ABA accelerations, then semi-implicit Euler (velocities, then positions).
    void step(double dt);

    // Helpers (world).
    Vec3 pointVelocity(int body, const Vec3& worldPoint) const;
    Vec3 bodyPos(int body) const { return body < 0 ? base_pos : bodies[body].pos; }
    Quat bodyRot(int body) const { return body < 0 ? base_rot : bodies[body].rot; }
    Vec3 angularVelocity(int body) const;  // world
    // Diagnostics for tests: total linear and angular momentum (about the world origin), kinetic energy.
    Vec3 linearMomentum() const;
    Vec3 angularMomentum() const;
    double kineticEnergy() const;
    double totalMass() const;
    Vec3 centreOfMass() const;
    SVec baseAccel() const { return base_acc_; }

private:
    std::vector<SVec> fext_;  // per body (index 0 = base), body coords
    SVec base_acc_;
};

}  // namespace tws
