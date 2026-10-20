// Stylised bike and rider built from simple shapes, placed from the sim's body poses each frame.
#include <cmath>

#include "bike.h"
#include "render.h"

using tws::Bike;
using tws::Quat;
using tws::Vec3;

namespace {

const Vec3 kOrange{0.95, 0.45, 0.08}, kBlack{0.08, 0.08, 0.09}, kWhite{0.90, 0.90, 0.88}, kSilver{0.70, 0.72, 0.75},
    kGold{0.85, 0.62, 0.18}, kDark{0.22, 0.22, 0.24}, kJersey{0.16, 0.36, 0.86}, kPants{0.20, 0.22, 0.26},
    kHelmet{0.98, 0.82, 0.10}, kVisor{0.10, 0.12, 0.15}, kBoot{0.85, 0.85, 0.83};

// Two-bone IK: joint position between a and c for bone lengths l1, l2, bending toward `pole`.
Vec3 ikJoint(const Vec3& a, const Vec3& c, double l1, double l2, const Vec3& pole) {
    Vec3 d = c - a;
    double len = std::max(1e-6, std::min(tws::length(d), l1 + l2 - 1e-4));
    Vec3 dir = tws::normalize(d);
    double along = (l1 * l1 - l2 * l2 + len * len) / (2 * len);
    double h = std::sqrt(std::max(l1 * l1 - along * along, 0.0));
    Vec3 p = pole - dir * tws::dot(pole, dir);
    p = tws::length(p) > 1e-6 ? tws::normalize(p) : tws::anyPerpendicular(dir);
    return a + dir * along + p * h;
}

}  // namespace

void buildBikeMesh(const Bike& bike, const Vec3& offset, bool contactColours, MeshBuilder& out) {
    const tws::Articulated& mb = bike.mb;
    const Quat R = mb.base_rot;
    const Vec3 O = mb.base_pos + offset;
    auto F = [&](const Vec3& l) { return O + R.rotate(l); };  // frame local -> world
    const tws::Body& steer = mb.bodies[Bike::kSteer];
    const tws::Body& fork = mb.bodies[Bike::kFork];
    const tws::Body& arm = mb.bodies[Bike::kSwingarm];
    const tws::Body& rider = mb.bodies[Bike::kRider];
    auto S = [&](const Vec3& l) { return steer.pos + offset + steer.rot.rotate(l); };  // steering, origin at the head
    auto K = [&](const Vec3& l) { return fork.pos + offset + fork.rot.rotate(l); };    // fork lower, origin at the axle
    auto A = [&](const Vec3& l) { return arm.pos + offset + arm.rot.rotate(l); };      // swingarm, origin at the pivot
    auto P = [&](const Vec3& l) { return rider.pos + offset + rider.rot.rotate(l); };  // rider, origin at the COM

    // Wheels.
    buildWheel(bike.front_tyre, bike.front_state, mb.bodies[Bike::kFrontWheel].pos, fork.rot,
               mb.bodies[Bike::kFrontWheel].q, offset, contactColours, out);
    buildWheel(bike.rear_tyre, bike.rear_state, mb.bodies[Bike::kRearWheel].pos, arm.rot, mb.bodies[Bike::kRearWheel].q,
               offset, contactColours, out);

    // Fork: upper tubes on the steering, lower legs on the slider.
    const Vec3 ax = bike.steer_axis;
    const Vec3 perp{0, std::sin(26.5 * tws::kPi / 180), std::cos(26.5 * tws::kPi / 180)};
    for (double sx : {0.09, -0.09}) {
        Vec3 side{sx, 0, 0};
        out.cylinder(S(side + ax * 0.06 + perp * 0.02), S(side - ax * 0.45 + perp * 0.03), 0.026, kDark, 10);
        out.cylinder(K(side + ax * 0.02), K(side + ax * 0.40), 0.033, kGold, 10);
    }
    out.box(S(ax * 0.04 + perp * 0.02), {0.12, 0.018, 0.045}, steer.rot, kSilver);          // top clamp
    out.box(S(-ax * 0.17 + perp * 0.025), {0.12, 0.022, 0.05}, steer.rot, kSilver);         // bottom clamp
    out.cylinder(S({-0.40, 0.17, -0.10}), S({0.40, 0.17, -0.10}), 0.014, kSilver, 8);       // bars
    out.cylinder(S({0.29, 0.17, -0.10}), S({0.42, 0.17, -0.10}), 0.02, kBlack, 8);          // grips
    out.cylinder(S({-0.29, 0.17, -0.10}), S({-0.42, 0.17, -0.10}), 0.02, kBlack, 8);
    out.cylinder(S({0, 0.04, -0.01}), S({0, 0.17, -0.07}), 0.02, kSilver, 8);               // bar mount
    Quat plateRot = steer.rot * Quat::axisAngle({1, 0, 0}, -26.5 * tws::kPi / 180);
    out.box(S(perp * 0.10 - ax * 0.05), {0.13, 0.11, 0.012}, plateRot, kWhite);             // number plate
    out.box(S(perp * 0.20 - ax * 0.24), {0.07, 0.012, 0.26}, steer.rot, kWhite);            // front fender

    // Frame, engine, bodywork (frame coords: origin at the swingarm pivot, +Z forward, +X left).
    const Vec3 head = bike.head_pos;
    for (double sx : {0.11, -0.11}) {
        out.cylinder(F(head - ax * 0.05), F({sx, 0.25, 0.02}), 0.024, kSilver, 8);   // main spars
        out.cylinder(F({sx, 0.25, 0.02}), F({sx, -0.05, 0.0}), 0.024, kSilver, 8);   // down to the pivot
        out.cylinder(F({sx, 0.25, 0.02}), F({sx * 0.8, 0.45, -0.55}), 0.016, kSilver, 8);  // subframe
    }
    out.cylinder(F(head - ax * 0.2), F({0, -0.16, 0.42}), 0.026, kSilver, 8);       // down tube
    out.box(F({0, 0.02, 0.27}), {0.12, 0.16, 0.19}, R, kDark);                      // engine cases
    out.box(F({0, 0.24, 0.36}), {0.07, 0.10, 0.07}, R * Quat::axisAngle({1, 0, 0}, -0.3), kSilver);  // cylinder
    out.cylinder(F({0.02, 0.30, 0.44}), F({0.10, -0.08, 0.62}), 0.04, kDark, 10);  // expansion chamber (pipe)
    out.cylinder(F({-0.14, 0.30, -0.22}), F({-0.15, 0.40, -0.62}), 0.05, kSilver, 10);  // silencer
    out.box(F({0, 0.46, 0.30}), {0.14, 0.08, 0.20}, R, kOrange);                    // tank
    for (double sx : {0.16, -0.16})
        out.box(F({sx, 0.38, 0.40}), {0.025, 0.12, 0.16}, R, kOrange);              // shrouds / radiators
    out.box(F({0, 0.50, -0.16}), {0.11, 0.035, 0.40}, R * Quat::axisAngle({1, 0, 0}, 0.05), kBlack);  // seat
    for (double sx : {0.13, -0.13})
        out.box(F({sx, 0.36, -0.32}), {0.02, 0.09, 0.18}, R, kWhite);                 // side panels
    out.box(F({0, 0.45, -0.68}), {0.10, 0.012, 0.22}, R * Quat::axisAngle({1, 0, 0}, 0.25), kOrange);  // rear fender
    out.box(F({0, -0.13, 0.30}), {0.10, 0.015, 0.18}, R, kSilver);                  // skid plate

    // Swingarm and shock.
    const Vec3 axle = mb.bodies[Bike::kRearWheel].joint_pos;  // swingarm frame
    for (double sx : {0.075, -0.075}) out.cylinder(A({sx, 0, 0}), A(Vec3{sx, 0, 0} + axle), 0.026, kSilver, 8);
    out.cylinder(F({0, 0.32, -0.04}), A(axle * 0.32 + Vec3{0, 0.06, 0}), 0.034, kGold, 10);  // shock spring
    // Chain (top run).
    out.cylinder(F(bike.countershaft + Vec3{0.09, bike.front_sprocket_r, 0}),
                 mb.bodies[Bike::kRearWheel].pos + offset + arm.rot.rotate({0.09, bike.rear_sprocket_r, 0}), 0.008,
                 kDark, 5);

    // Rider: torso, head, arms to the grips, legs to the pegs (or the ground while dabbing).
    const Vec3 hip = P({0, -0.12, -0.08});
    const Vec3 chest = P({0, 0.30, 0.12});
    out.capsule(hip, chest, 0.15, kJersey, 14);
    out.capsule(P({0, 0.62, 0.22}), P({0, 0.62, 0.22}), 0.13, kHelmet, 16);          // helmet
    out.box(P({0, 0.60, 0.33}), {0.08, 0.04, 0.03}, rider.rot, kVisor);              // goggles
    out.box(P({0, 0.70, 0.30}), {0.10, 0.012, 0.07}, rider.rot * Quat::axisAngle({1, 0, 0}, -0.3), kOrange);  // peak
    const Vec3 up = R.rotate({0, 1, 0}), fwd = R.rotate({0, 0, 1});
    for (int side = 0; side < 2; ++side) {
        double sx = side ? 1.0 : -1.0;
        Vec3 lateral = R.rotate({sx, 0, 0});
        // Arm: shoulder -> elbow -> grip.
        Vec3 shoulder = P({sx * 0.19, 0.40, 0.14});
        Vec3 grip = S({sx * 0.34, 0.17, -0.10});
        Vec3 elbow = ikJoint(shoulder, grip, 0.31, 0.31, lateral * 0.8 - up * 0.4 - fwd * 0.2);
        out.capsule(shoulder, elbow, 0.05, kJersey, 10);
        out.capsule(elbow, grip, 0.045, kJersey, 10);
        out.capsule(grip, grip, 0.04, kBlack, 8);
        // Leg: hip -> knee -> foot (on the peg, or on the ground when dabbing).
        Vec3 hipJ = hip + lateral * 0.11;
        bool dab = side ? bike.dab_active_left : bike.dab_active_right;
        Vec3 foot = dab ? F({sx * 0.40, -0.40, 0.10}) : F({sx * 0.17, -0.09, 0.08});
        Vec3 knee = ikJoint(hipJ, foot, 0.46, 0.46, fwd * 1.0 + lateral * 0.25);
        out.capsule(hipJ, knee, 0.065, kPants, 10);
        out.capsule(knee, foot + up * 0.06, 0.055, kBoot, 10);
        out.box(foot + fwd * 0.05, {0.05, 0.04, 0.13}, R, kBoot);
    }
}
