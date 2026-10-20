#include "wheel_lab.h"

#include <algorithm>

namespace tws {

WheelLab::WheelLab() {
    tyre.init(params);
    setScene(LabScene::ObstacleLane);
    resetRig();
}

void WheelLab::setScene(LabScene s) {
    scene_ = s;
    buildScene();
}

void WheelLab::buildScene() {
    // 12 m wide, 48 m long, 5 cm cells. The lane runs along +Z at x = 0 and loops every 40 m (flat at both ends).
    const double cell = 0.05;
    terrain.init(-6.0, -4.0, cell, 240, 960, MaterialId::Hardpack);
    if (scene_ == LabScene::Flat) return;

    for (int k = 0; k <= terrain.nz(); ++k) {
        double z = terrain.z0() + k * cell;
        for (int i = 0; i <= terrain.nx(); ++i) {
            // Features stay inside |x| <= 4 m so the grid edges meet the flat world outside without a seam.
            double x = terrain.x0() + i * cell;
            if (std::fabs(x) > 4.0) continue;
            // A 10 cm rock step up at z = 10 and back down at z = 13.
            if (z >= 10.0 - 1e-9 && z <= 13.0 + 1e-9) terrain.h(i, k) = 0.10f;
            if (z >= 9.9 && z <= 13.1) terrain.mat(i, k) = MaterialId::Rock;
            // Loam rollers: 6 cm high, 1.5 m apart, z = 31..37.
            if (z > 31.0 && z < 37.0) {
                terrain.h(i, k) = static_cast<float>(0.03 * (1.0 - std::cos(2.0 * kPi * (z - 31.0) / 1.5)));
                terrain.mat(i, k) = MaterialId::Loam;
            }
        }
    }
    auto& obs = terrain.obstacles();
    obs.push_back({Vec3{-4, 0.012, 17}, Vec3{4, 0.012, 17}, 0.035, MaterialId::WetRoot});  // root across the lane
    obs.push_back({Vec3{-4, 0.11, 22}, Vec3{4, 0.11, 22}, 0.12, MaterialId::Wood});        // 24 cm log
    obs.push_back({Vec3{0.25, 0.0, 27}, Vec3{0.25, 0.0, 27}, 0.28, MaterialId::Rock});     // rock, off-centre
}

void WheelLab::resetFree(const Vec3& p, double speed, double lean_deg, double heading_deg) {
    mode = LabMode::Free;
    tyre.init(params);
    carrier = Quat::axisAngle({0, 1, 0}, deg2rad(heading_deg)) * Quat::axisAngle({0, 0, 1}, deg2rad(lean_deg));
    Vec3 fwd = carrier.rotate({0, 0, 1});
    Vec3 fwdH = normalize(Vec3{fwd.x, 0, fwd.z});
    pos = p;
    vel = fwdH * speed;
    spin = speed / params.outer_radius;
    spin_angle = 0;
    Vec3 axle = carrier.rotate({1, 0, 0});
    ang_momentum = axle * (params.inertia_axle * spin);
    time = 0;
    ticks = 0;
    out = TyreOutput{};
}

void WheelLab::resetRig() {
    mode = LabMode::Rig;
    tyre.init(params);
    rig_z = 0;
    pos = Vec3{0, params.outer_radius - 0.012, 0};
    sprung_y = pos.y + rig.travel - rig.sag;
    sprung_vy = 0;
    vel = Vec3{0, 0, rig.speed};
    spin = rig.speed * std::cos(deg2rad(rig.slip_deg)) / params.outer_radius;
    spin_angle = 0;
    ang_momentum = Vec3{};
    carrier = Quat::axisAngle({0, 1, 0}, deg2rad(rig.slip_deg)) * Quat::axisAngle({0, 0, 1}, deg2rad(rig.camber_deg));
    time = 0;
    ticks = 0;
    out = TyreOutput{};
}

double WheelLab::applyBrake(double s, double brake, double inertia, double dt) const {
    double dw = brake / inertia * dt;
    if (std::fabs(s) <= dw) return 0.0;
    return s > 0 ? s - dw : s + dw;
}

void WheelLab::applyImpulse(const Vec3& j, const Vec3& at) {
    if (mode != LabMode::Free) return;
    vel += j / params.mass;
    ang_momentum += cross(at - pos, j);
}

void WheelLab::step() {
    // Rim strikes and deep compression put many very stiff springs on a light wheel. One 1 ms tick is then close to
    // the explicit integrator's stability limit, so those ticks are split into substeps (most ticks cost one step).
    const int n = (out.rim_strike || out.max_compression > 0.06) ? 4 : 1;
    for (int i = 0; i < n; ++i) {
        if (mode == LabMode::Free)
            stepFree(kDt / n);
        else
            stepRig(kDt / n);
    }
    time += kDt;
    ++ticks;
}

void WheelLab::stepRig(double dt) {
    carrier = Quat::axisAngle({0, 1, 0}, deg2rad(rig.slip_deg)) * Quat::axisAngle({0, 0, 1}, deg2rad(rig.camber_deg));
    vel.x = 0;
    WheelKinematics k{pos, carrier, vel, Vec3{}, spin};
    out = tyreStep(params, tyre, k, terrain, dt, scratch_);

    const Vec3 axle = carrier.rotate({1, 0, 0});
    // Vertical: the load rides on a shock above the hub. Spring rate from the sag; stiff, damped stops at both ends.
    const double ms = std::max(rig.load_kg, 1.0);
    const double ks = ms * kGravity / std::max(rig.sag, 0.01);
    const double c = rig.damping_ratio * 2.0 * std::sqrt(ks * ms);
    const double ext = sprung_y - pos.y;        // shock extension: travel = fully extended, 0 = bottomed
    const double extRate = sprung_vy - vel.y;
    const double extStatic = rig.travel - rig.sag;
    double fs = ks * (extStatic + ms * kGravity / ks - ext) - c * extRate;  // pushes load up, hub down
    const double kStop = 2.0e6, cStop = 2.0e4;
    if (ext > rig.travel) fs -= kStop * (ext - rig.travel) + cStop * extRate;
    if (ext < 0) fs += kStop * (-ext) - cStop * extRate;
    vel.y += ((out.force.y - fs) / params.mass - kGravity) * dt;
    pos.y += vel.y * dt;
    sprung_vy += (fs / ms - kGravity) * dt;
    sprung_y += sprung_vy * dt;
    // Spin: tyre torque about the axle + drive, then the brake.
    double s = spin + (dot(out.torque, axle) + rig.drive_torque) / params.inertia_axle * dt;
    spin = applyBrake(s, rig.brake_torque, params.inertia_axle, dt);
    spin_angle += spin * dt;
    // Carriage.
    // Carriage: the whole quarter bike has momentum along the lane and is towed toward the set speed by a limited
    // force (up to 1 g), so a hard hit slows it the way it would slow a real bike. (A carriage that held its speed
    // with unlimited force flung the wheel straight up off a log at speed.)
    const double mTotal = params.mass + std::max(rig.load_kg, 1.0);
    double tow = clampd(mTotal * (rig.speed - vel.z) / 0.25, -mTotal * kGravity, mTotal * kGravity);
    vel.z += (out.force.z + tow) / mTotal * dt;
    rig_z += vel.z * dt;
    if (rig_z >= lane_length_) rig_z -= lane_length_;
    if (rig_z < -2.0) rig_z += lane_length_;
    pos.x = 0;
    pos.z = rig_z;

    Vec3 fwd = carrier.rotate({0, 0, 1});
    Vec3 head = normalize(Vec3{fwd.x, 0, fwd.z});
    Vec3 side = normalize(Vec3{axle.x, 0, axle.z});
    measure.fx = dot(out.force, head);
    measure.fy = dot(out.force, side);
    measure.fz = out.force.y;
    measure.mz = out.torque.y;
    double vlong = dot(vel, head);
    measure.slip_ratio = (spin * out.effective_radius - vlong) / std::max(std::fabs(vlong), 0.1);
    measure.deflection = params.outer_radius - (pos.y - terrain.heightAt(pos.x, pos.z));
}

void WheelLab::stepFree(double dt) {
    const double ia = params.inertia_axle, it = params.inertia_transverse;
    auto invInertia = [&](const Vec3& axle, const Vec3& v) {
        return v / it + axle * (dot(axle, v) * (1.0 / ia - 1.0 / it));
    };
    Vec3 axle = carrier.rotate({1, 0, 0});
    Vec3 omega = invInertia(axle, ang_momentum);
    spin = dot(omega, axle);
    Vec3 carrierOmega = omega - axle * spin;

    WheelKinematics k{pos, carrier, vel, carrierOmega, spin};
    out = tyreStep(params, tyre, k, terrain, dt, scratch_);

    vel += (out.force / params.mass + Vec3{0, -kGravity, 0}) * dt;
    pos += vel * dt;
    ang_momentum += (out.torque + axle * free_drive_torque) * dt;

    // Brake about the axle (lab only: it reacts against nothing).
    omega = invInertia(axle, ang_momentum);
    double s = dot(omega, axle);
    double sb = applyBrake(s, free_brake_torque, ia, dt);
    ang_momentum += axle * (ia * (sb - s));

    omega = invInertia(axle, ang_momentum);
    spin = dot(omega, axle);
    carrier = carrier.integrated(omega - axle * spin, dt);
    spin_angle += spin * dt;

    measure = LabMeasure{};
    measure.fz = out.force.y;
    measure.deflection = out.max_compression;
}

uint64_t WheelLab::stateHash() const {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](const void* data, size_t n) {
        const unsigned char* b = static_cast<const unsigned char*>(data);
        for (size_t i = 0; i < n; ++i) {
            h ^= b[i];
            h *= 1099511628211ull;
        }
    };
    mix(&pos, sizeof pos);
    mix(&vel, sizeof vel);
    mix(&carrier, sizeof carrier);
    mix(&ang_momentum, sizeof ang_momentum);
    mix(&spin, sizeof spin);
    mix(&rig_z, sizeof rig_z);
    mix(&sprung_y, sizeof sprung_y);
    mix(tyre.d_long.data(), tyre.d_long.size() * sizeof(double));
    mix(tyre.d_lat.data(), tyre.d_lat.size() * sizeof(double));
    return h;
}

}  // namespace tws
