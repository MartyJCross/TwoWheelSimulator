// M0 wheel lab: one wheel + tyre on rigid ground, in two modes.
//  - Free: the wheel is a free rigid body. Drop it, roll it, push it; it falls over or rolls on like a real wheel.
//  - Rig:  a "quarter bike" on a flat-track tyre tester. The carriage moves forward at a set speed with a set slip
//          angle and camber. The load sits on a shock above the hub (like a bike's rear end), the hub moves up and
//          down on the tyre, and the wheel spins freely under drive/brake torque. Forces are measured the way a real
//          test machine measures them.
#pragma once
#include <cstdint>
#include <vector>

#include "vecmath.h"
#include "terrain.h"
#include "tyre.h"

namespace tws {

enum class LabMode { Free, Rig };
enum class LabScene { Flat, ObstacleLane };

struct RigSettings {
    double speed = 5.0;        // m/s, carriage speed
    double slip_deg = 0.0;     // slip angle (wheel heading vs. travel), degrees
    double camber_deg = 0.0;   // lean of the wheel, degrees
    double load_kg = 100.0;    // sprung mass carried on the shock above the hub
    double travel = 0.30;      // m of wheel travel
    double sag = 0.10;         // m of static sag (sets the spring rate)
    double damping_ratio = 0.35;
    double drive_torque = 0;   // N*m about the axle (+ drives forward)
    double brake_torque = 0;   // N*m, Coulomb brake (opposes spin, can lock)
};

struct LabMeasure {
    double fx = 0;     // longitudinal force along the wheel heading, ground plane (+ = drive)
    double fy = 0;     // lateral force, ground plane (+ = toward the axle's +X side)
    double fz = 0;     // vertical force on the wheel
    double mz = 0;     // aligning moment about the vertical through the hub
    double slip_ratio = 0;
    double deflection = 0;  // outer radius minus hub height above the ground under it
};

class WheelLab {
public:
    WheelLab();

    void setScene(LabScene scene);
    LabScene scene() const { return scene_; }

    // Free mode: hub at `pos`, rolling forward along +Z at `speed` (spin set to match), leaned by `lean_deg`.
    void resetFree(const Vec3& pos, double speed, double lean_deg, double heading_deg = 0);
    // Rig mode: hub starts just touching the ground at z = 0, wheel spinning to match the carriage.
    void resetRig();

    void step();  // one tick (1/tick_hz)
    void applyImpulse(const Vec3& impulse, const Vec3& at);  // free mode only

    uint64_t stateHash() const;

    static constexpr double kTickHz = 1000.0;
    static constexpr double kDt = 1.0 / kTickHz;

    // Public state (the game reads it; the UI writes params/rig/drive).
    TyreParams params;
    RigSettings rig;
    double free_drive_torque = 0;  // free mode: torque about the axle (no reaction body; lab only)
    double free_brake_torque = 0;

    LabMode mode = LabMode::Rig;
    Vec3 pos;           // hub position
    Vec3 vel;           // hub velocity
    Quat carrier;       // orientation without spin
    Vec3 ang_momentum;  // free mode: world angular momentum of the wheel
    double spin = 0;    // rad/s about the axle
    double spin_angle = 0;
    double rig_z = 0;   // carriage travel (wraps on the obstacle lane)
    double sprung_y = 0, sprung_vy = 0;  // rig: height and vertical speed of the load on the shock
    double time = 0;
    uint64_t ticks = 0;

    TyreState tyre;
    TyreOutput out;
    LabMeasure measure;
    Terrain terrain;

    double lane_length() const { return lane_length_; }

private:
    void buildScene();
    void stepFree();
    void stepRig();
    double applyBrake(double spinNew, double brakeTorque, double inertia) const;

    LabScene scene_ = LabScene::ObstacleLane;
    double lane_length_ = 40.0;
    std::vector<int> scratch_;
};

}  // namespace tws
