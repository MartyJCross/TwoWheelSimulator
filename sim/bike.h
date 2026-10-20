// The motorcycle: an articulated multibody (docs/03_PHYSICS_SPEC.md §4-6) with flexible tyres, suspension,
// drivetrain (engine, clutch, gearbox, chain) and a placeholder rider: one 75 kg body the rider's muscles move
// fore/aft, up/down and side to side relative to the bike. The full articulated rider arrives in M3.
//
// Bike frame axes: +X = rider's left, +Y = up, +Z = forward. The frame's origin is the swingarm pivot.
// Default numbers are a 300cc two-stroke enduro bike (about 104 kg dry plus fuel, 1480 mm wheelbase,
// 26.5 deg rake, 300 mm travel).
#pragma once
#include <cstdint>
#include <vector>

#include "articulated.h"
#include "terrain.h"
#include "tyre.h"

namespace tws {

struct BikeInput {
    double throttle = 0;      // 0..1
    double front_brake = 0;   // 0..1
    double rear_brake = 0;    // 0..1
    double clutch = 0;        // lever: 0 = released (engaged), 1 = pulled in (disengaged)
    double steer = 0;         // -1..1, + = right. Lean-intent: the lean you want; bar-torque mode: torque on the bars
    double body_fore = 0;     // -1 = back over the rear wheel .. +1 = forward over the bars
    double body_height = 0;   // -1 = crouch .. +1 = stand tall / extend
    double body_side = 0;     // -1 = left .. +1 = right (relative to the bike)
    bool shift_up = false;    // edge-triggered (true for one step)
    bool shift_down = false;
    bool start_engine = false;
    bool dab_left = false, dab_right = false;  // put a foot down
};

struct BikeAssists {
    bool lean_intent = true;   // stick = target lean (the rider's brain countersteers); off = raw bar torque
    bool auto_clutch = true;   // clutch slips/engages by rpm (like a real auto-clutch)
    bool auto_dab = true;      // feet go down below walking pace
};

struct EngineSpec {
    double inertia = 0.012;       // crank + flywheel, kg m^2 (stock 300 two-stroke; a flywheel weight adds ~0.004)
    double idle_rpm = 1600;
    double stall_rpm = 650;
    double limiter_rpm = 9800;
    // Full-throttle torque (N*m) at 1000, 2000 .. 10000 rpm (300cc two-stroke dyno shape).
    double torque[10] = {14, 22, 30, 36, 40, 42, 41, 37, 30, 18};
    double primary = 2.769;                                        // crank -> clutch basket
    double gears[6] = {2.40, 1.875, 1.40, 1.13, 0.92, 0.77};
    double front_sprocket = 13, rear_sprocket = 50;
    double clutch_capacity = 140;  // N*m at the crank, fully engaged
};

struct Telemetry {
    double speed = 0;        // m/s, forward
    double rpm = 0;
    int gear = 1;            // 0 = neutral
    bool engine_running = true;
    double clutch_engaged = 1;  // 0..1, what the clutch can transmit
    bool clutch_slipping = false;
    double lean_deg = 0;     // + = leaning right
    double pitch_deg = 0;    // + = nose up
    double steer_deg = 0;    // + = steered left
    double fork_travel = 0, shock_travel = 0;  // m (wheel travel for the shock)
    double front_load = 0, rear_load = 0;      // N, tyre normal load
    bool front_contact = false, rear_contact = false;
    bool crashed = false;
};

class Bike {
public:
    Bike();

    // Place the bike upright at (x, z) on the ground, facing +Z rotated by heading, at rest, engine idling in 1st.
    void reset(const Terrain& terrain, double x, double z, double headingDeg);
    void step(const Terrain& terrain, const BikeInput& in, double dt);
    // Set the bike rolling straight ahead at `speed` (m/s): wheels spinning to match, a gear picked so the engine is
    // in its mid range, clutch locked. Call right after reset().
    void setRolling(double speed);

    BikeAssists assists;
    EngineSpec engine;
    TyreParams front_tyre, rear_tyre;
    TyreState front_state, rear_state;
    TyreOutput front_out, rear_out;
    Telemetry tele;

    Articulated mb;  // the multibody
    // Body indices in mb.bodies.
    static constexpr int kSteer = 0, kFork = 1, kFrontWheel = 2, kSwingarm = 3, kRearWheel = 4, kRiderZ = 5,
                         kRiderY = 6, kRider = 7;

    // Geometry (frame coords), for the renderer.
    Vec3 head_pos;          // steering head (top of the steering axis)
    Vec3 steer_axis;        // unit, pointing up the steering axis
    Vec3 rider_base;        // rider's neutral centre of mass
    Vec3 countershaft;      // front sprocket centre
    double fork_length = 0.785;  // head to axle, fully extended
    double swingarm_length = 0.58;
    double front_sprocket_r = 0, rear_sprocket_r = 0;
    bool dab_active_left = false, dab_active_right = false;

    Vec3 position() const { return mb.base_pos; }
    Quat rotation() const { return mb.base_rot; }
    Vec3 velocity() const { return mb.pointVelocity(-1, mb.base_pos); }
    Vec3 hub(int wheelBody) const { return mb.bodies[wheelBody].pos; }

    uint64_t stateHash() const;

private:
    struct Contact {
        int body;
        Vec3 local;    // in the body's frame
        double radius;
        bool crash;    // touching the ground here means the rider is down
        int dab = 0;   // -1 / +1: a foot (only active while dabbing)
    };
    std::vector<Contact> contacts_;
    std::vector<int> scratch_;

    // Drivetrain state.
    double crank_w_ = 0;       // rad/s
    double out_w_ = 0;         // gearbox output speed, in rear-wheel rad/s
    double twist_ = 0;         // driveline wind-up at the wheel (rad)
    int gear_ = 1;
    bool running_ = true;
    double starter_timer_ = 0;
    double lean_prev_ = 0, lean_rate_ = 0;
    double lean_int_ = 0;      // integral of lean error (the steady bar pressure a rider holds in a turn)

    void tyreForces(const Terrain& t, int wheel, const TyreParams& p, TyreState& s, TyreOutput& o, double dt);
    double drivetrain(const BikeInput& in, double dt);  // returns driveline torque at the rear wheel
    void suspension();
    void riderMuscles(const BikeInput& in);
    double steeringTorque(const BikeInput& in, double dt);
    void bodyContacts(const Terrain& t, const BikeInput& in);
    void brakes(const BikeInput& in, double chainTorque, double dt);
    void updateTelemetry();
};

}  // namespace tws
