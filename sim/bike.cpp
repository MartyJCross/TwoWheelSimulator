#include "bike.h"

#include <algorithm>
#include <cmath>

namespace tws {

namespace {

constexpr double kRake = 26.5 * kPi / 180.0;
constexpr double kForkOffset = 0.040;      // m, axle ahead of the steering axis
constexpr double kForkTravel = 0.300;
constexpr double kSwingarmDroop = 24.4 * kPi / 180.0;  // swingarm angle below horizontal at full extension
constexpr double kRearTravel = 0.310;
constexpr double kRiderMass = 75.0;

double smoothstep(double e0, double e1, double x) {
    double t = clampd((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3 - 2 * t);
}

// Speed-sensitive damper with a high-speed blow-off knee at 0.5 m/s.
double damper(double v, double cComp, double cReb) {
    double c = v > 0 ? cComp : cReb;
    double a = std::fabs(v);
    double f = a < 0.5 ? c * a : c * 0.5 + 0.35 * c * (a - 0.5);
    return v > 0 ? f : -f;
}

double sprocketRadius(double teeth) { return 0.015875 / (2.0 * std::sin(kPi / teeth)); }  // 520 chain

}  // namespace

Bike::Bike() {
    // Tyres: 80/100-21 front, 120/90-18 rear, both at 13 psi.
    front_tyre.outer_radius = 0.347;
    front_tyre.rim_radius = 0.2667;
    front_tyre.crown_radius = 0.040;
    front_tyre.bottom_margin = 0.020;
    front_tyre.pressure = 13.0 * 6894.76;
    rear_tyre.pressure = 13.0 * 6894.76;

    steer_axis = Vec3{0, std::cos(kRake), -std::sin(kRake)};
    const Vec3 perp{0, std::sin(kRake), std::cos(kRake)};
    head_pos = Vec3{0, 0.520, 0.547};
    rider_base = Vec3{0, 0.82, -0.05};  // attack position: rider's centre of mass ~1.3 m above the ground
    countershaft = Vec3{0, 0.07, 0.075};
    front_sprocket_r = sprocketRadius(engine.front_sprocket);
    rear_sprocket_r = sprocketRadius(engine.rear_sprocket);

    // Frame + engine + tank + fuel: everything not listed below (bike ~112 kg wet in total).
    mb.base_mass = 73.0;
    mb.base_com = Vec3{0, 0.10, 0.25};
    mb.base_inertia = Mat3::diag(8.0, 9.0, 3.0);

    Body steer;  // bars, triple clamps, upper fork tubes
    steer.parent = -1;
    steer.type = JointType::Revolute;
    steer.axis = steer_axis;
    steer.joint_pos = head_pos;
    steer.mass = 6.0;
    steer.com = Vec3{0, 0.03, 0.0};
    steer.inertia = Mat3::diag(0.15, 0.08, 0.12);
    mb.bodies.push_back(steer);

    Body fork;  // lower legs, brake caliper, axle; prismatic along the steering axis (+ = compressing)
    fork.parent = kSteer;
    fork.type = JointType::Prismatic;
    fork.axis = steer_axis;
    fork.joint_pos = perp * kForkOffset - steer_axis * fork_length;
    fork.mass = 4.0;
    fork.com = steer_axis * 0.15;
    fork.inertia = Mat3::diag(0.05, 0.01, 0.05);
    mb.bodies.push_back(fork);

    Body front;  // 21" wheel with tyre and tube
    front.parent = kFork;
    front.type = JointType::Revolute;
    front.axis = Vec3{1, 0, 0};
    front.mass = 10.0;
    front.inertia = Mat3::diag(0.62, 0.33, 0.33);
    mb.bodies.push_back(front);

    Body arm;  // swingarm, pivoting at the frame origin; + = axle moving up
    arm.parent = -1;
    arm.type = JointType::Revolute;
    arm.axis = Vec3{1, 0, 0};
    const Vec3 axleLocal{0, -swingarm_length * std::sin(kSwingarmDroop), -swingarm_length * std::cos(kSwingarmDroop)};
    arm.mass = 5.0;
    arm.com = axleLocal * 0.5;
    arm.inertia = Mat3::diag(0.20, 0.20, 0.02);
    mb.bodies.push_back(arm);

    Body rear;  // 18" wheel with tyre, tube, sprocket, disc
    rear.parent = kSwingarm;
    rear.type = JointType::Revolute;
    rear.axis = Vec3{1, 0, 0};
    rear.joint_pos = axleLocal;
    rear.mass = 12.0;
    rear.inertia = Mat3::diag(0.85, 0.45, 0.45);
    mb.bodies.push_back(rear);

    // Rider: two massless sliders and the rider's mass (fore/aft, up/down, side to side, in frame axes).
    Body rz;
    rz.parent = -1;
    rz.type = JointType::Prismatic;
    rz.axis = Vec3{0, 0, 1};
    rz.joint_pos = rider_base;
    mb.bodies.push_back(rz);
    Body ry;
    ry.parent = kRiderZ;
    ry.type = JointType::Prismatic;
    ry.axis = Vec3{0, 1, 0};
    mb.bodies.push_back(ry);
    Body rider;
    rider.parent = kRiderY;
    rider.type = JointType::Prismatic;
    rider.axis = Vec3{1, 0, 0};
    rider.mass = kRiderMass;
    rider.inertia = Mat3::diag(9.0, 2.5, 9.0);
    mb.bodies.push_back(rider);

    // Things that can touch the ground other than the tyres.
    contacts_ = {
        {-1, {0.17, -0.13, 0.10}, 0.03, false},  {-1, {-0.17, -0.13, 0.10}, 0.03, false},   // footpegs
        {-1, {0, -0.10, 0.28}, 0.10, false},                                                // skid plate
        {-1, {0, 0.0, 0.50}, 0.08, false},                                                  // frame, front
        {-1, {0, 0.42, -0.60}, 0.07, false},                                                // rear fender
        {-1, {0.15, 0.30, -0.25}, 0.06, false},  {-1, {-0.15, 0.30, -0.25}, 0.06, false},   // side panels
        {-1, {0.20, 0.38, 0.40}, 0.06, false},   {-1, {-0.20, 0.38, 0.40}, 0.06, false},    // shrouds
        {kSteer, {0.40, 0.17, -0.10}, 0.035, false}, {kSteer, {-0.40, 0.17, -0.10}, 0.035, false},  // bar ends
        {kRider, {0, 0.62, 0.05}, 0.12, true},                                              // head
        {kRider, {0.20, 0.42, 0}, 0.08, true},   {kRider, {-0.20, 0.42, 0}, 0.08, true},    // shoulders
        {kRider, {0, -0.05, -0.05}, 0.14, true},                                            // hips
        {-1, {0.40, -0.43, 0.10}, 0.05, false, 1},  {-1, {-0.40, -0.43, 0.10}, 0.05, false, -1},  // feet (dab)
    };

    front_state.init(front_tyre);
    rear_state.init(rear_tyre);
}

void Bike::reset(const Terrain& terrain, double x, double z, double headingDeg) {
    for (Body& b : mb.bodies) {
        b.q = b.qd = b.qdd = b.tau = 0;
    }
    mb.bodies[kFork].q = 0.05;       // near static sag
    mb.bodies[kSwingarm].q = 0.18;
    mb.base_rot = Quat::axisAngle({0, 1, 0}, deg2rad(headingDeg));
    mb.base_vel = SVec{};
    mb.base_pos = Vec3{x, 0, z};
    mb.updateKinematics();
    // Drop it so the lower tyre just touches the ground.
    double gap = 1e9;
    for (int w : {kFrontWheel, kRearWheel}) {
        const Vec3 h = mb.bodies[w].pos;
        double r = (w == kFrontWheel ? front_tyre : rear_tyre).outer_radius;
        gap = std::min(gap, h.y - r - terrain.heightAt(h.x, h.z));
    }
    mb.base_pos.y -= gap + 0.012;
    mb.updateKinematics();
    // Feet: just above the ground when the bike stands upright.
    double pivotHeight = mb.base_pos.y - terrain.heightAt(mb.base_pos.x, mb.base_pos.z);
    for (Contact& c : contacts_)
        if (c.dab) c.local.y = -pivotHeight + c.radius + 0.01;

    front_state.init(front_tyre);
    rear_state.init(rear_tyre);
    front_out = rear_out = TyreOutput{};
    crank_w_ = engine.idle_rpm * 2 * kPi / 60;
    out_w_ = 0;
    twist_ = 0;
    gear_ = 1;
    running_ = true;
    starter_timer_ = 0;
    lean_prev_ = lean_rate_ = 0;
    lean_int_ = 0;
    tele = Telemetry{};
    updateTelemetry();
}

void Bike::setRolling(double speed) {
    const Vec3 fwd = mb.base_rot.rotate({0, 0, 1});
    mb.base_vel.lin = mb.base_rot.conj().rotate(fwd * speed);
    mb.bodies[kFrontWheel].qd = speed / front_tyre.outer_radius;
    mb.bodies[kRearWheel].qd = speed / rear_tyre.outer_radius;
    const double wheelW = mb.bodies[kRearWheel].qd;
    gear_ = 1;
    for (int g = 1; g <= 6; ++g) {
        gear_ = g;
        double N = engine.primary * engine.gears[g - 1] * engine.rear_sprocket / engine.front_sprocket;
        if (wheelW * N * 60.0 / (2 * kPi) < 6500.0) break;
    }
    const double N = engine.primary * engine.gears[gear_ - 1] * engine.rear_sprocket / engine.front_sprocket;
    out_w_ = wheelW;
    crank_w_ = std::max(wheelW * N, engine.idle_rpm * 2 * kPi / 60);
    twist_ = 0;
    mb.updateKinematics();
    updateTelemetry();
}

void Bike::tyreForces(const Terrain& t, int wheel, const TyreParams& p, TyreState& s, TyreOutput& o, double dt) {
    const Body& w = mb.bodies[wheel];
    const int parent = w.parent;
    const Quat carrier = mb.bodies[parent].rot;
    const Vec3 axle = carrier.rotate({1, 0, 0});
    const Vec3 wp = mb.angularVelocity(parent);
    WheelKinematics k;
    k.hub = w.pos;
    k.carrier = carrier;
    k.velocity = mb.pointVelocity(wheel, w.pos);
    k.carrier_omega = wp - axle * dot(wp, axle);
    k.spin = dot(wp, axle) + w.qd;
    k.tread_rate = w.qd;
    o = tyreStep(p, s, k, t, dt, scratch_);
    mb.addForce(wheel, o.force, w.pos);
    mb.addTorque(wheel, o.torque);
}

void Bike::suspension() {
    // Fork: coil springs with preload, progressive hydraulic bottoming, top-out, stiction. + q = compression.
    Body& f = mb.bodies[kFork];
    double fq = f.q, fv = f.qd;
    double force = 230.0 + 9000.0 * fq + damper(fv, 1500.0, 1800.0) + 35.0 * std::tanh(fv / 0.01);
    if (fq > kForkTravel - 0.03) force += 2.0e5 * (fq - (kForkTravel - 0.03)) + 3000.0 * std::max(fv, 0.0);
    if (fq < 0) force += 2.0e5 * fq + 2000.0 * std::min(fv, 0.0);
    f.tau = -force;

    // Shock through the linkage, expressed as a progressive wheel rate.
    Body& a = mb.bodies[kSwingarm];
    double L = swingarm_length;
    double w = L * (std::sin(kSwingarmDroop) - std::sin(kSwingarmDroop - a.q));
    double dwdq = L * std::cos(kSwingarmDroop - a.q);
    double wv = dwdq * a.qd;
    double fw = 150.0 + 6000.0 * w + 66000.0 * w * w * w + damper(wv, 2500.0, 3000.0) + 25.0 * std::tanh(wv / 0.01);
    if (w > kRearTravel - 0.02) fw += 3.0e5 * (w - (kRearTravel - 0.02)) + 4000.0 * std::max(wv, 0.0);
    if (w < 0) fw += 3.0e5 * w + 3000.0 * std::min(wv, 0.0);
    a.tau = -fw * dwdq;
}

void Bike::riderMuscles(const BikeInput& in) {
    // Targets relative to the neutral attack position. Muscles are stiff springs with force limits, plus the force
    // needed to hold the rider's own weight along each axis (so posture doesn't sag; inertia still pushes them).
    const double tz = 0.32 * clampd(in.body_fore, -1, 1);
    const double ty = in.body_height >= 0 ? 0.12 * std::min(in.body_height, 1.0) : 0.25 * std::max(in.body_height, -1.0);
    const double tx = -0.25 * clampd(in.body_side, -1, 1);  // + side = right = -X
    struct Axis { int body; double target, kp, kd, fmax; };
    const Axis axes[3] = {{kRiderZ, tz, 9000.0, 1600.0, 1800.0}, {kRiderY, ty, 5000.0, 900.0, 2500.0},
                          {kRider, tx, 2500.0, 500.0, 900.0}};
    for (const Axis& ax : axes) {
        Body& b = mb.bodies[ax.body];
        Vec3 worldAxis = mb.base_rot.rotate(b.axis);
        double hold = -kRiderMass * dot(mb.gravity, worldAxis);
        b.tau = clampd(ax.kp * (ax.target - b.q) - ax.kd * b.qd + hold, -ax.fmax, ax.fmax);
    }
}

double Bike::steeringTorque(const BikeInput& in, double dt) {
    const Body& s = mb.bodies[kSteer];
    const Vec3 fwd = mb.base_rot.rotate({0, 0, 1});
    const Vec3 left = mb.base_rot.rotate({1, 0, 0});
    const double lean = std::asin(clampd(left.y, -1, 1));                // + = leaning right
    const double leanRate = dot(mb.angularVelocity(-1), fwd);           // + = rolling right
    const double v = std::max(0.0, dot(velocity(), fwd));
    double torque = 0;
    const double stick = clampd(in.steer, -1, 1);
    if (assists.lean_intent) {
        // The rider's brain: hold the lean the stick asks for by steering into the lean error (countersteering
        // emerges: to lean right from upright, it first steers left). Blends to direct steering at walking pace.
        const double maxLean = deg2rad(48.0);
        const double leanRef = stick * maxLean;
        const double kp = 45.0, kd = 9.0;
        const double w = smoothstep(1.0, 4.0, v);
        // Integral: a steady turn needs steady bar pressure; without it the lean would settle short of the target.
        lean_int_ = clampd(lean_int_ + (lean - leanRef) * dt * w, -0.6, 0.6);
        const double ki = 40.0;
        const double tLean = -kp * (lean - leanRef) - kd * leanRate - ki * lean_int_;
        const double steerTarget = -stick * deg2rad(30.0);
        const double tDirect = 60.0 * (steerTarget - s.q) - 6.0 * s.qd;
        torque = w * tLean + (1.0 - w) * tDirect;
    } else {
        torque = -stick * 30.0;  // raw bar torque: + stick (right) turns the bars right
    }
    // Steering damper and lock stops.
    torque -= 2.0 * s.qd;
    const double lock = deg2rad(32.0);
    if (s.q > lock) torque -= 3000.0 * (s.q - lock) + 30.0 * s.qd;
    if (s.q < -lock) torque -= 3000.0 * (s.q + lock) + 30.0 * s.qd;
    return torque;
}

double Bike::drivetrain(const BikeInput& in, double dt) {
    const EngineSpec& e = engine;
    // Gear changes: sequential, neutral between 1st and 2nd (1-N-2-3-4-5-6).
    static const int order[7] = {1, 0, 2, 3, 4, 5, 6};
    int idx = 0;
    for (int i = 0; i < 7; ++i)
        if (order[i] == gear_) idx = i;
    // Neutral is a half-click up from 1st: only found when (nearly) stopped and off the throttle; on the move an
    // upshift from 1st goes straight to 2nd.
    const double groundSpeed = std::fabs(mb.bodies[kRearWheel].qd) * rear_tyre.outer_radius;
    if (in.shift_up && idx < 6) gear_ = (gear_ == 1 && (groundSpeed > 1.0 || in.throttle > 0.05)) ? 2 : order[idx + 1];
    if (in.shift_down && idx > 0) gear_ = order[idx - 1];

    double rpm = crank_w_ * 60.0 / (2 * kPi);
    // Engine torque: throttle (with an idle governor), drag when closed, starter, limiter, stall.
    if (in.start_engine && !running_) starter_timer_ = 0.8;
    double te;
    if (running_) {
        double idleThr = clampd(0.23 + (e.idle_rpm - rpm) * 0.0008, 0.0, 0.6);
        double thr = std::max(clampd(in.throttle, 0, 1), idleThr);
        if (rpm > e.limiter_rpm) thr = 0;
        double fr = clampd(rpm / 1000.0 - 1.0, 0.0, 8.999);
        int i0 = static_cast<int>(fr);
        double full = e.torque[i0] + (e.torque[i0 + 1] - e.torque[i0]) * (fr - i0);
        if (rpm < 1000) full = e.torque[0] * std::max(rpm, 0.0) / 1000.0;
        te = thr * full - (1.0 - thr) * (2.0 + 0.0015 * rpm) - 1.0;
        if (rpm < e.stall_rpm) running_ = false;
    } else {
        te = -(3.0 + 0.002 * std::fabs(rpm)) * (crank_w_ > 0 ? 1.0 : -1.0) * std::min(1.0, std::fabs(crank_w_) / 5.0);
        if (starter_timer_ > 0) {
            starter_timer_ -= dt;
            te += 15.0;
            if (rpm > 900) {
                running_ = true;
                starter_timer_ = 0;
            }
        }
    }

    // Clutch: lever -> capacity. Auto-clutch engages with rpm (and the lever can still override).
    double lever = clampd(in.clutch, 0, 1);
    if (assists.auto_clutch) lever = std::max(lever, 1.0 - smoothstep(1900.0, 3300.0, rpm));
    const double capacity = e.clutch_capacity * (1.0 - smoothstep(0.15, 0.75, lever));
    tele.clutch_engaged = capacity / e.clutch_capacity;

    // Driveline (chain, cush drive, tyre carcass in torsion) as a stiff torsional spring at the rear wheel.
    const double wheelW = mb.bodies[kRearWheel].qd;
    const double kDl = 12000.0, cDl = 40.0;
    const double tdl = kDl * twist_ + cDl * (out_w_ - wheelW);
    const double ie = e.inertia;
    tele.clutch_slipping = false;
    double tc = 0, N = 0, io = 0.02;
    if (gear_ > 0) {
        N = e.primary * e.gears[gear_ - 1] * e.rear_sprocket / e.front_sprocket;
        io = 0.002 * N * N + 0.02;  // gearbox input side, reflected to the wheel
        const double slip = crank_w_ - out_w_ * N;
        const double treq = (te / ie + N * tdl / io) / (1.0 / ie + N * N / io);
        if (std::fabs(slip) < 1.0 && std::fabs(treq) <= capacity) {
            tc = treq;  // locked
        } else {
            double dir = std::fabs(slip) > 1e-6 ? (slip > 0 ? 1.0 : -1.0) : (treq > 0 ? 1.0 : -1.0);
            tc = capacity * dir;
            tele.clutch_slipping = capacity > 0.5;
        }
    }
    const double crankAcc = (te - tc) / ie;
    crank_w_ += crankAcc * dt;
    // Gearbox output and driveline twist, implicitly: in neutral or with the clutch slipping the output side is a
    // tiny inertia on a stiff spring-damper, which an explicit step would make unstable.
    const double a = dt / io;
    out_w_ = (out_w_ + a * (tc * N - kDl * twist_ + (kDl * dt + cDl) * wheelW)) / (1.0 + a * (kDl * dt + cDl));
    if (gear_ > 0 && !tele.clutch_slipping && capacity > 0 && std::fabs(crank_w_ - out_w_ * N) < 1.0) crank_w_ = out_w_ * N;
    twist_ += (out_w_ - wheelW) * dt;

    // Chain: tension pulls the top run (or the bottom run when the engine brakes) between the sprockets.
    const Vec3 axleW = mb.bodies[kRearWheel].pos;
    const Vec3 csW = mb.base_pos + mb.base_rot.rotate(countershaft);
    const Vec3 d = normalize(csW - axleW);
    const Vec3 up = normalize(cross(d, mb.base_rot.rotate({1, 0, 0})));  // perpendicular, upward side of the chain
    const double tension = tdl / rear_sprocket_r;
    const Vec3 side = tension >= 0 ? up : -up;
    const double t = std::fabs(tension);
    mb.addForce(kRearWheel, d * t, axleW + side * rear_sprocket_r);
    mb.addForce(-1, -d * t, csW + side * front_sprocket_r);

    // The crank's reaction on the frame: its spin-up torque and its gyroscopic moment.
    const Vec3 crankAxis = mb.base_rot.rotate({1, 0, 0});
    mb.addTorque(-1, crankAxis * (-ie * crankAcc) - cross(mb.angularVelocity(-1), crankAxis * (ie * crank_w_)));

    tele.rpm = crank_w_ * 60.0 / (2 * kPi);
    tele.gear = gear_;
    tele.engine_running = running_;
    return tdl;
}

void Bike::brakes(const BikeInput& in, double chainTorque, double dt) {
    // Coulomb brakes on the wheel joints: they react against the fork and swingarm, so dive and squat emerge. A
    // brake can hold a stopped wheel against the other torques on it, up to its capacity.
    auto brake = [&](int wheel, double capacity, const TyreOutput& o, double extra) {
        Body& w = mb.bodies[wheel];
        const Vec3 axle = mb.bodies[w.parent].rot.rotate({1, 0, 0});
        const double known = dot(o.torque, axle) + extra;
        const double hold = -(w.qd * w.inertia.m[0][0] / dt + known);
        w.tau = clampd(hold, -capacity, capacity);
    };
    brake(kFrontWheel, 900.0 * clampd(in.front_brake, 0, 1), front_out, 0.0);
    brake(kRearWheel, 550.0 * clampd(in.rear_brake, 0, 1), rear_out, chainTorque);
}

void Bike::bodyContacts(const Terrain& t, const BikeInput& in) {
    const double speed = std::fabs(dot(velocity(), mb.base_rot.rotate({0, 0, 1})));
    const bool autoDab = assists.auto_dab && speed < 1.2;
    dab_active_left = in.dab_left || autoDab;
    dab_active_right = in.dab_right || autoDab;
    for (const Contact& c : contacts_) {
        if (c.dab > 0 && !dab_active_left) continue;
        if (c.dab < 0 && !dab_active_right) continue;
        const Vec3 p = mb.bodyPos(c.body) + mb.bodyRot(c.body).rotate(c.local);
        t.gatherObstacles(p, c.radius + 0.05, scratch_);
        ContactSample cs = t.contact(p, scratch_);
        double depth = cs.depth + c.radius;
        if (depth <= 0) continue;
        if (c.crash) tele.crashed = true;
        const Vec3 v = mb.pointVelocity(c.body, p);
        const double vn = dot(v, cs.normal);
        double fn = std::max(0.0, 6.0e4 * depth - 1500.0 * vn);
        const Vec3 vt = v - cs.normal * vn;
        const double vtl = length(vt);
        const double mu = c.dab ? 0.9 : 0.6;
        Vec3 ft;
        if (vtl > 1e-6) ft = vt * (-std::min(3000.0, mu * fn / vtl));
        mb.addForce(c.body, cs.normal * fn + ft, p - cs.normal * c.radius);
    }
}

void Bike::updateTelemetry() {
    const Vec3 fwd = mb.base_rot.rotate({0, 0, 1});
    const Vec3 left = mb.base_rot.rotate({1, 0, 0});
    const Vec3 fwdFlat = normalize(Vec3{fwd.x, 0, fwd.z});
    tele.speed = dot(velocity(), fwdFlat);  // along the ground, so a wheelie or stoppie doesn't change it
    tele.lean_deg = rad2deg(std::asin(clampd(left.y, -1, 1)));
    tele.pitch_deg = rad2deg(std::asin(clampd(fwd.y, -1, 1)));
    tele.steer_deg = rad2deg(mb.bodies[kSteer].q);
    tele.fork_travel = mb.bodies[kFork].q;
    const double q = mb.bodies[kSwingarm].q;
    tele.shock_travel = swingarm_length * (std::sin(kSwingarmDroop) - std::sin(kSwingarmDroop - q));
    tele.front_load = front_out.normal_load;
    tele.rear_load = rear_out.normal_load;
    tele.front_contact = front_out.contacts > 0;
    tele.rear_contact = rear_out.contacts > 0;
    tele.rpm = crank_w_ * 60.0 / (2 * kPi);
    tele.gear = gear_;
    tele.engine_running = running_;
    if (std::fabs(tele.lean_deg) > 75.0) tele.crashed = true;
}

void Bike::step(const Terrain& terrain, const BikeInput& input, double dt) {
    const bool stiff = front_out.rim_strike || rear_out.rim_strike || front_out.max_compression > 0.06 ||
                       rear_out.max_compression > 0.06;
    const int n = stiff ? 4 : 1;
    BikeInput in = input;
    for (int i = 0; i < n; ++i) {
        const double h = dt / n;
        tyreForces(terrain, kFrontWheel, front_tyre, front_state, front_out, h);
        tyreForces(terrain, kRearWheel, rear_tyre, rear_state, rear_out, h);
        suspension();
        riderMuscles(in);
        mb.bodies[kSteer].tau = steeringTorque(in, h);
        const double chain = drivetrain(in, h);
        brakes(in, chain, h);
        bodyContacts(terrain, in);
        mb.step(h);
        in.shift_up = in.shift_down = in.start_engine = false;  // edges act once
    }
    updateTelemetry();
}

uint64_t Bike::stateHash() const {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](const void* data, size_t n) {
        const unsigned char* b = static_cast<const unsigned char*>(data);
        for (size_t i = 0; i < n; ++i) {
            h ^= b[i];
            h *= 1099511628211ull;
        }
    };
    mix(&mb.base_pos, sizeof mb.base_pos);
    mix(&mb.base_rot, sizeof mb.base_rot);
    mix(&mb.base_vel, sizeof mb.base_vel);
    for (const Body& b : mb.bodies) {
        mix(&b.q, sizeof b.q);
        mix(&b.qd, sizeof b.qd);
    }
    mix(&crank_w_, sizeof crank_w_);
    mix(&out_w_, sizeof out_w_);
    mix(&twist_, sizeof twist_);
    mix(front_state.d_long.data(), front_state.d_long.size() * sizeof(double));
    mix(rear_state.d_long.data(), rear_state.d_long.size() * sizeof(double));
    return h;
}

}  // namespace tws
