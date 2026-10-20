// Physics tests: unit checks plus validation against real-world tyre behaviour (docs/03_PHYSICS_SPEC.md §7).
// Run: build/Release/sim_tests.exe [-v]   (-v prints the measured values)
#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "wheel_lab.h"

using namespace tws;

namespace {

int g_failures = 0;
bool g_verbose = false;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("    FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)

#define CHECK_RANGE(v, lo, hi)                                                                          \
    do {                                                                                                \
        double v_ = (v);                                                                                \
        if (g_verbose) std::printf("    %-34s = %10.4f   [%g .. %g]\n", #v, v_, (double)(lo), (double)(hi)); \
        if (!(v_ >= (lo) && v_ <= (hi))) {                                                              \
            std::printf("    FAILED %s:%d: %s = %g, expected [%g, %g]\n", __FILE__, __LINE__, #v, v_,      \
                        (double)(lo), (double)(hi));                                                    \
            ++g_failures;                                                                               \
        }                                                                                               \
    } while (0)

struct Test {
    const char* name;
    std::function<void()> fn;
};
std::vector<Test>& tests() {
    static std::vector<Test> t;
    return t;
}
struct Reg {
    Reg(const char* n, std::function<void()> f) { tests().push_back({n, std::move(f)}); }
};
#define TEST(name) \
    static void name(); \
    static Reg reg_##name(#name, name); \
    static void name()

void run(WheelLab& lab, double seconds) {
    int n = static_cast<int>(seconds * WheelLab::kTickHz + 0.5);
    for (int i = 0; i < n; ++i) lab.step();
}

// Average rig measurements over `seconds`.
LabMeasure average(WheelLab& lab, double seconds) {
    LabMeasure a;
    int n = static_cast<int>(seconds * WheelLab::kTickHz + 0.5);
    for (int i = 0; i < n; ++i) {
        lab.step();
        a.fx += lab.measure.fx;
        a.fy += lab.measure.fy;
        a.fz += lab.measure.fz;
        a.mz += lab.measure.mz;
        a.slip_ratio += lab.measure.slip_ratio;
        a.deflection += lab.measure.deflection;
    }
    a.fx /= n; a.fy /= n; a.fz /= n; a.mz /= n; a.slip_ratio /= n; a.deflection /= n;
    return a;
}

WheelLab flatRig(double speed, double loadKg) {
    WheelLab lab;
    lab.setScene(LabScene::Flat);
    lab.rig.speed = speed;
    lab.rig.load_kg = loadKg;
    lab.resetRig();
    return lab;
}

}  // namespace

TEST(math_quaternion_rotation) {
    Quat q = Quat::axisAngle({0, 1, 0}, kPi / 2);
    Vec3 v = q.rotate({0, 0, 1});
    CHECK(std::fabs(v.x - 1) < 1e-12 && std::fabs(v.z) < 1e-12);
    Quat r = Quat{}.integrated({0, 0, kPi}, 0.5);  // half a second at pi rad/s = 90 degrees about Z
    Vec3 u = r.rotate({1, 0, 0});
    CHECK(std::fabs(u.y - 1) < 1e-9);
}

TEST(terrain_heights_and_obstacles) {
    WheelLab lab;  // obstacle lane
    CHECK(std::fabs(lab.terrain.heightAt(0, 5)) < 1e-9);
    CHECK(std::fabs(lab.terrain.heightAt(0, 11) - 0.10) < 1e-6);
    std::vector<int> near;
    lab.terrain.gatherObstacles({0, 0.2, 22}, 0.1, near);
    ContactSample c = lab.terrain.contact({0, 0.2, 22}, near);  // 9 cm above the log's axis, inside its 12 cm radius
    CHECK_RANGE(c.depth, 0.029, 0.031);
    CHECK(c.mat == MaterialId::Wood);
}

// Vertical stiffness vs pressure. Published knobby rears: roughly 80-160 kN/m at 0.8-1.0 bar.
TEST(tyre_vertical_stiffness_rises_with_pressure) {
    double k[3];
    const double bars[3] = {0.6, 0.9, 1.4};
    for (int n = 0; n < 3; ++n) {
        WheelLab lab = flatRig(0.0, 140.0);
        lab.params.pressure = bars[n] * 1e5;
        lab.resetRig();
        run(lab, 1.5);
        LabMeasure m = average(lab, 0.5);
        k[n] = m.fz / m.deflection;
        if (g_verbose) std::printf("    %.1f bar: Fz %.0f N, deflection %.1f mm, k %.0f N/m\n", bars[n], m.fz, m.deflection * 1000, k[n]);
    }
    CHECK_RANGE(k[1], 70000, 170000);
    CHECK(k[0] < k[1] && k[1] < k[2]);
}

// Free rolling on hardpack: rolling resistance coefficient for a knobby is ~0.01-0.04.
TEST(tyre_rolling_resistance) {
    WheelLab lab = flatRig(10.0, 140.0);
    run(lab, 1.5);
    LabMeasure m = average(lab, 1.0);
    double crr = -m.fx / m.fz;
    CHECK_RANGE(crr, 0.003, 0.06);
    CHECK_RANGE(m.slip_ratio, -0.05, 0.01);
}

// Locked wheel: force = kinetic friction of the surface (hardpack 0.80).
TEST(tyre_locked_wheel_slides_at_kinetic_friction) {
    WheelLab lab = flatRig(10.0, 140.0);
    lab.rig.brake_torque = 5000;
    run(lab, 0.5);
    CHECK(lab.spin == 0.0);
    LabMeasure m = average(lab, 0.5);
    CHECK_RANGE(-m.fx / m.fz, 0.72, 0.85);
}

// Strong drive torque: wheelspin, force capped by friction; a moderate one: small slip, no spin-up.
TEST(tyre_drive_traction) {
    WheelLab lab = flatRig(10.0, 140.0);
    lab.rig.drive_torque = 250;  // ~750 N at the ground, half the available grip at ~1500 N load
    run(lab, 1.0);
    LabMeasure m = average(lab, 0.5);
    CHECK_RANGE(m.fx / m.fz, 0.4, 0.6);
    CHECK_RANGE(m.slip_ratio, 0.0, 0.15);

    WheelLab spinning = flatRig(10.0, 140.0);
    spinning.rig.drive_torque = 1500;
    run(spinning, 1.0);
    LabMeasure s = average(spinning, 0.3);
    CHECK_RANGE(s.slip_ratio, 0.5, 1e9);
    CHECK_RANGE(s.fx / s.fz, 0.70, 0.97);
}

// Slip angle: lateral force builds roughly linearly at small angles and saturates near friction at large ones.
TEST(tyre_lateral_force_curve) {
    auto fyAt = [](double slip) {
        WheelLab lab = flatRig(10.0, 140.0);
        lab.rig.slip_deg = slip;
        lab.resetRig();
        run(lab, 1.0);
        LabMeasure m = average(lab, 0.5);
        if (g_verbose) std::printf("    slip %5.1f deg: Fy/Fz %.3f  Mz %.1f N*m\n", slip, m.fy / m.fz, m.mz);
        return m.fy / m.fz;
    };
    double f1 = fyAt(1.0), f4 = fyAt(4.0), f25 = fyAt(25.0);
    // Positive slip turns the heading toward +X while travel stays +Z: the tyre pushes toward where it points
    // (that is how a steered wheel turns), i.e. along +axle.
    CHECK(f1 > 0 && f4 > 0 && f25 > 0);
    CHECK_RANGE(f4, 0.25, 0.95);
    CHECK_RANGE(f25, 0.65, 1.0);
    CHECK(f4 > f1 * 2.0);  // growing with slip at small angles
}

// Camber thrust: a leaned wheel pushes toward the side it leans to.
TEST(tyre_camber_thrust) {
    WheelLab lab = flatRig(10.0, 140.0);
    lab.rig.camber_deg = 20.0;  // rotate about +Z: top goes toward -X, wheel leans to -X
    lab.resetRig();
    run(lab, 1.0);
    LabMeasure m = average(lab, 0.5);
    if (g_verbose) std::printf("    camber 20 deg: Fy/Fz %.3f\n", m.fy / m.fz);
    // Leaning toward -X: thrust toward -X. measure.fy is along the axle's horizontal (+X-ish), so negative.
    CHECK(m.fy < 0);
}

// Static grip: a parked wheel under load does not creep.
TEST(tyre_static_wheel_does_not_creep) {
    WheelLab lab = flatRig(0.0, 140.0);
    run(lab, 2.0);
    CHECK_RANGE(std::fabs(lab.spin), 0.0, 1e-3);
    CHECK_RANGE(std::fabs(lab.measure.fx), 0.0, 5.0);
}

// A free wheel dropped from 30 cm bounces and settles on the ground without tunnelling. (Left alone longer, a
// stationary wheel topples over, as a real one does.)
TEST(free_wheel_drop_settles) {
    WheelLab lab;
    lab.setScene(LabScene::Flat);
    lab.resetFree({0, 0.337 + 0.3, 0}, 0.0, 0.0);
    double minY = 1, maxAfter = 0;
    bool touched = false;
    for (int i = 0; i < 1500; ++i) {
        lab.step();
        minY = std::min(minY, lab.pos.y);
        touched = touched || lab.out.contacts > 0;
        if (touched) maxAfter = std::max(maxAfter, lab.pos.y);
    }
    CHECK_RANGE(minY, 0.26, 0.33);         // compresses the tyre on impact but never reaches through it
    CHECK_RANGE(maxAfter, 0.337, 0.60);    // bounces, but lower than it was dropped from (0.637 m: energy is lost)
}

// A rolling wheel is self-stable (gyroscopic + camber thrust): it rolls on upright-ish for seconds.
TEST(free_wheel_rolls_upright) {
    WheelLab lab;
    lab.setScene(LabScene::Flat);
    lab.resetFree({0, 0.33, 0}, 8.0, 3.0);
    run(lab, 3.0);
    Vec3 axle = lab.carrier.rotate({1, 0, 0});
    double leanDeg = rad2deg(std::asin(clampd(axle.y, -1, 1)));
    double speed = length(lab.vel);
    if (g_verbose) std::printf("    after 3 s: lean %.1f deg, speed %.2f m/s, z %.1f m\n", leanDeg, speed, lab.pos.z);
    CHECK_RANGE(std::fabs(leanDeg), 0.0, 35.0);
    CHECK_RANGE(speed, 6.0, 8.1);
}

// The rig runs the whole obstacle lane (step, root, log, rock, rollers) without blowing up.
TEST(rig_obstacle_lane_is_stable) {
    WheelLab lab;
    lab.rig.speed = 4.0;
    lab.resetRig();
    double maxY = 0;
    bool rim = false;
    for (int i = 0; i < 11000; ++i) {
        lab.step();
        maxY = std::max(maxY, lab.pos.y);
        rim = rim || lab.out.rim_strike;
        if (!std::isfinite(lab.pos.y)) {
            CHECK(std::isfinite(lab.pos.y));
            return;
        }
    }
    if (g_verbose) std::printf("    max hub height %.3f m, rim strike seen: %d\n", maxY, rim ? 1 : 0);
    CHECK_RANGE(maxY, 0.45, 0.9);  // it climbed the step, log and rock without being launched
}

// Very low and zero pressure: the rig still runs the whole lane without blowing up (lots of rim strikes, though).
TEST(rig_low_pressure_is_stable) {
    for (double psi : {0.0, 3.0, 6.0}) {
        WheelLab lab;
        lab.params.pressure = psi * 6894.76;
        lab.rig.speed = 8.0;
        lab.resetRig();
        double maxY = 0;
        for (int i = 0; i < 5000; ++i) {
            lab.step();
            maxY = std::max(maxY, lab.pos.y);
        }
        if (g_verbose) std::printf("    %.0f psi: max hub %.3f m\n", psi, maxY);
        CHECK(std::isfinite(lab.pos.y));
        CHECK_RANGE(maxY, 0.2, 0.9);
    }
}

TEST(determinism_same_inputs_same_state) {
    auto runOnce = [] {
        WheelLab lab;
        lab.resetFree({0, 0.4, 0}, 6.0, 5.0);
        lab.free_drive_torque = 30;
        for (int i = 0; i < 4000; ++i) {
            if (i == 1000) lab.applyImpulse({20, 0, 0}, lab.pos + Vec3{0, 0.3, 0});
            lab.step();
        }
        return lab.stateHash();
    };
    CHECK(runOnce() == runOnce());
}

// Dev tool: `sim_tests --trace-lane [speed]` prints the rig's hub height along the obstacle lane.
static void traceLane(double speed) {
    WheelLab lab;
    lab.rig.speed = speed;
    lab.resetRig();
    int n = static_cast<int>(40.0 / speed * WheelLab::kTickHz);
    for (int i = 0; i < n; ++i) {
        lab.step();
        if (i % 25 == 0 || lab.out.rim_strike)
            std::printf("z %6.2f  y %.3f  vy %6.2f  Fz %7.0f  comp %.3f  contacts %3d%s\n", lab.pos.z, lab.pos.y, lab.vel.y,
                        lab.out.force.y, lab.out.max_compression, lab.out.contacts, lab.out.rim_strike ? "  RIM" : "");
    }
}

// Dev tool: `sim_tests --stress` runs the rig and the free wheel fast over the lane and reports the worst moments.
static void stress() {
    const double speeds[] = {4, 8, 12, 16, 20, 25};
    for (double v : speeds) {
        WheelLab lab;
        lab.rig.speed = v;
        lab.resetRig();
        double maxY = 0, maxF = 0, maxJump = 0, maxSpin = 0, maxSlipVel = 0;
        double lastF = 0;
        int n = static_cast<int>(40.0 / v * WheelLab::kTickHz);
        for (int i = 0; i < n; ++i) {
            lab.step();
            double f = length(lab.out.force);
            maxY = std::max(maxY, lab.pos.y);
            maxF = std::max(maxF, f);
            maxJump = std::max(maxJump, std::fabs(f - lastF));
            maxSpin = std::max(maxSpin, std::fabs(lab.spin * lab.params.outer_radius - v));
            lastF = f;
            (void)maxSlipVel;
        }
        std::printf("rig %4.0f m/s: max hub %.2f m, max force %6.0f N, max force jump/tick %6.0f N, max |wheel surface speed - v| %.1f m/s, finite %d\n",
                    v, maxY, maxF, maxJump, maxSpin, std::isfinite(lab.pos.y) ? 1 : 0);
    }
    for (double v : speeds) {
        WheelLab lab;
        lab.resetFree({0, 0.34, 1.0}, v, 0.0);
        double maxY = 0, maxF = 0, maxAng = 0;
        int n = static_cast<int>(6.0 * WheelLab::kTickHz);
        for (int i = 0; i < n; ++i) {
            lab.step();
            maxY = std::max(maxY, lab.pos.y);
            maxF = std::max(maxF, length(lab.out.force));
            Vec3 axle = lab.carrier.rotate({1, 0, 0});
            maxAng = std::max(maxAng, std::fabs(rad2deg(std::asin(clampd(axle.y, -1, 1)))));
        }
        std::printf("free %4.0f m/s: max hub %.2f m, max force %6.0f N, max lean %.0f deg, end z %.1f, finite %d\n", v,
                    maxY, maxF, maxAng, lab.pos.z, std::isfinite(lab.pos.y) ? 1 : 0);
    }
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--stress") == 0) {
            stress();
            return 0;
        }
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-v") == 0) g_verbose = true;
        if (std::strcmp(argv[i], "--trace-lane") == 0) {
            traceLane(i + 1 < argc ? std::atof(argv[i + 1]) : 4.0);
            return 0;
        }
    }
    int failedTests = 0;
    for (auto& t : tests()) {
        int before = g_failures;
        std::printf("%s\n", t.name);
        t.fn();
        if (g_failures != before) ++failedTests;
    }
    std::printf("\n%zu tests, %d failed\n", tests().size(), failedTests);
    return failedTests ? 1 : 0;
}
