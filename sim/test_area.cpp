#include "test_area.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace tws {

namespace {

double smooth01(double t) {
    t = clampd(t, 0.0, 1.0);
    return t * t * (3 - 2 * t);
}

// Height profile of a jump along its run-up axis `s` (metres from the start of the take-off), with rounded edges.
// face: take-off length; top: flat length (0 for a double's gap is handled by the caller); land: landing length.
double tableProfile(double s, double height, double face, double top, double land) {
    if (s < 0) return 0;
    if (s < face) return height * smooth01(s / face) * (0.35 + 0.65 * s / face);  // steepening lip
    if (s < face + top) return height;
    double d = s - face - top;
    if (d < land) return height * (1.0 - smooth01(d / land));
    return 0;
}

uint32_t hash(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352d;
    x ^= x >> 15;
    x *= 0x846ca68b;
    x ^= x >> 16;
    return x;
}
double rand01(uint32_t seed) { return (hash(seed) & 0xFFFFFF) / double(0xFFFFFF); }

}  // namespace

TestAreaInfo buildTestArea(Terrain& t) {
    // 160 m x 160 m at 25 cm. Edges stay at height 0 so the flat world outside meets it without a seam.
    const double cell = 0.25;
    t.init(-80.0, -40.0, cell, 640, 640, MaterialId::Hardpack);

    for (int k = 0; k <= t.nz(); ++k) {
        const double z = t.z0() + k * cell;
        for (int i = 0; i <= t.nx(); ++i) {
            const double x = t.x0() + i * cell;
            double h = 0;
            MaterialId m = MaterialId::Hardpack;

            // Tabletop straight ahead of the spawn: 1.4 m high, 7 m face, 6 m table, 9 m landing; 7 m wide.
            if (std::fabs(x) < 4.5) {
                double w = 1.0 - smooth01((std::fabs(x) - 3.5) / 1.0);
                double p = tableProfile(z - 35.0, 1.4, 7.0, 6.0, 9.0);
                if (p > 0) {
                    h = std::max(h, p * w);
                    m = MaterialId::Loam;
                }
            }
            // A double to the left: 1.2 m take-off, 6 m gap, landing.
            if (x > 14.0 && x < 22.0) {
                double w = 1.0 - smooth01((std::fabs(x - 18.0) - 3.0) / 1.0);
                double takeoff = tableProfile(z - 20.0, 1.2, 6.0, 0.0, 1.5);
                double landing = tableProfile(z - 31.5, 1.2, 1.5, 0.5, 8.0);
                double p = std::max(takeoff, landing);
                if (p > 0) {
                    h = std::max(h, p * w);
                    m = MaterialId::Loam;
                }
            }
            // Rollers to the right: 0.45 m high, 4 m apart.
            if (x < -14.0 && x > -22.0 && z > 10.0 && z < 50.0) {
                double w = 1.0 - smooth01((std::fabs(x + 18.0) - 3.0) / 1.0);
                h = std::max(h, 0.225 * (1.0 - std::cos(2.0 * kPi * (z - 10.0) / 4.0)) * w);
                m = MaterialId::Loam;
            }
            // Hill climb, far right: rising to 7 m over 30 m (about 13 deg, steepening to ~25 deg), loam.
            if (x < -34.0 && x > -48.0 && z > 20.0 && z < 75.0) {
                double w = 1.0 - smooth01((std::fabs(x + 41.0) - 5.0) / 2.0);
                double s = z - 20.0;
                double rise = s < 35.0 ? 7.0 * std::pow(s / 35.0, 1.6) : 7.0 * (1.0 - smooth01((s - 35.0) / 20.0));
                h = std::max(h, rise * w);
                m = MaterialId::Loam;
            }
            // Wheelie straight: a long flat lane on the left with rock stripes every 10 m.
            if (x > 30.0 && x < 34.0 && z > 0.0 && z < 110.0) {
                double zz = std::fmod(z, 10.0);
                if (zz < 0.5) m = MaterialId::Rock;
            }
            t.h(i, k) = static_cast<float>(h);
            t.mat(i, k) = m;
        }
    }

    auto& obs = t.obstacles();
    // Logs: right of the spawn, increasing size (15, 22, 30 cm).
    const double logR[3] = {0.075, 0.11, 0.15};
    for (int n = 0; n < 3; ++n) {
        double z = 10.0 + n * 7.0, r = logR[n];
        obs.push_back({Vec3{-10.0, r - 0.01, z}, Vec3{-6.0, r - 0.01, z}, r, MaterialId::Wood});
    }
    // Rock garden: behind the logs, 60 rocks from fist to boulder size.
    for (int n = 0; n < 60; ++n) {
        double x = -11.0 + rand01(n * 3 + 1) * 6.0;
        double z = 34.0 + rand01(n * 3 + 2) * 18.0;
        double r = 0.10 + std::pow(rand01(n * 3 + 3), 2.0) * 0.32;
        double sink = r * 0.35;
        obs.push_back({Vec3{x, r - sink, z}, Vec3{x, r - sink, z}, r, MaterialId::Rock});
    }
    // A few roots across a lane next to the logs.
    for (int n = 0; n < 5; ++n) {
        double z = 8.0 + n * 3.5;
        double dz = (rand01(100 + n) - 0.5) * 1.2;
        obs.push_back({Vec3{-14.5, 0.012, z}, Vec3{-11.5, 0.012, z + dz}, 0.04, MaterialId::WetRoot});
    }
    return TestAreaInfo{};
}

}  // namespace tws
