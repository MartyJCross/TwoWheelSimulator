// The tyre: a ring of carcass elements across a round tread profile, each with a pressure/carcass spring and a
// brush-model bristle that sticks or slides (docs/03_PHYSICS_SPEC.md §1).
//
// Elements live in the wheel's *carrier* frame (the wheel's orientation without its spin), so they stay put around
// the contact patch while the tread flows through them. Bristle deflection is carried along with the tread
// (semi-Lagrangian advection). Wheelspin, lock-up, slip angle, combined slip, relaxation length and static grip all
// come out of this; no slip curve is fitted.
//
// Carrier frame: +X = axle (rider's left), +Y = up, +Z = forward. Positive spin about +X rolls the wheel forward.
#pragma once
#include <cstdint>
#include <vector>

#include "vecmath.h"
#include "terrain.h"

namespace tws {

struct TyreParams {
    // Geometry: an enduro rear (120/90-18) by default.
    double outer_radius = 0.337;  // m, unloaded
    double rim_radius = 0.2286;   // m (18" bead seat)
    double crown_radius = 0.060;  // m, radius of the round tread profile
    double profile_half_angle = deg2rad(75.0);  // how far round the profile the tread reaches
    double bottom_margin = 0.025;  // m of rubber left between ground and rim at a rim strike
    int sections = 128;            // elements round the ring
    int profile_points = 7;        // elements across the tread

    // Inflation. 0.9 bar (13 psi) is a typical enduro rear.
    double pressure = 0.9e5;  // Pa (gauge)

    // Structure. Values give ~100 kN/m vertical stiffness at 0.9 bar, close to published knobby data.
    double carcass_k = 2.5e6;        // N/m per m^2 of element area: sidewall/carcass stiffness without air
    double contact_ramp = 0.002;     // m: pressure force builds over the first 2 mm (smooths entry into contact)
    double damp_compress = 1.8e4;    // N*s/m per m^2 (hysteresis: more damping in compression -> rolling resistance)
    double damp_rebound = 0.6e4;     // N*s/m per m^2
    double rim_k = 5.0e8;            // N/m per m^2 once the rim bottoms through the tyre
    double rim_damp = 3.0e6;         // N*s/m per m^2: a pinched tyre soaks up the hit instead of springing back

    // Tread bristles (brush model). Longitudinal stiffer than lateral, as on real knobbies.
    double bristle_k_long = 2.3e7;   // N/m per m^2
    double bristle_k_lat = 1.5e7;    // N/m per m^2
    double bristle_damp = 1.5e4;     // N*s/m per m^2

    // Mass of wheel + tyre (rim, hub, spokes, tyre, tube): ~13 kg for an enduro rear.
    double mass = 13.0;
    double inertia_axle = 0.85;      // kg m^2 about the axle
    double inertia_transverse = 0.45;
};

struct TyreOutput {
    Vec3 force;          // total on the wheel, world
    Vec3 torque;         // total about the hub, world
    double normal_load = 0;     // sum of element normal forces (N)
    double max_compression = 0; // m, deepest element
    double patch_area = 0;      // m^2 of elements in contact
    double effective_radius = 0;
    int contacts = 0;
    int sliding = 0;
    bool rim_strike = false;
};

// Per-element state that persists between ticks, plus per-element data the renderer and telemetry read.
struct TyreState {
    std::vector<double> d_long, d_lat;     // bristle deflection (m), carried with the tread
    std::vector<double> scratch_long, scratch_lat;
    double last_max_compression = 0;      // m, from the previous tick (sets the rolling radius)
    // Read-only outputs, refreshed each tick:
    std::vector<Vec3> point;               // element surface point (deformed where in contact), world
    std::vector<float> load;               // contact pressure (Pa) or 0
    std::vector<uint8_t> sliding;          // 1 if the element is sliding
    void init(const TyreParams& p);
};

struct WheelKinematics {
    Vec3 hub;        // hub position
    Quat carrier;    // wheel orientation without spin
    Vec3 velocity;   // hub velocity
    Vec3 carrier_omega;  // angular velocity of the carrier (world; no component along the axle)
    double spin = 0;     // spin rate about the axle (rad/s), + = rolling forward
};

// One tick of contact: advect bristles, find contacts, compute forces. `scratch` holds obstacle indices.
TyreOutput tyreStep(const TyreParams& p, TyreState& s, const WheelKinematics& k, const Terrain& terrain,
                    double dt, std::vector<int>& scratch);

// Undeformed element point in the carrier frame (section i, profile point j).
Vec3 tyreRestPoint(const TyreParams& p, int i, int j);

}  // namespace tws
