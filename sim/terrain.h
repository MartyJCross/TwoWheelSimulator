// The ground: a heightfield plus rigid obstacles (capsules; a capsule with a == b is a sphere, used for rocks).
// M0 ground is rigid. Deformable soil (sinkage, ruts, roost) arrives in M4 on top of the same queries.
#pragma once
#include <cstdint>
#include <vector>

#include "vecmath.h"

namespace tws {

enum class MaterialId : uint8_t { Hardpack, Loam, Rock, WetRoot, Wood, Count };

struct Material {
    const char* name;
    double mu_static;    // friction at zero sliding speed
    double mu_kinetic;   // friction when fully sliding
    double slide_speed;  // m/s: how quickly friction falls from static to kinetic (Stribeck-style)
    float r, g, b;       // stylised colour (the game reads it; the sim ignores it)
};

// Rubber-on-surface friction. Sources: published tyre/terrain friction ranges (dry dirt 0.6-1.0, wet roots ~0.3,
// dry rock 0.8-1.0). Tuned further against the feel once the bike rides (M1).
const Material& material(MaterialId id);

struct Capsule {
    Vec3 a, b;         // segment ends (equal for a sphere)
    double radius = 0;
    MaterialId mat = MaterialId::Rock;
};

struct ContactSample {
    double depth = -1e9;  // > 0 means the point is inside the ground by this distance
    Vec3 normal{0, 1, 0};
    MaterialId mat = MaterialId::Hardpack;
};

class Terrain {
public:
    // Flat ground at height 0 covering [x0, x0 + nx*cell] x [z0, z0 + nz*cell].
    void init(double x0, double z0, double cell, int nx, int nz, MaterialId mat);

    double cell() const { return cell_; }
    int nx() const { return nx_; }
    int nz() const { return nz_; }
    double x0() const { return x0_; }
    double z0() const { return z0_; }

    // Grid access (vertex heights; vertex (i, k) sits at x0 + i*cell, z0 + k*cell).
    float& h(int i, int k) { return heights_[static_cast<size_t>(k) * (nx_ + 1) + i]; }
    float h(int i, int k) const { return heights_[static_cast<size_t>(k) * (nx_ + 1) + i]; }
    MaterialId& mat(int i, int k) { return mats_[static_cast<size_t>(k) * (nx_ + 1) + i]; }
    MaterialId mat(int i, int k) const { return mats_[static_cast<size_t>(k) * (nx_ + 1) + i]; }

    // Bilinear height and its gradient-based normal. Outside the grid: flat hardpack at height 0.
    double heightAt(double x, double z) const;
    double heightAt(double x, double z, Vec3* normal) const;
    MaterialId materialAt(double x, double z) const;

    std::vector<Capsule>& obstacles() { return obstacles_; }
    const std::vector<Capsule>& obstacles() const { return obstacles_; }

    // Indices of obstacles whose bounds come within `radius` of `center`.
    void gatherObstacles(const Vec3& center, double radius, std::vector<int>& out) const;

    // Deepest contact of point p against the heightfield and the listed obstacles.
    ContactSample contact(const Vec3& p, const std::vector<int>& nearObstacles) const;

private:
    double x0_ = 0, z0_ = 0, cell_ = 0.05;
    int nx_ = 0, nz_ = 0;
    std::vector<float> heights_;
    std::vector<MaterialId> mats_;
    std::vector<Capsule> obstacles_;
};

}  // namespace tws
