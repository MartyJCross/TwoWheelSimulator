#include "terrain.h"

#include <algorithm>

namespace tws {

const Material& material(MaterialId id) {
    static const Material table[] = {
        {"Hardpack", 1.00, 0.85, 0.6, 0.78f, 0.62f, 0.42f},
        {"Loam", 1.00, 0.85, 0.6, 0.45f, 0.33f, 0.24f},
        {"Rock", 1.00, 0.85, 0.5, 0.62f, 0.62f, 0.66f},
        {"Wet root", 0.35, 0.25, 0.4, 0.52f, 0.34f, 0.20f},
        {"Wood", 0.75, 0.60, 0.5, 0.60f, 0.45f, 0.30f},
    };
    return table[static_cast<int>(id)];
}

void Terrain::init(double x0, double z0, double cell, int nx, int nz, MaterialId m) {
    x0_ = x0;
    z0_ = z0;
    cell_ = cell;
    nx_ = nx;
    nz_ = nz;
    heights_.assign(static_cast<size_t>(nx + 1) * (nz + 1), 0.0f);
    mats_.assign(heights_.size(), m);
    obstacles_.clear();
}

double Terrain::heightAt(double x, double z) const { return heightAt(x, z, nullptr); }

double Terrain::heightAt(double x, double z, Vec3* normal) const {
    // Outside the grid the world is flat hardpack at height 0 (grid edges are kept at 0 so there is no seam).
    if (x < x0_ || z < z0_ || x > x0_ + nx_ * cell_ || z > z0_ + nz_ * cell_) {
        if (normal) *normal = Vec3{0, 1, 0};
        return 0.0;
    }
    double fx = clampd((x - x0_) / cell_, 0.0, nx_ - 1e-9);
    double fz = clampd((z - z0_) / cell_, 0.0, nz_ - 1e-9);
    int i = static_cast<int>(fx);
    int k = static_cast<int>(fz);
    double tx = fx - i, tz = fz - k;
    double h00 = h(i, k), h10 = h(i + 1, k), h01 = h(i, k + 1), h11 = h(i + 1, k + 1);
    double hz0 = h00 + (h10 - h00) * tx;
    double hz1 = h01 + (h11 - h01) * tx;
    if (normal) {
        double dhdx = ((h10 - h00) * (1 - tz) + (h11 - h01) * tz) / cell_;
        double dhdz = (hz1 - hz0) / cell_;
        *normal = normalize(Vec3{-dhdx, 1.0, -dhdz});
    }
    return hz0 + (hz1 - hz0) * tz;
}

MaterialId Terrain::materialAt(double x, double z) const {
    if (x < x0_ || z < z0_ || x > x0_ + nx_ * cell_ || z > z0_ + nz_ * cell_) return MaterialId::Hardpack;
    int i = static_cast<int>(clampd(std::floor((x - x0_) / cell_ + 0.5), 0, nx_));
    int k = static_cast<int>(clampd(std::floor((z - z0_) / cell_ + 0.5), 0, nz_));
    return mat(i, k);
}

void Terrain::gatherObstacles(const Vec3& c, double radius, std::vector<int>& out) const {
    out.clear();
    for (int n = 0; n < static_cast<int>(obstacles_.size()); ++n) {
        const Capsule& o = obstacles_[n];
        double r = o.radius + radius;
        Vec3 lo{std::min(o.a.x, o.b.x) - r, std::min(o.a.y, o.b.y) - r, std::min(o.a.z, o.b.z) - r};
        Vec3 hi{std::max(o.a.x, o.b.x) + r, std::max(o.a.y, o.b.y) + r, std::max(o.a.z, o.b.z) + r};
        if (c.x >= lo.x && c.x <= hi.x && c.y >= lo.y && c.y <= hi.y && c.z >= lo.z && c.z <= hi.z) out.push_back(n);
    }
}

ContactSample Terrain::contact(const Vec3& p, const std::vector<int>& nearObstacles) const {
    ContactSample best;
    Vec3 n;
    double hgt = heightAt(p.x, p.z, &n);
    // Distance below the local tangent plane of the heightfield.
    best.depth = (hgt - p.y) * n.y;
    best.normal = n;
    best.mat = materialAt(p.x, p.z);
    for (int idx : nearObstacles) {
        const Capsule& o = obstacles_[idx];
        Vec3 ab = o.b - o.a;
        double len2 = dot(ab, ab);
        double t = len2 > 1e-12 ? clampd(dot(p - o.a, ab) / len2, 0.0, 1.0) : 0.0;
        Vec3 d = p - (o.a + ab * t);
        double dist = length(d);
        double depth = o.radius - dist;
        if (depth > best.depth) {
            best.depth = depth;
            best.normal = dist > 1e-9 ? d / dist : Vec3{0, 1, 0};
            best.mat = o.mat;
        }
    }
    return best;
}

}  // namespace tws
