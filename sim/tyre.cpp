#include "tyre.h"

#include <algorithm>

namespace tws {

namespace {

double sectionStep(const TyreParams& p) { return 2.0 * kPi / p.sections; }
double profileStep(const TyreParams& p) { return 2.0 * p.profile_half_angle / p.profile_points; }
double profileAngle(const TyreParams& p, int j) { return -p.profile_half_angle + (j + 0.5) * profileStep(p); }

// Radius (from the axle) of profile point j: the crown is a circle of crown_radius whose top is at outer_radius.
double profileRadius(const TyreParams& p, int j) {
    return p.outer_radius - p.crown_radius + p.crown_radius * std::cos(profileAngle(p, j));
}

double elementArea(const TyreParams& p, int j) {
    return profileRadius(p, j) * sectionStep(p) * p.crown_radius * profileStep(p);
}

}  // namespace

Vec3 tyreRestPoint(const TyreParams& p, int i, int j) {
    double th = i * sectionStep(p);  // 0 = straight down, +90 deg = forward
    double ph = profileAngle(p, j);
    double r = profileRadius(p, j);
    return Vec3{p.crown_radius * std::sin(ph), -r * std::cos(th), r * std::sin(th)};
}

void TyreState::init(const TyreParams& p) {
    size_t n = static_cast<size_t>(p.sections) * p.profile_points;
    d_long.assign(n, 0.0);
    d_lat.assign(n, 0.0);
    scratch_long.assign(n, 0.0);
    scratch_lat.assign(n, 0.0);
    point.assign(n, Vec3{});
    load.assign(n, 0.0f);
    sliding.assign(n, 0);
    last_max_compression = 0;
}

TyreOutput tyreStep(const TyreParams& p, TyreState& s, const WheelKinematics& k, const Terrain& terrain, double dt,
                    std::vector<int>& nearObs) {
    TyreOutput out;
    const int nc = p.sections, nw = p.profile_points;
    const double dth = sectionStep(p);
    const Vec3 axle = k.carrier.rotate(Vec3{1, 0, 0});
    const Vec3 fwd = k.carrier.rotate(Vec3{0, 0, 1});

    // Effective rolling radius: the tread belt barely stretches, so the tread speed is set by a radius between the
    // free and the loaded one (R_e ~ R0 - deflection/3, Pacejka). Uses last tick's deflection.
    const double re = p.outer_radius - s.last_max_compression / 3.0;
    out.effective_radius = re;

    // Tread flows from the front of the patch to the back: slot i is fed from slot i + shift.
    const double shift = k.spin * dt / dth;

    terrain.gatherObstacles(k.hub, p.outer_radius + 0.05, nearObs);
    const double cullMargin = 2.0 * p.crown_radius;

    for (int i = 0; i < nc; ++i) {
        // Cheap reject: the middle of the section is well clear of the ground and of any obstacle.
        Vec3 mid = k.hub + k.carrier.rotate(tyreRestPoint(p, i, nw / 2));
        Vec3 gn;
        double gh = terrain.heightAt(mid.x, mid.z, &gn);
        bool clear = (mid.y - gh) * std::max(gn.y, 0.3) > cullMargin;
        if (clear && !nearObs.empty()) {
            ContactSample cs = terrain.contact(mid, nearObs);
            clear = cs.depth < -cullMargin;
        }
        double src = i + shift;
        double fl = std::floor(src);
        double f = src - fl;
        int i0 = static_cast<int>(((static_cast<long long>(fl) % nc) + nc) % nc);
        int i1 = (i0 + 1) % nc;

        for (int j = 0; j < nw; ++j) {
            size_t e = static_cast<size_t>(i) * nw + j;
            Vec3 r = k.carrier.rotate(tyreRestPoint(p, i, j));
            Vec3 p0 = k.hub + r;
            s.point[e] = p0;
            s.load[e] = 0;
            s.sliding[e] = 0;
            s.scratch_long[e] = 0;
            s.scratch_lat[e] = 0;
            if (clear) continue;

            ContactSample cs = terrain.contact(p0, nearObs);
            if (cs.depth <= 0) continue;

            const double area = elementArea(p, j);
            const double comp = cs.depth;
            const Vec3 n = cs.normal;

            // Velocity of the carcass at this element: hub + carrier rotation + rigid spin. Its normal part is the
            // compression rate (the tread compresses entering the patch and recovers leaving it: hysteresis).
            Vec3 spinVel = cross(axle, r) * k.spin;
            Vec3 vel = k.velocity + cross(k.carrier_omega, r) + spinVel;
            double compRate = -dot(vel, n);
            // In the patch the tread belt runs flat along the ground at the belt speed spin * R_e (R_e for this
            // profile point: leaned over, the tyre rolls on a smaller radius).
            Vec3 along = spinVel - n * dot(spinVel, n);
            double alongLen = length(along);
            if (alongLen > 1e-9) {
                double belt = std::fabs(k.spin) * (profileRadius(p, j) - s.last_max_compression / 3.0);
                vel += along * (belt / alongLen) - along;
            }

            double t = std::min(comp / p.contact_ramp, 1.0);
            double spring = p.pressure * area * t * t * (3 - 2 * t) + p.carcass_k * area * comp;
            // Hunt-Crossley-style damping: it grows with compression, so a fast hit starts soft instead of producing
            // a huge force the instant the tread touches. It is also capped at the elastic force (rubber can't push
            // back harder than it is squeezed), which keeps very fast impacts physical.
            double damp = (compRate > 0 ? p.damp_compress : p.damp_rebound) * area * compRate *
                          std::min(comp / p.damp_depth, 1.0);
            double bottom = std::max(profileRadius(p, j) - p.rim_radius - p.bottom_margin, 0.01);
            if (comp > bottom) {
                double pinch = comp - bottom;
                spring += p.rim_k * area * pinch;
                damp += p.rim_damp * area * compRate * std::min(pinch / p.damp_depth, 1.0);
                out.rim_strike = true;
            }
            double fn = std::max(spring + std::clamp(damp, -spring, spring), 0.0);

            // Ground-plane basis: lateral follows the axle, longitudinal the wheel's heading.
            Vec3 lat0 = axle - n * dot(axle, n);
            Vec3 eLat, eLong;
            if (length(lat0) > 0.1) {
                eLat = normalize(lat0);
                eLong = cross(eLat, n);
            } else {
                eLong = normalize(fwd - n * dot(fwd, n));
                eLat = cross(n, eLong);
            }
            Vec3 vs = vel - n * dot(vel, n);
            double sl = dot(vs, eLong), st = dot(vs, eLat);

            // Bristle: carried in from upstream, then stretched by this tick's slip.
            double dl = (1 - f) * s.d_long[static_cast<size_t>(i0) * nw + j] + f * s.d_long[static_cast<size_t>(i1) * nw + j];
            double dlat = (1 - f) * s.d_lat[static_cast<size_t>(i0) * nw + j] + f * s.d_lat[static_cast<size_t>(i1) * nw + j];
            dl += sl * dt;
            dlat += st * dt;
            double kl = p.bristle_k_long * area, kt = p.bristle_k_lat * area, cb = p.bristle_damp * area;
            double Fl = -(kl * dl + cb * sl);
            double Ft = -(kt * dlat + cb * st);

            const Material& m = material(cs.mat);
            double vslide = length(vs);
            double ratio = vslide / m.slide_speed;
            double mu = m.mu_kinetic + (m.mu_static - m.mu_kinetic) * std::exp(-ratio * ratio);
            double lim = mu * fn;
            double mag = std::sqrt(Fl * Fl + Ft * Ft);
            if (mag > lim) {
                double sc = mag > 1e-12 ? lim / mag : 0.0;
                Fl *= sc;
                Ft *= sc;
                dl = -Fl / kl;
                dlat = -Ft / kt;
                s.sliding[e] = 1;
                ++out.sliding;
            }
            s.scratch_long[e] = dl;
            s.scratch_lat[e] = dlat;

            Vec3 F = n * fn + eLong * Fl + eLat * Ft;
            Vec3 pc = p0 + n * comp;
            out.force += F;
            out.torque += cross(pc - k.hub, F);
            out.normal_load += fn;
            out.patch_area += area;
            out.max_compression = std::max(out.max_compression, comp);
            ++out.contacts;
            s.point[e] = pc;
            s.load[e] = static_cast<float>(fn / area);
        }
    }
    s.last_max_compression = out.max_compression;
    s.d_long.swap(s.scratch_long);
    s.d_lat.swap(s.scratch_lat);
    return out;
}

}  // namespace tws
