#include "render.h"

#include <algorithm>
#include <cmath>

#include "terrain.h"
#include "wheel_lab.h"

using tws::Quat;
using tws::Vec3;

// ---------------------------------------------------------------------------------------------------------------
// Matrices

Mat4 Mat4::perspective(float fovy, float aspect, float zn, float zf) {
    Mat4 r{};
    float f = 1.0f / std::tan(fovy * 0.5f);
    r.m[0] = f / aspect;
    r.m[5] = f;
    r.m[10] = (zf + zn) / (zn - zf);
    r.m[11] = -1.0f;
    r.m[14] = 2.0f * zf * zn / (zn - zf);
    return r;
}

Mat4 Mat4::lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
    Vec3 f = tws::normalize(target - eye);
    Vec3 s = tws::normalize(tws::cross(f, up));
    Vec3 u = tws::cross(s, f);
    Mat4 r{};
    r.m[0] = (float)s.x; r.m[4] = (float)s.y; r.m[8] = (float)s.z;
    r.m[1] = (float)u.x; r.m[5] = (float)u.y; r.m[9] = (float)u.z;
    r.m[2] = (float)-f.x; r.m[6] = (float)-f.y; r.m[10] = (float)-f.z;
    r.m[12] = (float)-tws::dot(s, eye);
    r.m[13] = (float)-tws::dot(u, eye);
    r.m[14] = (float)tws::dot(f, eye);
    r.m[15] = 1.0f;
    return r;
}

Mat4 Mat4::operator*(const Mat4& o) const {
    Mat4 r{};
    for (int c = 0; c < 4; ++c)
        for (int rr = 0; rr < 4; ++rr) {
            float s = 0;
            for (int k = 0; k < 4; ++k) s += m[k * 4 + rr] * o.m[c * 4 + k];
            r.m[c * 4 + rr] = s;
        }
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// Mesh

void Mesh::upload(const std::vector<Vtx>& v, const std::vector<uint32_t>& idx, bool dynamic) {
    if (!vao_) {
        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glGenBuffers(1, &ebo_);
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)(3 * sizeof(float)));
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
    }
    glBindVertexArray(vao_);
    GLenum usage = dynamic ? GL_STREAM_DRAW : GL_STATIC_DRAW;
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(v.size() * sizeof(Vtx)), v.data(), usage);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(idx.size() * sizeof(uint32_t)), idx.data(), usage);
    count_ = (int)idx.size();
}

void Mesh::draw() const {
    if (!count_) return;
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, count_, GL_UNSIGNED_INT, nullptr);
}

// ---------------------------------------------------------------------------------------------------------------
// MeshBuilder

uint32_t MeshBuilder::vert(const Vec3& p, const Vec3& n, const Vec3& c) {
    v.push_back({(float)p.x, (float)p.y, (float)p.z, (float)n.x, (float)n.y, (float)n.z, (float)c.x, (float)c.y,
                 (float)c.z});
    return (uint32_t)(v.size() - 1);
}

void MeshBuilder::cylinder(const Vec3& a, const Vec3& b, double radius, const Vec3& color, int seg) {
    Vec3 axis = tws::normalize(b - a);
    Vec3 u = tws::anyPerpendicular(axis);
    Vec3 w = tws::cross(axis, u);
    uint32_t base = (uint32_t)v.size();
    for (int i = 0; i <= seg; ++i) {
        double t = 2.0 * tws::kPi * i / seg;
        Vec3 n = u * std::cos(t) + w * std::sin(t);
        vert(a + n * radius, n, color);
        vert(b + n * radius, n, color);
    }
    for (int i = 0; i < seg; ++i) {
        uint32_t k = base + i * 2;
        tri(k, k + 2, k + 1);
        tri(k + 1, k + 2, k + 3);
    }
    // End caps.
    for (int end = 0; end < 2; ++end) {
        Vec3 c = end ? b : a;
        Vec3 n = end ? axis : -axis;
        uint32_t center = vert(c, n, color);
        uint32_t first = (uint32_t)v.size();
        for (int i = 0; i <= seg; ++i) {
            double t = 2.0 * tws::kPi * i / seg;
            vert(c + (u * std::cos(t) + w * std::sin(t)) * radius, n, color);
        }
        for (int i = 0; i < seg; ++i) {
            if (end)
                tri(center, first + i, first + i + 1);
            else
                tri(center, first + i + 1, first + i);
        }
    }
}

void MeshBuilder::capsule(const Vec3& a, const Vec3& b, double radius, const Vec3& color, int seg) {
    Vec3 d = b - a;
    double len = tws::length(d);
    Vec3 axis = len > 1e-9 ? d / len : Vec3{0, 1, 0};
    Vec3 u = tws::anyPerpendicular(axis);
    Vec3 w = tws::cross(axis, u);
    int rings = seg / 2;
    // Latitude rings from the bottom pole (at a, -axis) to the top pole (at b, +axis).
    uint32_t base = (uint32_t)v.size();
    int rows = 0;
    for (int r = 0; r <= rings; ++r) {
        double phi = -tws::kPi / 2 + tws::kPi * r / rings;
        Vec3 center = (r <= rings / 2) ? a : b;
        // Duplicate the equator so the cylinder section spans a -> b.
        for (int dup = 0; dup < ((r == rings / 2) ? 2 : 1); ++dup) {
            Vec3 c = dup ? b : center;
            for (int i = 0; i <= seg; ++i) {
                double t = 2.0 * tws::kPi * i / seg;
                Vec3 n = (u * std::cos(t) + w * std::sin(t)) * std::cos(phi) + axis * std::sin(phi);
                vert(c + n * radius, n, color);
            }
            ++rows;
        }
    }
    for (int r = 0; r < rows - 1; ++r)
        for (int i = 0; i < seg; ++i) {
            uint32_t k0 = base + r * (seg + 1) + i, k1 = k0 + seg + 1;
            tri(k0, k1, k0 + 1);
            tri(k0 + 1, k1, k1 + 1);
        }
}

void MeshBuilder::box(const Vec3& c, const Vec3& h, const Quat& q, const Vec3& color) {
    static const int faces[6][4] = {{1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1}};
    static const Vec3 normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    Vec3 corners[8];
    for (int i = 0; i < 8; ++i)
        corners[i] = c + q.rotate({(i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z});
    for (int f = 0; f < 6; ++f) {
        Vec3 n = q.rotate(normals[f]);
        uint32_t k = (uint32_t)v.size();
        for (int j = 0; j < 4; ++j) vert(corners[faces[f][j]], n, color);
        tri(k, k + 1, k + 2);
        tri(k, k + 2, k + 3);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Shaders

namespace {

const char* kLitVS = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
uniform mat4 uVP;
out vec3 vWorld;
out vec3 vNormal;
out vec3 vColor;
void main() {
    vWorld = aPos;
    vNormal = aNormal;
    vColor = aColor;
    gl_Position = uVP * vec4(aPos, 1.0);
}
)";

// Stylised: wrapped sun, sky/ground hemisphere ambient, a soft rim, distance fog; optional 1 m grid for scale.
const char* kLitFS = R"(#version 330 core
in vec3 vWorld;
in vec3 vNormal;
in vec3 vColor;
uniform vec3 uEye;
uniform vec3 uSky;
uniform float uGrid;
out vec4 oColor;
void main() {
    vec3 n = normalize(vNormal);
    vec3 L = normalize(vec3(0.45, 0.8, 0.35));
    float sun = clamp(dot(n, L) * 0.65 + 0.35, 0.0, 1.0);
    vec3 amb = mix(vec3(0.38, 0.34, 0.30), uSky, n.y * 0.5 + 0.5);
    vec3 V = normalize(uEye - vWorld);
    float rim = pow(1.0 - clamp(dot(n, V), 0.0, 1.0), 3.0) * 0.12;
    vec3 c = vColor * (vec3(1.0, 0.96, 0.88) * sun * 0.78 + amb * 0.42) + rim;
    if (uGrid > 0.5) {
        vec2 g = abs(fract(vWorld.xz - 0.5) - 0.5) / fwidth(vWorld.xz);
        float line = 1.0 - min(min(g.x, g.y), 1.0);
        c *= 1.0 - 0.16 * line;
    }
    float d = length(uEye - vWorld);
    float fog = 1.0 - exp(-d * 0.012);
    c = mix(c, uSky, clamp(fog, 0.0, 1.0));
    oColor = vec4(c, 1.0);
}
)";

const char* kLineVS = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uVP;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uVP * vec4(aPos, 1.0);
}
)";

const char* kLineFS = R"(#version 330 core
in vec3 vColor;
out vec4 oColor;
void main() { oColor = vec4(vColor, 1.0); }
)";

GLuint compile(GLenum type, const char* src, std::string& err) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        err = log;
        return 0;
    }
    return s;
}

GLuint program(const char* vs, const char* fs, std::string& err) {
    GLuint v = compile(GL_VERTEX_SHADER, vs, err);
    if (!v) return 0;
    GLuint f = compile(GL_FRAGMENT_SHADER, fs, err);
    if (!f) return 0;
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(p, sizeof log, nullptr, log);
        err = log;
        return 0;
    }
    return p;
}

}  // namespace

bool Renderer::init(std::string& err) {
    lit_ = program(kLitVS, kLitFS, err);
    if (!lit_) return false;
    line_ = program(kLineVS, kLineFS, err);
    if (!line_) return false;
    litVP_ = glGetUniformLocation(lit_, "uVP");
    litEye_ = glGetUniformLocation(lit_, "uEye");
    litGrid_ = glGetUniformLocation(lit_, "uGrid");
    litSky_ = glGetUniformLocation(lit_, "uSky");
    lineVP_ = glGetUniformLocation(line_, "uVP");
    glGenVertexArrays(1, &lineVao_);
    glGenBuffers(1, &lineVbo_);
    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineVtx), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(LineVtx), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    return true;
}

void Renderer::begin(int w, int h, const Vec3& eye, const Mat4& vp) {
    vp_ = vp;
    eye_ = eye;
    glViewport(0, 0, w, h);
    glClearColor((float)sky.x, (float)sky.y, (float)sky.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);  // a few procedural meshes mix windings; the cost is negligible at this scale
}

void Renderer::drawMesh(const Mesh& mesh, bool grid) {
    glUseProgram(lit_);
    glUniformMatrix4fv(litVP_, 1, GL_FALSE, vp_.m);
    glUniform3f(litEye_, (float)eye_.x, (float)eye_.y, (float)eye_.z);
    glUniform3f(litSky_, (float)sky.x, (float)sky.y, (float)sky.z);
    glUniform1f(litGrid_, grid ? 1.0f : 0.0f);
    mesh.draw();
}

void Renderer::drawLines(const std::vector<LineVtx>& lines) {
    if (lines.empty()) return;
    glDisable(GL_DEPTH_TEST);
    glUseProgram(line_);
    glUniformMatrix4fv(lineVP_, 1, GL_FALSE, vp_.m);
    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(lines.size() * sizeof(LineVtx)), lines.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_LINES, 0, (GLsizei)lines.size());
    glEnable(GL_DEPTH_TEST);
}

// ---------------------------------------------------------------------------------------------------------------
// Scene builders

namespace {

float hash01(int a, int b) {
    uint32_t h = (uint32_t)a * 73856093u ^ (uint32_t)b * 19349663u;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return (h & 0xFFFF) / 65535.0f;
}

Vec3 matColour(tws::MaterialId id) {
    const tws::Material& m = tws::material(id);
    return {m.r, m.g, m.b};
}

}  // namespace

void buildTerrainMesh(const tws::Terrain& t, MeshBuilder& out) {
    out.clear();
    const int nx = t.nx(), nz = t.nz();
    const double c = t.cell();
    out.v.reserve((size_t)(nx + 1) * (nz + 1) + 20000);
    out.idx.reserve((size_t)nx * nz * 6 + 60000);
    for (int k = 0; k <= nz; ++k)
        for (int i = 0; i <= nx; ++i) {
            double x = t.x0() + i * c, z = t.z0() + k * c;
            double hl = t.h(std::max(i - 1, 0), k), hr = t.h(std::min(i + 1, nx), k);
            double hd = t.h(i, std::max(k - 1, 0)), hu = t.h(i, std::min(k + 1, nz));
            Vec3 n = tws::normalize(Vec3{(hl - hr) / (2 * c), 1.0, (hd - hu) / (2 * c)});
            // Stylised: flat material colour with a gentle large-scale variation so the ground reads at speed.
            float var = 0.94f + 0.08f * hash01(i / 6, k / 6) + 0.03f * hash01(i, k);
            Vec3 col = matColour(t.mat(i, k)) * var;
            out.vert({x, t.h(i, k), z}, n, col);
        }
    for (int k = 0; k < nz; ++k)
        for (int i = 0; i < nx; ++i) {
            uint32_t a = (uint32_t)(k * (nx + 1) + i), b = a + 1, d = a + nx + 1, e = d + 1;
            out.tri(a, d, b);
            out.tri(b, d, e);
        }
    for (const tws::Capsule& o : t.obstacles()) out.capsule(o.a, o.b, o.radius, matColour(o.mat), 24);
    // The flat world beyond the grid: four big quads around it, out to 1.5 km (fog hides the edge).
    const double far = 1500.0;
    const double gx0 = t.x0(), gx1 = t.x0() + nx * c, gz0 = t.z0(), gz1 = t.z0() + nz * c;
    const Vec3 up{0, 1, 0}, base = matColour(tws::MaterialId::Hardpack) * 0.98;
    auto quad = [&](double xa, double za, double xb, double zb) {
        uint32_t k = out.vert({xa, 0, za}, up, base);
        out.vert({xb, 0, za}, up, base);
        out.vert({xb, 0, zb}, up, base);
        out.vert({xa, 0, zb}, up, base);
        out.tri(k, k + 1, k + 2);
        out.tri(k, k + 2, k + 3);
    };
    quad(-far, -far, far, gz0);
    quad(-far, gz1, far, far);
    quad(-far, gz0, gx0, gz1);
    quad(gx1, gz0, far, gz1);
}

void buildWheel(const tws::TyreParams& p, const tws::TyreState& s, const Vec3& hubNow, const Quat& carrier,
                double spinAngle, const Vec3& offset, bool contactColours, MeshBuilder& out) {
    const int nc = p.sections, nw = p.profile_points;
    const Vec3 hub = hubNow + offset;
    const Vec3 axle = carrier.rotate({1, 0, 0});
    auto P = [&](int i, int j) { return s.point[(size_t)(((i % nc) + nc) % nc) * nw + j] + offset; };

    // Tread surface.
    const Vec3 rubber{0.13, 0.13, 0.14};
    uint32_t base = (uint32_t)out.v.size();
    for (int i = 0; i < nc; ++i)
        for (int j = 0; j < nw; ++j) {
            Vec3 du = P(i + 1, j) - P(i - 1, j);
            Vec3 dv = P(i, std::min(j + 1, nw - 1)) - P(i, std::max(j - 1, 0));
            Vec3 n = tws::normalize(tws::cross(dv, du));
            if (tws::dot(n, P(i, j) - hub) < 0) n = -n;
            Vec3 col = rubber;
            size_t e = (size_t)i * nw + j;
            if (contactColours && s.load[e] > 0) {
                double t = std::min(s.load[e] / (2.0 * p.pressure), 1.0);
                col = s.sliding[e] ? Vec3{0.15, 0.75, 0.95} : tws::lerp(Vec3{0.98, 0.85, 0.15}, Vec3{0.95, 0.20, 0.10}, t);
            }
            out.vert(P(i, j), n, col);
        }
    for (int i = 0; i < nc; ++i)
        for (int j = 0; j < nw - 1; ++j) {
            uint32_t a = base + (uint32_t)(i * nw + j), b = base + (uint32_t)(((i + 1) % nc) * nw + j);
            out.tri(a, b, a + 1);
            out.tri(a + 1, b, b + 1);
        }

    // Sidewalls: from the tread edges down to the rim flanges.
    const double flange = 0.035;
    for (int side = 0; side < 2; ++side) {
        int j = side ? nw - 1 : 0;
        double sx = side ? 1.0 : -1.0;
        uint32_t sb = (uint32_t)out.v.size();
        for (int i = 0; i < nc; ++i) {
            double th = 2.0 * tws::kPi * i / nc;
            Vec3 radial = carrier.rotate({0, -std::cos(th), std::sin(th)});
            Vec3 rimPt = hub + radial * p.rim_radius + axle * (sx * flange);
            Vec3 n = tws::normalize(axle * sx + radial * 0.3);
            out.vert(P(i, j), n, rubber);
            out.vert(rimPt, n, rubber);
        }
        for (int i = 0; i < nc; ++i) {
            uint32_t a = sb + i * 2, b = sb + ((i + 1) % nc) * 2;
            if (side) {
                out.tri(a, b, a + 1);
                out.tri(a + 1, b, b + 1);
            } else {
                out.tri(a, a + 1, b);
                out.tri(a + 1, b + 1, b);
            }
        }
    }

    // Rim, hub and spokes turn with the wheel's spin angle.
    const Quat spun = carrier * Quat::axisAngle({1, 0, 0}, spinAngle);
    const Vec3 metal{0.72, 0.74, 0.78}, anodised{0.20, 0.35, 0.80};
    const int rimSeg = 48;
    for (int i = 0; i < rimSeg; ++i) {
        double t0 = 2.0 * tws::kPi * i / rimSeg, t1 = 2.0 * tws::kPi * (i + 1) / rimSeg;
        Vec3 r0 = carrier.rotate({0, -std::cos(t0), std::sin(t0)}) * (p.rim_radius - 0.01);
        Vec3 r1 = carrier.rotate({0, -std::cos(t1), std::sin(t1)}) * (p.rim_radius - 0.01);
        out.cylinder(hub + r0, hub + r1, 0.012, anodised, 6);
    }
    out.cylinder(hub - axle * 0.07, hub + axle * 0.07, 0.045, metal, 16);
    for (int i = 0; i < 18; ++i) {
        double t = 2.0 * tws::kPi * i / 18;
        double sx = (i % 2) ? 0.05 : -0.05;
        Vec3 end = hub + spun.rotate({0, -std::cos(t), std::sin(t)}) * (p.rim_radius - 0.015);
        out.cylinder(hub + axle * sx, end, 0.0035, metal, 5);
    }
    // A marker on the rim so spin is easy to see.
    out.box(hub + spun.rotate({0, -(p.rim_radius - 0.02), 0}), {0.02, 0.012, 0.03}, spun, {0.95, 0.55, 0.10});
}

void buildWheelMesh(const tws::WheelLab& lab, const Vec3& offset, const WheelDrawOptions& opt, MeshBuilder& out) {
    out.clear();
    buildWheel(lab.params, lab.tyre, lab.pos, lab.carrier, lab.spin_angle, offset, opt.contact_colours, out);
    const Vec3 hub = lab.pos + offset;
    const Vec3 axle = lab.carrier.rotate({1, 0, 0});
    if (lab.mode == tws::LabMode::Rig && opt.show_rig) {
        // Quarter-bike rig: fork legs from the axle up to the sprung load, plus the load itself.
        // Drawn 0.3 m higher than the sim's shock geometry so the load sits clear above the tyre.
        Vec3 top{hub.x, lab.sprung_y + offset.y + 0.3, hub.z};
        Vec3 orange{0.95, 0.45, 0.10}, grey{0.30, 0.30, 0.33};
        out.cylinder(hub + axle * 0.10, top + axle * 0.10, 0.022, grey, 10);
        out.cylinder(hub - axle * 0.10, top - axle * 0.10, 0.022, grey, 10);
        out.cylinder(hub - axle * 0.10, hub + axle * 0.10, 0.015, grey, 8);
        out.box(top + Vec3{0, 0.06, 0}, {0.16, 0.06, 0.20}, Quat{}, orange);
    }
}
