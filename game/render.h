// Lean forward renderer: one stylised lit shader (hemisphere ambient + wrapped sun + fog, optional 1 m grid) and one
// line shader. Meshes are plain vertex/index buffers. Everything the sim knows is drawn from sim state; the renderer
// owns no gameplay state.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "gl.h"
#include "vecmath.h"

namespace tws {
class Terrain;
class WheelLab;
}

struct Vtx {
    float px, py, pz;
    float nx, ny, nz;
    float r, g, b;
};

struct LineVtx {
    float px, py, pz;
    float r, g, b;
};

struct Mat4 {
    float m[16];  // column-major
    static Mat4 perspective(float fovyRad, float aspect, float zn, float zf);
    static Mat4 lookAt(const tws::Vec3& eye, const tws::Vec3& target, const tws::Vec3& up);
    Mat4 operator*(const Mat4& o) const;
};

class Mesh {
public:
    void upload(const std::vector<Vtx>& v, const std::vector<uint32_t>& idx, bool dynamic);
    void draw() const;
    bool empty() const { return count_ == 0; }

private:
    GLuint vao_ = 0, vbo_ = 0, ebo_ = 0;
    int count_ = 0;
};

// Builds triangles into a vertex/index list.
struct MeshBuilder {
    std::vector<Vtx> v;
    std::vector<uint32_t> idx;
    void clear() { v.clear(); idx.clear(); }
    uint32_t vert(const tws::Vec3& p, const tws::Vec3& n, const tws::Vec3& c);
    void tri(uint32_t a, uint32_t b, uint32_t c) { idx.insert(idx.end(), {a, b, c}); }
    // Primitives (flat colour, smooth normals).
    void cylinder(const tws::Vec3& a, const tws::Vec3& b, double radius, const tws::Vec3& color, int segments = 12);
    void capsule(const tws::Vec3& a, const tws::Vec3& b, double radius, const tws::Vec3& color, int segments = 20);
    void box(const tws::Vec3& center, const tws::Vec3& half, const tws::Quat& rot, const tws::Vec3& color);
};

class Renderer {
public:
    bool init(std::string& error);
    void begin(int width, int height, const tws::Vec3& eye, const Mat4& viewProj);
    void drawMesh(const Mesh& mesh, bool grid);
    void drawLines(const std::vector<LineVtx>& lines);

    tws::Vec3 sky{0.62, 0.78, 0.92};

private:
    GLuint lit_ = 0, line_ = 0;
    GLint litVP_ = -1, litEye_ = -1, litGrid_ = -1, litSky_ = -1;
    GLint lineVP_ = -1;
    GLuint lineVao_ = 0, lineVbo_ = 0;
    Mat4 vp_{};
    tws::Vec3 eye_;
};

// Scene builders.
void buildTerrainMesh(const tws::Terrain& t, MeshBuilder& out);
struct WheelDrawOptions {
    bool contact_colours = true;
    bool show_rig = true;
};
// `offset` shifts the drawn wheel (render interpolation of the hub between two sim states).
void buildWheelMesh(const tws::WheelLab& lab, const tws::Vec3& offset, const WheelDrawOptions& opt, MeshBuilder& out);
