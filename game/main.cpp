// Two Wheel Simulator: M0 wheel lab. One wheel + flexible tyre on rigid ground, a free mode and a quarter-bike rig.
//
// Command line (all optional):
//   --mode rig|free   --scene lane|flat   --speed <m/s>   --size <w>x<h>   --position <x>,<y>
//   --screenshot <file.png> [--frames <n>]   renders n frames at a fixed 1/60 s each, saves, and exits
//   --cam <yaw>,<pitch>,<dist>               --no-ui
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "gl.h"
#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl3.h"
#include "png.h"
#include "render.h"
#include "wheel_lab.h"

using namespace tws;

namespace {

struct Options {
    LabMode mode = LabMode::Rig;
    LabScene scene = LabScene::ObstacleLane;
    double speed = 4.0;
    int width = 1600, height = 900;
    bool hasPos = false;
    int posX = 0, posY = 0;
    std::string screenshot;
    int frames = 120;
    bool ui = true;
    float camYaw = 70, camPitch = 10, camDist = 2.2f;
};

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--mode") o.mode = std::strcmp(next(), "free") == 0 ? LabMode::Free : LabMode::Rig;
        else if (a == "--scene") o.scene = std::strcmp(next(), "flat") == 0 ? LabScene::Flat : LabScene::ObstacleLane;
        else if (a == "--speed") o.speed = std::atof(next());
        else if (a == "--size") std::sscanf(next(), "%dx%d", &o.width, &o.height);
        else if (a == "--position") o.hasPos = std::sscanf(next(), "%d,%d", &o.posX, &o.posY) == 2;
        else if (a == "--screenshot") o.screenshot = next();
        else if (a == "--frames") o.frames = std::atoi(next());
        else if (a == "--no-ui") o.ui = false;
        else if (a == "--cam") std::sscanf(next(), "%f,%f,%f", &o.camYaw, &o.camPitch, &o.camDist);
    }
    return o;
}

void* getProc(const char* name) { return reinterpret_cast<void*>(SDL_GL_GetProcAddress(name)); }

struct History {
    static constexpr int N = 400;  // 4 s at one sample per 10 ticks
    float fx[N] = {}, fy[N] = {}, fz[N] = {};
    int head = 0;
    void push(float x, float y, float z) {
        fx[head] = x;
        fy[head] = y;
        fz[head] = z;
        head = (head + 1) % N;
    }
};

void resetLab(WheelLab& lab, double freeSpeed, double freeLean) {
    if (lab.mode == LabMode::Free) {
        lab.resetFree({0, lab.params.outer_radius + 0.05, 1.0}, freeSpeed, freeLean);
    } else {
        lab.resetRig();
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options opt = parseArgs(argc, argv);

    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    std::string title = std::string("Two Wheel Simulator v") + TWS_VERSION + " - wheel lab";
    SDL_Window* window = SDL_CreateWindow(title.c_str(), opt.width, opt.height,
                                          SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 1;
    }
    if (opt.hasPos) SDL_SetWindowPosition(window, opt.posX, opt.posY);
    SDL_GLContext ctx = SDL_GL_CreateContext(window);
    if (!ctx) {
        // Retry without multisampling (some drivers refuse it).
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
        ctx = SDL_GL_CreateContext(window);
    }
    if (!ctx) {
        std::fprintf(stderr, "OpenGL 3.3 context failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_MakeCurrent(window, ctx);
    SDL_GL_SetSwapInterval(0);  // no FPS cap
    if (const char* missing = loadGL(getProc)) {
        std::fprintf(stderr, "Missing OpenGL function %s\n", missing);
        return 1;
    }
    SDL_ShowWindow(window);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGui::GetStyle().WindowRounding = 6.0f;
    ImGui::GetStyle().FrameRounding = 4.0f;
    ImGui_ImplSDL3_InitForOpenGL(window, ctx);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    Renderer renderer;
    std::string err;
    if (!renderer.init(err)) {
        std::fprintf(stderr, "Shader error: %s\n", err.c_str());
        return 1;
    }

    WheelLab lab;
    lab.setScene(opt.scene);
    lab.rig.speed = opt.speed;
    double freeSpeed = opt.speed, freeLean = 4.0;
    lab.mode = opt.mode;
    resetLab(lab, freeSpeed, freeLean);

    MeshBuilder mb;
    Mesh terrainMesh, wheelMesh;
    buildTerrainMesh(lab.terrain, mb);
    terrainMesh.upload(mb.v, mb.idx, false);

    // View state.
    float camYaw = opt.camYaw, camPitch = opt.camPitch, camDist = opt.camDist;
    Vec3 camTarget = lab.pos;
    bool paused = false, showUi = opt.ui, contactColours = true, showForces = true, showGrid = true, showRig = true;
    float timeScale = 1.0f;
    float pressurePsi = (float)(lab.params.pressure / 6894.76);
    Vec3 prevPos = lab.pos, prevSprung{0, lab.sprung_y, 0};
    double accumulator = 0;
    double tickMicros = 0;
    History hist;
    int frame = 0;
    bool keyDrive = false, keyBrake = false;

    uint64_t lastCounter = SDL_GetPerformanceCounter();
    const double freq = (double)SDL_GetPerformanceFrequency();
    double fpsTimer = 0, fpsShown = 0;
    int fpsFrames = 0;
    bool running = true;

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            ImGui_ImplSDL3_ProcessEvent(&ev);
            if (ev.type == SDL_EVENT_QUIT) running = false;
            ImGuiIO& io = ImGui::GetIO();
            if (ev.type == SDL_EVENT_KEY_DOWN && !io.WantCaptureKeyboard) {
                switch (ev.key.key) {
                    case SDLK_ESCAPE: running = false; break;
                    case SDLK_SPACE: paused = !paused; break;
                    case SDLK_R: resetLab(lab, freeSpeed, freeLean); prevPos = lab.pos; break;
                    case SDLK_F1: showUi = !showUi; break;
                    case SDLK_PERIOD: if (paused) lab.step(); break;
                    default: break;
                }
            }
            if (ev.type == SDL_EVENT_MOUSE_MOTION && (ev.motion.state & SDL_BUTTON_RMASK) && !io.WantCaptureMouse) {
                camYaw -= ev.motion.xrel * 0.3f;
                camPitch = std::clamp(camPitch + ev.motion.yrel * 0.3f, -10.0f, 85.0f);
            }
            if (ev.type == SDL_EVENT_MOUSE_WHEEL && !io.WantCaptureMouse)
                camDist = std::clamp(camDist * (ev.wheel.y > 0 ? 0.9f : 1.1f), 0.5f, 30.0f);
        }
        {
            const bool* keys = SDL_GetKeyboardState(nullptr);
            bool cap = ImGui::GetIO().WantCaptureKeyboard;
            keyDrive = !cap && keys[SDL_SCANCODE_W];
            keyBrake = !cap && keys[SDL_SCANCODE_S];
        }

        uint64_t now = SDL_GetPerformanceCounter();
        double frameDt = (now - lastCounter) / freq;
        lastCounter = now;
        if (!opt.screenshot.empty()) frameDt = 1.0 / 60.0;  // deterministic screenshots
        fpsTimer += frameDt;
        ++fpsFrames;
        if (fpsTimer >= 0.5) {
            fpsShown = fpsFrames / fpsTimer;
            fpsTimer = 0;
            fpsFrames = 0;
        }

        // --- Simulation: fixed 1 kHz ticks. ---
        lab.params.pressure = pressurePsi * 6894.76;
        double rigDrive = lab.rig.drive_torque, rigBrake = lab.rig.brake_torque;
        double freeDrive = lab.free_drive_torque, freeBrake = lab.free_brake_torque;
        if (keyDrive) { lab.rig.drive_torque = std::max(lab.rig.drive_torque, 300.0); lab.free_drive_torque = std::max(lab.free_drive_torque, 60.0); }
        if (keyBrake) { lab.rig.brake_torque = std::max(lab.rig.brake_torque, 800.0); lab.free_brake_torque = std::max(lab.free_brake_torque, 200.0); }
        if (!paused) {
            accumulator += std::min(frameDt, 0.1) * timeScale;
            int ticks = 0;
            uint64_t t0 = SDL_GetPerformanceCounter();
            while (accumulator >= WheelLab::kDt && ticks < 200) {
                prevPos = lab.pos;
                prevSprung.y = lab.sprung_y;
                lab.step();
                accumulator -= WheelLab::kDt;
                ++ticks;
                if (lab.ticks % 10 == 0) {
                    if (lab.mode == LabMode::Rig)
                        hist.push((float)lab.measure.fx, (float)lab.measure.fy, (float)lab.measure.fz);
                    else
                        hist.push((float)lab.out.force.z, (float)lab.out.force.x, (float)lab.out.force.y);
                }
            }
            if (ticks) tickMicros = tickMicros * 0.9 + 0.1 * ((SDL_GetPerformanceCounter() - t0) / freq * 1e6 / ticks);
            if (accumulator > WheelLab::kDt) accumulator = 0;  // fell behind: drop time rather than spiral
        }
        lab.rig.drive_torque = rigDrive; lab.rig.brake_torque = rigBrake;
        lab.free_drive_torque = freeDrive; lab.free_brake_torque = freeBrake;
        // Free wheel lost off the edge of the world: put it back.
        if (lab.mode == LabMode::Free && lab.pos.y < -3) { resetLab(lab, freeSpeed, freeLean); prevPos = lab.pos; }

        // Interpolate the hub between the last two ticks (a wrap of the rig lane snaps instead).
        double alpha = paused ? 1.0 : std::clamp(accumulator / WheelLab::kDt, 0.0, 1.0);
        Vec3 drawPos = length(lab.pos - prevPos) > 1.0 ? lab.pos : lerp(prevPos, lab.pos, alpha);
        Vec3 offset = drawPos - lab.pos;

        // --- Camera: orbit, following the hub smoothly. ---
        camTarget = length(drawPos - camTarget) > 3.0 ? drawPos : lerp(camTarget, drawPos, std::min(1.0, frameDt * 10.0));
        int w = 0, h = 0;
        SDL_GetWindowSizeInPixels(window, &w, &h);
        double yaw = deg2rad(camYaw), pitch = deg2rad(camPitch);
        Vec3 eye = camTarget + Vec3{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)} * camDist;
        eye.y = std::max(eye.y, lab.terrain.heightAt(eye.x, eye.z) + 0.05);
        Mat4 vp = Mat4::perspective((float)deg2rad(55.0), (float)w / std::max(h, 1), 0.02f, 400.0f) *
                  Mat4::lookAt(eye, camTarget, {0, 1, 0});

        // --- Draw. ---
        renderer.begin(w, h, eye, vp);
        renderer.drawMesh(terrainMesh, showGrid);
        WheelDrawOptions wopt;
        wopt.contact_colours = contactColours;
        wopt.show_rig = showRig;
        buildWheelMesh(lab, offset, wopt, mb);
        wheelMesh.upload(mb.v, mb.idx, true);
        renderer.drawMesh(wheelMesh, false);

        if (showForces && lab.out.contacts > 0) {
            // Total tyre force drawn from the load-weighted contact centre: 1 m per 2000 N.
            Vec3 centre;
            double wsum = 0;
            for (size_t e = 0; e < lab.tyre.load.size(); ++e)
                if (lab.tyre.load[e] > 0) {
                    centre += lab.tyre.point[e] * lab.tyre.load[e];
                    wsum += lab.tyre.load[e];
                }
            centre = centre / wsum + offset;
            Vec3 f = lab.out.force / 2000.0;
            Vec3 fh{f.x, 0, f.z};
            std::vector<LineVtx> lines = {
                {(float)centre.x, (float)centre.y, (float)centre.z, 0.2f, 1.0f, 0.3f},
                {(float)(centre.x + f.x), (float)(centre.y + f.y), (float)(centre.z + f.z), 0.2f, 1.0f, 0.3f},
                {(float)centre.x, (float)centre.y + 0.002f, (float)centre.z, 1.0f, 0.3f, 0.9f},
                {(float)(centre.x + fh.x * 4), (float)centre.y + 0.002f, (float)(centre.z + fh.z * 4), 1.0f, 0.3f, 0.9f},
            };
            renderer.drawLines(lines);
        }

        // --- UI. ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        if (showUi) {
            ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_Once);
            ImGui::SetNextWindowSize(ImVec2(430, 0), ImGuiCond_Once);
            ImGui::Begin("Wheel lab", nullptr, ImGuiWindowFlags_NoCollapse);
            ImGui::Text("%.0f FPS   sim %.1f us/tick (1000 ticks/s)", fpsShown, tickMicros);

            int mode = lab.mode == LabMode::Rig ? 0 : 1;
            bool changed = ImGui::RadioButton("Quarter-bike rig", &mode, 0);
            ImGui::SameLine();
            changed |= ImGui::RadioButton("Free wheel", &mode, 1);
            int scene = lab.scene() == LabScene::ObstacleLane ? 0 : 1;
            bool sceneChanged = ImGui::RadioButton("Obstacle lane", &scene, 0);
            ImGui::SameLine();
            sceneChanged |= ImGui::RadioButton("Flat", &scene, 1);
            if (sceneChanged) {
                lab.setScene(scene == 0 ? LabScene::ObstacleLane : LabScene::Flat);
                buildTerrainMesh(lab.terrain, mb);
                terrainMesh.upload(mb.v, mb.idx, false);
                changed = true;
            }
            if (changed) {
                lab.mode = mode == 0 ? LabMode::Rig : LabMode::Free;
                resetLab(lab, freeSpeed, freeLean);
                prevPos = lab.pos;
            }

            ImGui::SeparatorText("Tyre");
            ImGui::SliderFloat("Pressure (psi)", &pressurePsi, 3.0f, 30.0f, "%.1f");
            ImGui::SameLine();
            ImGui::TextDisabled("%.2f bar", pressurePsi * 0.0689476);

            if (lab.mode == LabMode::Rig) {
                ImGui::SeparatorText("Rig");
                float speed = (float)lab.rig.speed, slip = (float)lab.rig.slip_deg, camber = (float)lab.rig.camber_deg;
                float load = (float)lab.rig.load_kg, drive = (float)lab.rig.drive_torque, brake = (float)lab.rig.brake_torque;
                if (ImGui::SliderFloat("Speed (m/s)", &speed, 0.0f, 25.0f, "%.1f")) lab.rig.speed = speed;
                if (ImGui::SliderFloat("Slip angle (deg)", &slip, -30.0f, 30.0f, "%.1f")) lab.rig.slip_deg = slip;
                if (ImGui::SliderFloat("Camber / lean (deg)", &camber, -55.0f, 55.0f, "%.1f")) lab.rig.camber_deg = camber;
                if (ImGui::SliderFloat("Load (kg)", &load, 10.0f, 250.0f, "%.0f")) lab.rig.load_kg = load;
                if (ImGui::SliderFloat("Drive torque (N*m)", &drive, -400.0f, 1500.0f, "%.0f")) lab.rig.drive_torque = drive;
                if (ImGui::SliderFloat("Brake torque (N*m)", &brake, 0.0f, 2000.0f, "%.0f")) lab.rig.brake_torque = brake;
                if (ImGui::Button("Zero torques")) { lab.rig.drive_torque = 0; lab.rig.brake_torque = 0; }
                ImGui::SameLine();
                if (ImGui::Button("Lock wheel")) lab.rig.brake_torque = 5000;
                ImGui::SameLine();
                if (ImGui::Button("Reset (R)")) { lab.resetRig(); prevPos = lab.pos; }
            } else {
                ImGui::SeparatorText("Free wheel");
                float fs = (float)freeSpeed, fl = (float)freeLean;
                if (ImGui::SliderFloat("Launch speed (m/s)", &fs, 0.0f, 20.0f, "%.1f")) freeSpeed = fs;
                if (ImGui::SliderFloat("Launch lean (deg)", &fl, -30.0f, 30.0f, "%.1f")) freeLean = fl;
                if (ImGui::Button("Launch (R)")) { resetLab(lab, freeSpeed, freeLean); prevPos = lab.pos; }
                ImGui::SameLine();
                if (ImGui::Button("Push left")) lab.applyImpulse(lab.carrier.rotate({1, 0, 0}) * 12.0, lab.pos + Vec3{0, 0.25, 0});
                ImGui::SameLine();
                if (ImGui::Button("Push right")) lab.applyImpulse(lab.carrier.rotate({-1, 0, 0}) * 12.0, lab.pos + Vec3{0, 0.25, 0});
                ImGui::SameLine();
                if (ImGui::Button("Hop")) lab.applyImpulse({0, 45.0, 0}, lab.pos);
                float drive = (float)lab.free_drive_torque, brake = (float)lab.free_brake_torque;
                if (ImGui::SliderFloat("Drive torque (N*m)", &drive, -150.0f, 150.0f, "%.0f")) lab.free_drive_torque = drive;
                if (ImGui::SliderFloat("Brake torque (N*m)", &brake, 0.0f, 300.0f, "%.0f")) lab.free_brake_torque = brake;
            }
            ImGui::TextDisabled("Hold W = drive, S = brake");

            ImGui::SeparatorText("Sim and view");
            ImGui::Checkbox("Pause (Space)", &paused);
            ImGui::SameLine();
            if (ImGui::Button("Step 1 tick (.)") && paused) lab.step();
            ImGui::SliderFloat("Time scale", &timeScale, 0.02f, 1.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
            ImGui::Checkbox("Contact colours", &contactColours);
            ImGui::SameLine();
            ImGui::Checkbox("Force", &showForces);
            ImGui::SameLine();
            ImGui::Checkbox("Grid", &showGrid);
            ImGui::SameLine();
            ImGui::Checkbox("Rig", &showRig);
            ImGui::TextDisabled("Right-drag: orbit   Wheel: zoom   F1: hide panel");

            ImGui::SeparatorText("Measurements");
            const TyreOutput& o = lab.out;
            double speedNow = length(Vec3{lab.vel.x, 0, lab.vel.z});
            if (ImGui::BeginTable("m", 2, ImGuiTableFlags_SizingStretchProp)) {
                auto row = [](const char* name, const char* fmt, double v) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(name);
                    ImGui::TableNextColumn();
                    ImGui::Text(fmt, v);
                };
                if (lab.mode == LabMode::Rig) {
                    row("Longitudinal force Fx", "%.0f N", lab.measure.fx);
                    row("Lateral force Fy", "%.0f N", lab.measure.fy);
                    row("Vertical force Fz", "%.0f N", lab.measure.fz);
                    row("Horizontal / vertical force", "%.2f", std::hypot(lab.measure.fx, lab.measure.fy) / std::max(lab.measure.fz, 1.0));
                    row("Aligning moment Mz", "%.1f N*m", lab.measure.mz);
                    row("Slip ratio", "%.3f", lab.measure.slip_ratio);
                    row("Shock travel used", "%.0f mm", (lab.rig.travel - (lab.sprung_y - lab.pos.y)) * 1000.0);
                } else {
                    row("Ground force (N)", "%.0f", length(o.force));
                    row("Lean (deg)", "%.1f", rad2deg(std::asin(std::clamp(lab.carrier.rotate({1, 0, 0}).y, -1.0, 1.0))));
                }
                row("Speed", "%.2f m/s", speedNow);
                row("Wheel speed", "%.0f rpm", lab.spin * 60.0 / (2.0 * kPi));
                row("Tyre deflection", "%.1f mm", o.max_compression * 1000.0);
                row("Contact patch", "%.0f cm^2", o.patch_area * 1e4);
                row("Elements in contact", "%.0f", (double)o.contacts);
                row("Elements sliding", "%.0f", (double)o.sliding);
                ImGui::EndTable();
            }
            if (o.rim_strike) ImGui::TextColored(ImVec4(1, 0.35f, 0.25f, 1), "RIM STRIKE");
            ImGui::PlotLines("Fx", hist.fx, History::N, hist.head, nullptr, -2500, 2500, ImVec2(0, 40));
            ImGui::PlotLines("Fy", hist.fy, History::N, hist.head, nullptr, -2500, 2500, ImVec2(0, 40));
            ImGui::PlotLines("Fz", hist.fz, History::N, hist.head, nullptr, 0, 6000, ImVec2(0, 40));
            ImGui::TextDisabled("Tyre colours: yellow -> red = contact pressure, cyan = sliding");

            ImGui::End();

            // Footprint: the tyre unrolled round the bottom (front at the top), one cell per element.
            ImGui::SetNextWindowPos(ImVec2((float)w - 200, 12), ImGuiCond_Once);
            ImGui::Begin("Footprint", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
            ImGui::TextDisabled("front at top, +-112 deg");
            {
                const int nc = lab.params.sections, nw = lab.params.profile_points;
                const int half = 40;  // sections either side of straight down (+-112 degrees)
                const float cw = 22.0f, ch = 4.0f;
                ImDrawList* dl = ImGui::GetWindowDrawList();
                ImVec2 org = ImGui::GetCursorScreenPos();
                float x0 = org.x + 8;
                dl->AddRectFilled(ImVec2(x0 - 2, org.y), ImVec2(x0 + cw * nw + 2, org.y + ch * (2 * half + 1)), IM_COL32(25, 25, 28, 255));
                for (int r = -half; r <= half; ++r) {
                    int i = ((r % nc) + nc) % nc;
                    float y = org.y + (half - r) * ch;
                    for (int j = 0; j < nw; ++j) {
                        size_t e = (size_t)i * nw + j;
                        float ld = lab.tyre.load[e];
                        if (ld <= 0) continue;
                        ImU32 col;
                        if (lab.tyre.sliding[e]) {
                            col = IM_COL32(40, 190, 240, 255);
                        } else {
                            float t = std::min(ld / (float)(2.0 * lab.params.pressure), 1.0f);
                            col = IM_COL32(250, (int)(215 - 165 * t), (int)(40 - 15 * t), 255);
                        }
                        dl->AddRectFilled(ImVec2(x0 + j * cw + 1, y), ImVec2(x0 + (j + 1) * cw - 1, y + ch), col);
                    }
                }
                dl->AddLine(ImVec2(x0 - 6, org.y + half * ch + ch * 0.5f), ImVec2(x0 - 1, org.y + half * ch + ch * 0.5f), IM_COL32(200, 200, 200, 255));
                ImGui::Dummy(ImVec2(cw * nw + 12, ch * (2 * half + 1) + 4));
            }
            ImGui::End();
        }
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        ++frame;
        if (!opt.screenshot.empty() && frame >= opt.frames) {
            std::vector<uint8_t> px((size_t)w * h * 3), flipped((size_t)w * h * 3);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            for (int y = 0; y < h; ++y)
                std::memcpy(&flipped[(size_t)y * w * 3], &px[(size_t)(h - 1 - y) * w * 3], (size_t)w * 3);
            bool ok = writePng(opt.screenshot.c_str(), w, h, flipped.data());
            std::printf("%s %s (%dx%d), sim %.1f us/tick, renderer: %s\n", ok ? "Saved" : "FAILED to save",
                        opt.screenshot.c_str(), w, h, tickMicros, (const char*)glGetString(GL_RENDERER));
            running = false;
        }
        SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DestroyContext(ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
