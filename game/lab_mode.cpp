// Wheel lab (M0): one wheel + flexible tyre on rigid ground, as a free wheel or on a quarter-bike rig.
#include <algorithm>
#include <cmath>
#include <vector>

#include "imgui.h"
#include "modes.h"
#include "wheel_lab.h"

using namespace tws;

namespace {

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

class LabModeImpl : public GameMode {
public:
    LabModeImpl(const LabOptions& opt, Shared& sh) {
        lab_.setScene(opt.flat ? LabScene::Flat : LabScene::ObstacleLane);
        lab_.rig.speed = opt.speed;
        lab_.rig.drive_torque = opt.drive;
        freeSpeed_ = opt.speed;
        lab_.mode = opt.free_mode ? LabMode::Free : LabMode::Rig;
        camYaw_ = opt.cam_yaw;
        camPitch_ = opt.cam_pitch;
        camDist_ = opt.cam_dist;
        pressurePsi_ = (float)(lab_.params.pressure / 6894.76);
        resetLab();
        rebuildTerrain(sh);
        camTarget_ = lab_.pos;
    }

    const char* name() const override { return "Wheel lab"; }

    void event(const SDL_Event& ev, Shared& sh) override {
        ImGuiIO& io = ImGui::GetIO();
        if (ev.type == SDL_EVENT_KEY_DOWN && !io.WantCaptureKeyboard) {
            switch (ev.key.key) {
                case SDLK_SPACE: sh.paused = !sh.paused; break;
                case SDLK_R: resetLab(); break;
                case SDLK_PERIOD: if (sh.paused) lab_.step(); break;
                default: break;
            }
        }
        if (ev.type == SDL_EVENT_MOUSE_MOTION && (ev.motion.state & SDL_BUTTON_RMASK) && !io.WantCaptureMouse) {
            camYaw_ -= ev.motion.xrel * 0.3f;
            camPitch_ = std::clamp(camPitch_ + ev.motion.yrel * 0.3f, -10.0f, 85.0f);
        }
        if (ev.type == SDL_EVENT_MOUSE_WHEEL && !io.WantCaptureMouse)
            camDist_ = std::clamp(camDist_ * (ev.wheel.y > 0 ? 0.9f : 1.1f), 0.5f, 30.0f);
    }

    void update(double frameDt, Shared& sh) override {
        Pad& pad = *sh.pad;
        const bool* keys = SDL_GetKeyboardState(nullptr);
        const bool cap = ImGui::GetIO().WantCaptureKeyboard;
        const bool keyDrive = !cap && keys[SDL_SCANCODE_W];
        const bool keyBrake = !cap && keys[SDL_SCANCODE_S];

        if (pad.connected()) {
            if (pad.pressed(SDL_GAMEPAD_BUTTON_START)) sh.paused = !sh.paused;
            if (pad.pressed(SDL_GAMEPAD_BUTTON_NORTH)) { resetLab(); sh.haptics->reset(); }
            if (pad.pressed(SDL_GAMEPAD_BUTTON_EAST)) {
                lab_.mode = lab_.mode == LabMode::Rig ? LabMode::Free : LabMode::Rig;
                resetLab();
                sh.haptics->reset();
            }
            if (pad.pressed(SDL_GAMEPAD_BUTTON_WEST)) {
                lab_.setScene(lab_.scene() == LabScene::ObstacleLane ? LabScene::Flat : LabScene::ObstacleLane);
                rebuildTerrain(sh);
                resetLab();
            }
            if (pad.pressed(SDL_GAMEPAD_BUTTON_SOUTH) && lab_.mode == LabMode::Free) lab_.applyImpulse({0, 45.0, 0}, lab_.pos);
            if (pad.pressed(SDL_GAMEPAD_BUTTON_DPAD_LEFT)) pressurePsi_ = std::max(0.0f, pressurePsi_ - 1.0f);
            if (pad.pressed(SDL_GAMEPAD_BUTTON_DPAD_RIGHT)) pressurePsi_ = std::min(30.0f, pressurePsi_ + 1.0f);
            if (pad.pressed(SDL_GAMEPAD_BUTTON_DPAD_UP)) {
                if (lab_.mode == LabMode::Rig) lab_.rig.speed = std::min(25.0, lab_.rig.speed + 0.5);
                else freeSpeed_ = std::min(20.0, freeSpeed_ + 0.5);
            }
            if (pad.pressed(SDL_GAMEPAD_BUTTON_DPAD_DOWN)) {
                if (lab_.mode == LabMode::Rig) lab_.rig.speed = std::max(0.0, lab_.rig.speed - 0.5);
                else freeSpeed_ = std::max(0.0, freeSpeed_ - 0.5);
            }
            if (lab_.mode == LabMode::Rig) {
                double lean = (pad.held(SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER) ? 1.0 : 0.0) - (pad.held(SDL_GAMEPAD_BUTTON_LEFT_SHOULDER) ? 1.0 : 0.0);
                lab_.rig.camber_deg = std::clamp(lab_.rig.camber_deg + lean * 30.0 * frameDt, -55.0, 55.0);
            }
            if (pad.pressed(SDL_GAMEPAD_BUTTON_LEFT_STICK)) camDist_ = camDist_ > 3.0f ? 1.4f : (camDist_ > 1.8f ? 4.0f : 2.2f);
            if (pad.pressed(SDL_GAMEPAD_BUTTON_RIGHT_STICK)) { camYaw_ = 70; camPitch_ = 10; }
            camYaw_ -= pad.rightX() * 150.0f * (float)frameDt;
            camPitch_ = std::clamp(camPitch_ + pad.rightY() * 100.0f * (float)frameDt, -10.0f, 85.0f);
        }

        lab_.params.pressure = pressurePsi_ * 6894.76;
        // Temporary overrides from held keys / triggers / stick, restored after the ticks.
        const double rigDrive = lab_.rig.drive_torque, rigBrake = lab_.rig.brake_torque, rigSlip = lab_.rig.slip_deg;
        const double freeDrive = lab_.free_drive_torque, freeBrake = lab_.free_brake_torque;
        if (keyDrive) { lab_.rig.drive_torque = std::max(lab_.rig.drive_torque, 300.0); lab_.free_drive_torque = std::max(lab_.free_drive_torque, 60.0); }
        if (keyBrake) { lab_.rig.brake_torque = std::max(lab_.rig.brake_torque, 800.0); lab_.free_brake_torque = std::max(lab_.free_brake_torque, 200.0); }
        const float rt = pad.rightTrigger(), lt = pad.leftTrigger();
        if (rt > 0) { lab_.rig.drive_torque = std::max(lab_.rig.drive_torque, rt * 700.0); lab_.free_drive_torque = std::max(lab_.free_drive_torque, rt * 100.0); }
        if (lt > 0) { lab_.rig.brake_torque = std::max(lab_.rig.brake_torque, lt * lt * 2500.0); lab_.free_brake_torque = std::max(lab_.free_brake_torque, lt * lt * 300.0); }
        if (pad.leftX() != 0.0f) {
            if (lab_.mode == LabMode::Rig) lab_.rig.slip_deg = std::clamp(lab_.rig.slip_deg + pad.leftX() * 15.0, -30.0, 30.0);
            else if (!sh.paused) lab_.applyImpulse(lab_.carrier.rotate({-1, 0, 0}) * (pad.leftX() * 60.0 * frameDt * sh.time_scale), lab_.pos + Vec3{0, 0.25, 0});
        }
        const bool driving = keyDrive || rt > 0 || lab_.rig.drive_torque > 0 || lab_.free_drive_torque > 0;
        const bool braking = keyBrake || lt > 0 || lab_.rig.brake_torque > 0 || lab_.free_brake_torque > 0;

        if (!sh.paused) {
            accumulator_ += frameDt * sh.time_scale;
            int ticks = 0;
            uint64_t t0 = SDL_GetPerformanceCounter();
            while (accumulator_ >= WheelLab::kDt && ticks < 200) {
                prevPos_ = lab_.pos;
                lab_.step();
                accumulator_ -= WheelLab::kDt;
                ++ticks;
                if (lab_.ticks % 10 == 0) {
                    if (lab_.mode == LabMode::Rig)
                        hist_.push((float)lab_.measure.fx, (float)lab_.measure.fy, (float)lab_.measure.fz);
                    else
                        hist_.push((float)lab_.out.force.z, (float)lab_.out.force.x, (float)lab_.out.force.y);
                }
            }
            if (ticks)
                sh.tick_micros = sh.tick_micros * 0.9 + 0.1 * ((SDL_GetPerformanceCounter() - t0) / (double)SDL_GetPerformanceFrequency() * 1e6 / ticks);
            if (accumulator_ > WheelLab::kDt) accumulator_ = 0;  // fell behind: drop time rather than spiral
        }
        lab_.rig.drive_torque = rigDrive;
        lab_.rig.brake_torque = rigBrake;
        lab_.rig.slip_deg = rigSlip;
        lab_.free_drive_torque = freeDrive;
        lab_.free_brake_torque = freeBrake;

        HapticSignal s;
        const TyreOutput& o = lab_.out;
        s.load = o.normal_load;
        s.rest_load = (lab_.params.mass + (lab_.mode == LabMode::Rig ? lab_.rig.load_kg : 0.0)) * kGravity;
        s.rim_strike = o.rim_strike;
        s.slide_rear = s.slide_front = o.contacts > 0 ? (double)o.sliding / o.contacts : 0.0;
        s.speed = std::sqrt(lab_.vel.x * lab_.vel.x + lab_.vel.z * lab_.vel.z);
        s.in_contact = o.contacts > 0;
        s.driving = driving;
        s.braking = braking;
        sh.haptics->update(s, sh.paused ? 0.0 : frameDt);

        if (lab_.mode == LabMode::Free && (lab_.pos.y < -3 || std::fabs(lab_.pos.x) > 1000 || std::fabs(lab_.pos.z) > 1000)) resetLab();
        frameDt_ = frameDt;
        paused_ = sh.paused;
    }

    void render(Shared& sh, int w, int h) override {
        double alpha = paused_ ? 1.0 : std::clamp(accumulator_ / WheelLab::kDt, 0.0, 1.0);
        Vec3 drawPos = length(lab_.pos - prevPos_) > 1.0 ? lab_.pos : lerp(prevPos_, lab_.pos, alpha);
        Vec3 offset = drawPos - lab_.pos;
        camTarget_ = length(drawPos - camTarget_) > 3.0 ? drawPos : lerp(camTarget_, drawPos, std::min(1.0, frameDt_ * 10.0));
        double yaw = deg2rad(camYaw_), pitch = deg2rad(camPitch_);
        Vec3 eye = camTarget_ + Vec3{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)} * camDist_;
        eye.y = std::max(eye.y, lab_.terrain.heightAt(eye.x, eye.z) + 0.05);
        Mat4 vp = Mat4::perspective((float)deg2rad(55.0), (float)w / std::max(h, 1), 0.02f, 400.0f) *
                  Mat4::lookAt(eye, camTarget_, {0, 1, 0});
        Renderer& r = *sh.renderer;
        r.begin(w, h, eye, vp);
        r.drawMesh(terrainMesh_, showGrid_);
        WheelDrawOptions wopt;
        wopt.contact_colours = contactColours_;
        wopt.show_rig = showRig_;
        buildWheelMesh(lab_, offset, wopt, *sh.mb);
        wheelMesh_.upload(sh.mb->v, sh.mb->idx, true);
        r.drawMesh(wheelMesh_, false);
        if (showForces_ && lab_.out.contacts > 0) {
            Vec3 centre;
            double wsum = 0;
            for (size_t e = 0; e < lab_.tyre.load.size(); ++e)
                if (lab_.tyre.load[e] > 0) {
                    centre += lab_.tyre.point[e] * lab_.tyre.load[e];
                    wsum += lab_.tyre.load[e];
                }
            centre = centre / wsum + offset;
            Vec3 f = lab_.out.force / 2000.0;  // 1 m per 2000 N
            Vec3 fh{f.x, 0, f.z};
            std::vector<LineVtx> lines = {
                {(float)centre.x, (float)centre.y, (float)centre.z, 0.2f, 1.0f, 0.3f},
                {(float)(centre.x + f.x), (float)(centre.y + f.y), (float)(centre.z + f.z), 0.2f, 1.0f, 0.3f},
                {(float)centre.x, (float)centre.y + 0.002f, (float)centre.z, 1.0f, 0.3f, 0.9f},
                {(float)(centre.x + fh.x * 4), (float)centre.y + 0.002f, (float)(centre.z + fh.z * 4), 1.0f, 0.3f, 0.9f},
            };
            r.drawLines(lines);
        }
    }

    void ui(Shared& sh, int w, int h) override;

    std::string status() const override { return {}; }

private:
    void resetLab() {
        if (lab_.mode == LabMode::Free)
            lab_.resetFree({0, lab_.params.outer_radius + 0.05, 1.0}, freeSpeed_, freeLean_);
        else
            lab_.resetRig();
        prevPos_ = lab_.pos;
    }
    void rebuildTerrain(Shared& sh) {
        buildTerrainMesh(lab_.terrain, *sh.mb);
        terrainMesh_.upload(sh.mb->v, sh.mb->idx, false);
    }

    WheelLab lab_;
    Mesh terrainMesh_, wheelMesh_;
    double freeSpeed_ = 4.0, freeLean_ = 4.0;
    float camYaw_ = 70, camPitch_ = 10, camDist_ = 2.2f;
    Vec3 camTarget_;
    bool contactColours_ = true, showForces_ = true, showGrid_ = true, showRig_ = true;
    float pressurePsi_ = 13;
    Vec3 prevPos_;
    double accumulator_ = 0, frameDt_ = 1.0 / 60.0;
    bool paused_ = false;
    History hist_;
};

void LabModeImpl::ui(Shared& sh, int w, int h) {
    (void)h;
    Pad& pad = *sh.pad;
    Haptics& haptics = *sh.haptics;
    ImGui::SetNextWindowPos(ImVec2(12, 52), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(430, 0), ImGuiCond_Once);
    ImGui::Begin("Wheel lab", nullptr, ImGuiWindowFlags_NoCollapse);
    ImGui::Text("%.0f FPS   sim %.1f us/tick (1000 ticks/s)", sh.fps, sh.tick_micros);

    int mode = lab_.mode == LabMode::Rig ? 0 : 1;
    bool changed = ImGui::RadioButton("Quarter-bike rig", &mode, 0);
    ImGui::SameLine();
    changed |= ImGui::RadioButton("Free wheel", &mode, 1);
    int scene = lab_.scene() == LabScene::ObstacleLane ? 0 : 1;
    bool sceneChanged = ImGui::RadioButton("Obstacle lane", &scene, 0);
    ImGui::SameLine();
    sceneChanged |= ImGui::RadioButton("Flat", &scene, 1);
    if (sceneChanged) {
        lab_.setScene(scene == 0 ? LabScene::ObstacleLane : LabScene::Flat);
        rebuildTerrain(sh);
        changed = true;
    }
    if (changed) {
        lab_.mode = mode == 0 ? LabMode::Rig : LabMode::Free;
        resetLab();
    }

    ImGui::SeparatorText("Tyre");
    ImGui::SliderFloat("Pressure (psi)", &pressurePsi_, 0.0f, 30.0f, "%.1f");
    ImGui::SameLine();
    ImGui::TextDisabled("%.2f bar", pressurePsi_ * 0.0689476);
    if (pressurePsi_ < 0.5f)
        ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "Flat tyre: only the carcass holds it up");
    else if (pressurePsi_ < 7.0f)
        ImGui::TextDisabled("Hard-enduro low: huge grip on rock, rim strikes come easily");

    if (lab_.mode == LabMode::Rig) {
        ImGui::SeparatorText("Rig");
        float speed = (float)lab_.rig.speed, slip = (float)lab_.rig.slip_deg, camber = (float)lab_.rig.camber_deg;
        float load = (float)lab_.rig.load_kg, drive = (float)lab_.rig.drive_torque, brake = (float)lab_.rig.brake_torque;
        if (ImGui::SliderFloat("Speed (m/s)", &speed, 0.0f, 25.0f, "%.1f")) lab_.rig.speed = speed;
        if (ImGui::SliderFloat("Slip angle (deg)", &slip, -30.0f, 30.0f, "%.1f")) lab_.rig.slip_deg = slip;
        if (ImGui::SliderFloat("Camber / lean (deg)", &camber, -55.0f, 55.0f, "%.1f")) lab_.rig.camber_deg = camber;
        if (ImGui::SliderFloat("Load (kg)", &load, 10.0f, 250.0f, "%.0f")) lab_.rig.load_kg = load;
        if (ImGui::SliderFloat("Drive torque (N*m)", &drive, -400.0f, 1500.0f, "%.0f")) lab_.rig.drive_torque = drive;
        if (ImGui::SliderFloat("Brake torque (N*m)", &brake, 0.0f, 2000.0f, "%.0f")) lab_.rig.brake_torque = brake;
        if (ImGui::Button("Zero torques")) { lab_.rig.drive_torque = 0; lab_.rig.brake_torque = 0; }
        ImGui::SameLine();
        if (ImGui::Button("Lock wheel")) lab_.rig.brake_torque = 5000;
        ImGui::SameLine();
        if (ImGui::Button("Reset (R)")) resetLab();
    } else {
        ImGui::SeparatorText("Free wheel");
        float fs = (float)freeSpeed_, fl = (float)freeLean_;
        if (ImGui::SliderFloat("Launch speed (m/s)", &fs, 0.0f, 20.0f, "%.1f")) freeSpeed_ = fs;
        if (ImGui::SliderFloat("Launch lean (deg)", &fl, -30.0f, 30.0f, "%.1f")) freeLean_ = fl;
        if (ImGui::Button("Launch (R)")) resetLab();
        ImGui::SameLine();
        if (ImGui::Button("Push left")) lab_.applyImpulse(lab_.carrier.rotate({1, 0, 0}) * 12.0, lab_.pos + Vec3{0, 0.25, 0});
        ImGui::SameLine();
        if (ImGui::Button("Push right")) lab_.applyImpulse(lab_.carrier.rotate({-1, 0, 0}) * 12.0, lab_.pos + Vec3{0, 0.25, 0});
        ImGui::SameLine();
        if (ImGui::Button("Hop")) lab_.applyImpulse({0, 45.0, 0}, lab_.pos);
        float drive = (float)lab_.free_drive_torque, brake = (float)lab_.free_brake_torque;
        if (ImGui::SliderFloat("Drive torque (N*m)", &drive, -150.0f, 150.0f, "%.0f")) lab_.free_drive_torque = drive;
        if (ImGui::SliderFloat("Brake torque (N*m)", &brake, 0.0f, 300.0f, "%.0f")) lab_.free_brake_torque = brake;
    }
    ImGui::TextDisabled("Hold W = drive, S = brake");

    ImGui::SeparatorText("Controller");
    if (!pad.connected()) {
        ImGui::TextDisabled("No controller found. Plug one in (USB or Bluetooth) at any time.");
    } else {
        ImGui::Text("%s  %s", pad.name().c_str(), pad.active() ? "(in use)" : "(press any button to use)");
        ImGui::Checkbox("Haptics", &haptics.enabled);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        ImGui::SliderFloat("Strength", &haptics.strength, 0.0f, 2.0f, "%.1f");
        ImGui::ProgressBar(haptics.low, ImVec2(95, 0), "heavy");
        ImGui::SameLine();
        ImGui::ProgressBar(haptics.high, ImVec2(95, 0), "light");
        ImGui::SameLine();
        ImGui::ProgressBar(haptics.lt, ImVec2(70, 0), "LT");
        ImGui::SameLine();
        ImGui::ProgressBar(haptics.rt, ImVec2(70, 0), "RT");
        if (ImGui::TreeNode("Controller layout")) {
            ImGui::TextUnformatted("RT drive   LT brake   Left stick: steer (rig) / push (free)");
            ImGui::TextUnformatted("LB / RB lean (rig)   A hop (free)   B rig <-> free");
            ImGui::TextUnformatted("X obstacle lane <-> flat   Y reset   D-pad L/R pressure");
            ImGui::TextUnformatted("D-pad U/D speed   Right stick camera   L3 zoom   R3 camera reset");
            ImGui::TextUnformatted("Start pause   Back/View hide panels");
            ImGui::TreePop();
        }
    }

    ImGui::SeparatorText("Sim and view");
    ImGui::Checkbox("Pause (Space)", &sh.paused);
    ImGui::SameLine();
    if (ImGui::Button("Step 1 tick (.)") && sh.paused) lab_.step();
    ImGui::SliderFloat("Time scale", &sh.time_scale, 0.02f, 1.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::Checkbox("Contact colours", &contactColours_);
    ImGui::SameLine();
    ImGui::Checkbox("Force", &showForces_);
    ImGui::SameLine();
    ImGui::Checkbox("Grid", &showGrid_);
    ImGui::SameLine();
    ImGui::Checkbox("Rig", &showRig_);
    ImGui::TextDisabled("Right-drag: orbit   Wheel: zoom   F1: hide panel");

    ImGui::SeparatorText("Measurements");
    const TyreOutput& o = lab_.out;
    double speedNow = length(Vec3{lab_.vel.x, 0, lab_.vel.z});
    if (ImGui::BeginTable("m", 2, ImGuiTableFlags_SizingStretchProp)) {
        auto row = [](const char* label, const char* fmt, double v) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(label);
            ImGui::TableNextColumn();
            ImGui::Text(fmt, v);
        };
        if (lab_.mode == LabMode::Rig) {
            row("Longitudinal force Fx", "%.0f N", lab_.measure.fx);
            row("Lateral force Fy", "%.0f N", lab_.measure.fy);
            row("Vertical force Fz", "%.0f N", lab_.measure.fz);
            row("Horizontal / vertical force", "%.2f", std::hypot(lab_.measure.fx, lab_.measure.fy) / std::max(lab_.measure.fz, 1.0));
            row("Aligning moment Mz", "%.1f N*m", lab_.measure.mz);
            row("Slip ratio", "%.3f", lab_.measure.slip_ratio);
            row("Shock travel used", "%.0f mm", (lab_.rig.travel - (lab_.sprung_y - lab_.pos.y)) * 1000.0);
        } else {
            row("Ground force (N)", "%.0f", length(o.force));
            row("Lean (deg)", "%.1f", rad2deg(std::asin(std::clamp(lab_.carrier.rotate({1, 0, 0}).y, -1.0, 1.0))));
        }
        row("Speed", "%.2f m/s", speedNow);
        row("Wheel speed", "%.0f rpm", lab_.spin * 60.0 / (2.0 * kPi));
        row("Tyre deflection", "%.1f mm", o.max_compression * 1000.0);
        row("Contact patch", "%.0f cm^2", o.patch_area * 1e4);
        row("Elements in contact", "%.0f", (double)o.contacts);
        row("Elements sliding", "%.0f", (double)o.sliding);
        ImGui::EndTable();
    }
    if (o.rim_strike) ImGui::TextColored(ImVec4(1, 0.35f, 0.25f, 1), "RIM STRIKE");
    ImGui::PlotLines("Fx", hist_.fx, History::N, hist_.head, nullptr, -2500, 2500, ImVec2(0, 40));
    ImGui::PlotLines("Fy", hist_.fy, History::N, hist_.head, nullptr, -2500, 2500, ImVec2(0, 40));
    ImGui::PlotLines("Fz", hist_.fz, History::N, hist_.head, nullptr, 0, 6000, ImVec2(0, 40));
    ImGui::TextDisabled("Tyre colours: yellow -> red = contact pressure, cyan = sliding");
    ImGui::End();

    // Footprint: the tyre unrolled round the bottom (front at the top), one cell per element.
    ImGui::SetNextWindowPos(ImVec2((float)w - 200, 52), ImGuiCond_Once);
    ImGui::Begin("Footprint", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::TextDisabled("front at top, +-112 deg");
    const int nc = lab_.params.sections, nw = lab_.params.profile_points;
    const int half = 40;
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
            float ld = lab_.tyre.load[e];
            if (ld <= 0) continue;
            ImU32 col;
            if (lab_.tyre.sliding[e]) {
                col = IM_COL32(40, 190, 240, 255);
            } else {
                float t = std::min(ld / (float)(2.0 * lab_.params.pressure), 1.0f);
                col = IM_COL32(250, (int)(215 - 165 * t), (int)(40 - 15 * t), 255);
            }
            dl->AddRectFilled(ImVec2(x0 + j * cw + 1, y), ImVec2(x0 + (j + 1) * cw - 1, y + ch), col);
        }
    }
    dl->AddLine(ImVec2(x0 - 6, org.y + half * ch + ch * 0.5f), ImVec2(x0 - 1, org.y + half * ch + ch * 0.5f), IM_COL32(200, 200, 200, 255));
    ImGui::Dummy(ImVec2(cw * nw + 12, ch * (2 * half + 1) + 4));
    ImGui::End();
}

}  // namespace

std::unique_ptr<GameMode> makeLabMode(const LabOptions& opt, Shared& sh) { return std::make_unique<LabModeImpl>(opt, sh); }
