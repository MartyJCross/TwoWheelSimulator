// Ride (M1): the bike on the test area. Controls follow docs/04_CONTROLS.md (MX and hard-enduro profiles).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "bike.h"
#include "imgui.h"
#include "modes.h"
#include "test_area.h"

using namespace tws;

namespace {

// Move `v` toward `target` at `up` per second when rising and `down` per second when falling.
double ramp(double v, double target, double up, double down, double dt) {
    if (target > v) return std::min(target, v + up * dt);
    return std::max(target, v - down * dt);
}

const char* kCameraNames[4] = {"Chase", "Close", "Side", "Orbit"};

class RideModeImpl : public GameMode {
public:
    RideModeImpl(const RideOptions& opt, Shared& sh) : opt_(opt) {
        info_ = buildTestArea(terrain_);
        buildTerrainMesh(terrain_, *sh.mb);
        terrainMesh_.upload(sh.mb->v, sh.mb->idx, false);
        camera_ = opt.camera;
        respawn();
        if (opt.demo_speed > 0) bike_.setRolling(opt.demo_speed);
        prevBase_ = bike_.position();
        camTarget_ = bike_.position();
        camHeading_ = headingOf(bike_);
    }

    const char* name() const override { return "Ride"; }

    void event(const SDL_Event& ev, Shared& sh) override {
        ImGuiIO& io = ImGui::GetIO();
        if (ev.type == SDL_EVENT_KEY_DOWN && !io.WantCaptureKeyboard && !ev.key.repeat) {
            switch (ev.key.key) {
                case SDLK_E: shiftUp_ = true; break;
                case SDLK_Q: shiftDown_ = true; break;
                case SDLK_RETURN: startEngine_ = true; break;
                case SDLK_R: respawn(); sh.haptics->reset(); break;
                case SDLK_T: pickUp(); sh.haptics->reset(); break;
                case SDLK_C: cycleCamera(); break;
                case SDLK_P: sh.paused = !sh.paused; break;
                default: break;
            }
        }
        if (ev.type == SDL_EVENT_MOUSE_MOTION && (ev.motion.state & SDL_BUTTON_RMASK) && !io.WantCaptureMouse) {
            orbitYaw_ -= ev.motion.xrel * 0.3f;
            orbitPitch_ = std::clamp(orbitPitch_ + ev.motion.yrel * 0.3f, -20.0f, 80.0f);
        }
        if (ev.type == SDL_EVENT_MOUSE_WHEEL && !io.WantCaptureMouse)
            zoom_ = std::clamp(zoom_ * (ev.wheel.y > 0 ? 0.9f : 1.1f), 0.4f, 4.0f);
    }

    void update(double dt, Shared& sh) override {
        Pad& pad = *sh.pad;
        BikeInput in;
        readKeyboard(dt, in);
        if (pad.connected()) readPad(dt, pad, in, sh);
        in.shift_up = shiftUp_;
        in.shift_down = shiftDown_;
        in.start_engine = startEngine_;
        shiftUp_ = shiftDown_ = startEngine_ = false;
        if (opt_.demo_throttle >= 0) {
            in.throttle = opt_.demo_throttle;
            in.steer = opt_.demo_steer;
        }
        lastInput_ = in;

        if (!sh.paused) {
            accumulator_ += dt * sh.time_scale;
            int ticks = 0;
            uint64_t t0 = SDL_GetPerformanceCounter();
            while (accumulator_ >= 0.001 && ticks < 200) {
                prevBase_ = bike_.position();
                bike_.step(terrain_, in, 0.001);
                in.shift_up = in.shift_down = in.start_engine = false;
                accumulator_ -= 0.001;
                ++ticks;
            }
            if (ticks)
                sh.tick_micros = sh.tick_micros * 0.9 + 0.1 * ((SDL_GetPerformanceCounter() - t0) / (double)SDL_GetPerformanceFrequency() * 1e6 / ticks);
            if (accumulator_ > 0.001) accumulator_ = 0;
        }
        if (bike_.position().y < -20.0) respawn();

        // Haptics.
        HapticSignal s;
        s.load = bike_.front_out.normal_load + bike_.rear_out.normal_load;
        s.rest_load = 187.0 * kGravity;
        s.rim_strike = bike_.front_out.rim_strike || bike_.rear_out.rim_strike;
        s.slide_front = bike_.front_out.contacts ? (double)bike_.front_out.sliding / bike_.front_out.contacts : 0.0;
        s.slide_rear = bike_.rear_out.contacts ? (double)bike_.rear_out.sliding / bike_.rear_out.contacts : 0.0;
        s.speed = std::fabs(bike_.tele.speed);
        s.in_contact = bike_.tele.front_contact || bike_.tele.rear_contact;
        s.rpm = bike_.tele.engine_running ? bike_.tele.rpm : 0.0;
        s.driving = in.throttle > 0.05;
        s.braking = in.front_brake > 0.05;
        sh.haptics->update(s, sh.paused ? 0.0 : dt);
        frameDt_ = dt;
        paused_ = sh.paused;
    }

    void render(Shared& sh, int w, int h) override {
        // Interpolate the whole bike between the last two ticks by shifting it.
        double alpha = paused_ ? 1.0 : std::clamp(accumulator_ / 0.001, 0.0, 1.0);
        Vec3 cur = bike_.position();
        Vec3 drawPos = length(cur - prevBase_) > 0.5 ? cur : lerp(prevBase_, cur, alpha);
        Vec3 offset = drawPos - cur;

        // Camera.
        const double dt = std::min(frameDt_, 0.05);
        Vec3 target = bike_.mb.centreOfMass() + offset + Vec3{0, 0.35, 0};
        camTarget_ = length(target - camTarget_) > 5.0 ? target : lerp(camTarget_, target, std::min(1.0, dt * 12.0));
        double want = headingOf(bike_);
        double diff = std::remainder(want - camHeading_, 2 * kPi);
        camHeading_ += diff * std::min(1.0, dt * 3.0);
        Vec3 eye;
        double yaw = camHeading_ + deg2rad(orbitYaw_);
        double pitch = deg2rad(orbitPitch_);
        if (camera_ == 0 || camera_ == 1) {
            double dist = (camera_ == 0 ? 4.2 : 2.6) * zoom_;
            double basePitch = camera_ == 0 ? 0.24 : 0.18;
            double p = basePitch + pitch;
            eye = camTarget_ + Vec3{-std::sin(yaw) * std::cos(p), std::sin(p), -std::cos(yaw) * std::cos(p)} * dist;
        } else if (camera_ == 2) {
            double y2 = yaw + kPi / 2;
            eye = camTarget_ + Vec3{std::sin(y2), 0.25 + std::sin(pitch), std::cos(y2)} * (3.2 * zoom_);
        } else {
            eye = camTarget_ + Vec3{-std::sin(yaw) * std::cos(pitch + 0.3), std::sin(pitch + 0.3), -std::cos(yaw) * std::cos(pitch + 0.3)} * (6.0 * zoom_);
        }
        eye.y = std::max(eye.y, terrain_.heightAt(eye.x, eye.z) + 0.3);
        Mat4 vp = Mat4::perspective((float)deg2rad(60.0), (float)w / std::max(h, 1), 0.05f, 800.0f) *
                  Mat4::lookAt(eye, camTarget_, {0, 1, 0});
        Renderer& r = *sh.renderer;
        r.begin(w, h, eye, vp);
        r.drawMesh(terrainMesh_, true);
        sh.mb->clear();
        buildBikeMesh(bike_, offset, contactColours_, *sh.mb);
        bikeMesh_.upload(sh.mb->v, sh.mb->idx, true);
        r.drawMesh(bikeMesh_, false);
    }

    void ui(Shared& sh, int w, int h) override;

    std::string status() const override {
        char buf[200];
        std::snprintf(buf, sizeof buf, "ride: speed %.1f m/s, gear %d, %.0f rpm, lean %.1f, pitch %.1f%s", bike_.tele.speed,
                      bike_.tele.gear, bike_.tele.rpm, bike_.tele.lean_deg, bike_.tele.pitch_deg,
                      bike_.tele.crashed ? ", CRASHED" : "");
        return buf;
    }

private:
    static double headingOf(const Bike& b) {
        Vec3 f = b.rotation().rotate({0, 0, 1});
        return std::atan2(f.x, f.z);
    }

    void respawn() {
        bike_.reset(terrain_, info_.spawn.x, info_.spawn.z, info_.spawn_heading_deg);
        prevBase_ = bike_.position();
        accumulator_ = 0;
    }
    void pickUp() {
        Vec3 p = bike_.position();
        double heading = rad2deg(headingOf(bike_));
        bike_.reset(terrain_, p.x, p.z, heading);
        prevBase_ = bike_.position();
        accumulator_ = 0;
    }
    void cycleCamera() {
        camera_ = (camera_ + 1) % 4;
        orbitYaw_ = 0;
        orbitPitch_ = 0;
        zoom_ = 1.0f;
    }

    void readKeyboard(double dt, BikeInput& in) {
        const bool* k = SDL_GetKeyboardState(nullptr);
        if (ImGui::GetIO().WantCaptureKeyboard) {
            kThrottle_ = kFront_ = kRear_ = kClutch_ = kSteer_ = 0;
            return;
        }
        kThrottle_ = ramp(kThrottle_, k[SDL_SCANCODE_W] ? 1.0 : 0.0, 4.0, 8.0, dt);
        kFront_ = ramp(kFront_, k[SDL_SCANCODE_S] ? 1.0 : 0.0, 2.5, 8.0, dt);
        kRear_ = ramp(kRear_, k[SDL_SCANCODE_SPACE] ? 1.0 : 0.0, 3.0, 8.0, dt);
        kClutch_ = ramp(kClutch_, k[SDL_SCANCODE_LSHIFT] ? 1.0 : 0.0, 8.0, 3.0, dt);
        double steerTarget = (k[SDL_SCANCODE_D] ? 1.0 : 0.0) - (k[SDL_SCANCODE_A] ? 1.0 : 0.0);
        const double steerRate = (steerTarget == 0.0 ? 4.0 : 2.5) * dt;  // centres faster than it winds on
        kSteer_ += std::clamp(steerTarget - kSteer_, -steerRate, steerRate);
        kFore_ = ramp(kFore_, (k[SDL_SCANCODE_UP] ? 1.0 : 0.0) - (k[SDL_SCANCODE_DOWN] ? 1.0 : 0.0), 4.0, 4.0, dt);
        kSide_ = ramp(kSide_, (k[SDL_SCANCODE_RIGHT] ? 1.0 : 0.0) - (k[SDL_SCANCODE_LEFT] ? 1.0 : 0.0), 4.0, 4.0, dt);
        kHeight_ = ramp(kHeight_, k[SDL_SCANCODE_LCTRL] ? -1.0 : 0.0, 5.0, 5.0, dt);
        in.throttle = kThrottle_;
        in.front_brake = kFront_;
        in.rear_brake = kRear_;
        in.clutch = kClutch_;
        in.steer = kSteer_;
        in.body_fore = kFore_;
        in.body_side = kSide_;
        in.body_height = kHeight_;
        in.dab_left = k[SDL_SCANCODE_Z];
        in.dab_right = k[SDL_SCANCODE_X];
    }

    void readPad(double dt, Pad& pad, BikeInput& in, Shared& sh) {
        // Ramped buttons for the inputs that sit on bumpers.
        const bool lb = pad.held(SDL_GAMEPAD_BUTTON_LEFT_SHOULDER), rb = pad.held(SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER);
        pRear_ = ramp(pRear_, lb ? 1.0 : 0.0, 3.0, 8.0, dt);
        pBumperR_ = ramp(pBumperR_, rb ? 1.0 : 0.0, enduroProfile_ ? 2.5 : 8.0, enduroProfile_ ? 8.0 : 3.0, dt);
        in.throttle = std::max(in.throttle, (double)pad.rightTrigger());
        in.rear_brake = std::max(in.rear_brake, pRear_);
        if (enduroProfile_) {
            in.clutch = std::max(in.clutch, (double)pad.leftTrigger());
            in.front_brake = std::max(in.front_brake, pBumperR_);
        } else {
            in.front_brake = std::max(in.front_brake, (double)pad.leftTrigger());
            in.clutch = std::max(in.clutch, pBumperR_);
        }
        if (pad.leftX() != 0.0f) in.steer = pad.leftX();
        if (pad.leftY() != 0.0f) in.body_height = -pad.leftY();
        if (pad.rightX() != 0.0f) in.body_side = pad.rightX();
        if (pad.rightY() != 0.0f) in.body_fore = -pad.rightY();
        in.dab_left = in.dab_left || pad.held(SDL_GAMEPAD_BUTTON_DPAD_LEFT);
        in.dab_right = in.dab_right || pad.held(SDL_GAMEPAD_BUTTON_DPAD_RIGHT);
        if (pad.pressed(SDL_GAMEPAD_BUTTON_SOUTH)) shiftUp_ = true;
        if (pad.pressed(SDL_GAMEPAD_BUTTON_WEST)) shiftDown_ = true;
        if (pad.pressed(SDL_GAMEPAD_BUTTON_NORTH)) startEngine_ = true;
        if (pad.pressed(SDL_GAMEPAD_BUTTON_EAST)) { pickUp(); sh.haptics->reset(); }
        if (pad.pressed(SDL_GAMEPAD_BUTTON_BACK)) { respawn(); sh.haptics->reset(); }
        if (pad.pressed(SDL_GAMEPAD_BUTTON_START)) sh.paused = !sh.paused;
        if (pad.pressed(SDL_GAMEPAD_BUTTON_RIGHT_STICK)) cycleCamera();
        if (pad.pressed(SDL_GAMEPAD_BUTTON_DPAD_UP)) sh.show_ui = !sh.show_ui;
    }

    RideOptions opt_;
    Terrain terrain_;
    TestAreaInfo info_;
    Bike bike_;
    Mesh terrainMesh_, bikeMesh_;
    double accumulator_ = 0, frameDt_ = 1.0 / 60.0;
    bool paused_ = false;
    Vec3 prevBase_, camTarget_;
    double camHeading_ = 0;
    int camera_ = 0;
    float orbitYaw_ = 0, orbitPitch_ = 0, zoom_ = 1.0f;
    bool contactColours_ = false;
    bool enduroProfile_ = false;
    bool shiftUp_ = false, shiftDown_ = false, startEngine_ = false;
    double kThrottle_ = 0, kFront_ = 0, kRear_ = 0, kClutch_ = 0, kSteer_ = 0, kFore_ = 0, kSide_ = 0, kHeight_ = 0;
    double pRear_ = 0, pBumperR_ = 0;
    BikeInput lastInput_;
};

void RideModeImpl::ui(Shared& sh, int w, int h) {
    const Telemetry& t = bike_.tele;
    // --- HUD: always on. ---
    ImGui::SetNextWindowPos(ImVec2(16, (float)h - 16), ImGuiCond_Always, ImVec2(0, 1));
    ImGui::SetNextWindowBgAlpha(0.45f);
    ImGui::Begin("##hud", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                       ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoFocusOnAppearing);
    ImGui::SetWindowFontScale(2.4f);
    ImGui::Text("%3.0f km/h   %s", std::fabs(t.speed) * 3.6, t.gear == 0 ? "N" : std::to_string(t.gear).c_str());
    ImGui::SetWindowFontScale(1.0f);
    char rpmText[32];
    std::snprintf(rpmText, sizeof rpmText, "%.0f rpm", t.rpm);
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, t.rpm > 9000 ? ImVec4(0.95f, 0.25f, 0.2f, 1) : ImVec4(0.95f, 0.55f, 0.1f, 1));
    ImGui::ProgressBar((float)std::clamp(t.rpm / 10000.0, 0.0, 1.0), ImVec2(260, 0), rpmText);
    ImGui::PopStyleColor();
    ImGui::ProgressBar((float)t.clutch_engaged, ImVec2(260, 0), t.clutch_slipping ? "clutch slipping" : "clutch");
    ImGui::Text("lean %5.1f   pitch %5.1f", t.lean_deg, t.pitch_deg);
    if (!t.engine_running) ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "Stalled: Y / Enter to start");
    ImGui::End();
    if (t.crashed) {
        ImGui::SetNextWindowPos(ImVec2(w * 0.5f, h * 0.3f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowBgAlpha(0.6f);
        ImGui::Begin("##crash", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs);
        ImGui::SetWindowFontScale(1.8f);
        ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "Crashed");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::TextUnformatted("B / T: pick the bike up here    View / R: back to the start");
        ImGui::End();
    }
    if (sh.paused) {
        ImGui::SetNextWindowPos(ImVec2(w * 0.5f, 60), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
        ImGui::Begin("##paused", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs);
        ImGui::TextUnformatted("PAUSED (Start / P)");
        ImGui::End();
    }
    if (!sh.show_ui) return;

    // --- Panel. ---
    Pad& pad = *sh.pad;
    Haptics& hap = *sh.haptics;
    ImGui::SetNextWindowPos(ImVec2(12, 52), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_Once);
    ImGui::Begin("Ride", nullptr, ImGuiWindowFlags_NoCollapse);
    ImGui::Text("%.0f FPS   sim %.0f us/tick (1000 ticks/s)", sh.fps, sh.tick_micros);
    if (ImGui::Button("Back to start (R)")) respawn();
    ImGui::SameLine();
    if (ImGui::Button("Pick bike up (T)")) pickUp();
    ImGui::SameLine();
    if (ImGui::Button(kCameraNames[camera_])) cycleCamera();
    ImGui::SameLine();
    ImGui::TextDisabled("camera (C)");

    ImGui::SeparatorText("Assists");
    ImGui::Checkbox("Lean intent", &bike_.assists.lean_intent);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("On: the stick sets the lean you want and the rider countersteers for you.\nOff: the stick is raw torque on the bars (full sim).");
    ImGui::SameLine();
    ImGui::Checkbox("Auto-clutch", &bike_.assists.auto_clutch);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Engages and slips the clutch by engine speed, like a real auto-clutch.\nThe lever still works on top.");
    ImGui::SameLine();
    ImGui::Checkbox("Auto-dab", &bike_.assists.auto_dab);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Feet go down below walking pace so you don't tip over when stopped.");

    ImGui::SeparatorText("Tyres");
    float fp = (float)(bike_.front_tyre.pressure / 6894.76), rp = (float)(bike_.rear_tyre.pressure / 6894.76);
    if (ImGui::SliderFloat("Front (psi)", &fp, 0.0f, 30.0f, "%.1f")) bike_.front_tyre.pressure = fp * 6894.76;
    if (ImGui::SliderFloat("Rear (psi)", &rp, 0.0f, 30.0f, "%.1f")) bike_.rear_tyre.pressure = rp * 6894.76;
    ImGui::Checkbox("Contact colours on the tyres", &contactColours_);

    ImGui::SeparatorText("Controller");
    if (!pad.connected()) {
        ImGui::TextDisabled("No controller found. Plug one in at any time.");
    } else {
        ImGui::Text("%s  %s", pad.name().c_str(), pad.active() ? "(in use)" : "(press any button to use)");
        int prof = enduroProfile_ ? 1 : 0;
        if (ImGui::RadioButton("MX profile", &prof, 0)) enduroProfile_ = false;
        ImGui::SameLine();
        if (ImGui::RadioButton("Hard enduro profile", &prof, 1)) enduroProfile_ = true;
        ImGui::TextDisabled(enduroProfile_ ? "RT throttle  LT clutch  RB front brake  LB rear brake"
                                           : "RT throttle  LT front brake  LB rear brake  RB clutch");
        ImGui::Checkbox("Haptics", &hap.enabled);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(130);
        ImGui::SliderFloat("Strength", &hap.strength, 0.0f, 2.0f, "%.1f");
        ImGui::ProgressBar(hap.low, ImVec2(88, 0), "heavy");
        ImGui::SameLine();
        ImGui::ProgressBar(hap.high, ImVec2(88, 0), "light");
        ImGui::SameLine();
        ImGui::ProgressBar(hap.lt, ImVec2(70, 0), "LT");
        ImGui::SameLine();
        ImGui::ProgressBar(hap.rt, ImVec2(70, 0), "RT");
    }
    if (ImGui::TreeNode("Controls")) {
        ImGui::TextUnformatted("Controller: left stick steer (+ up/down = stand tall/crouch)");
        ImGui::TextUnformatted("  right stick = body forward/back and side to side");
        ImGui::TextUnformatted("  A shift up   X shift down   Y start engine   B pick up bike");
        ImGui::TextUnformatted("  D-pad left/right dab a foot   View back to start   Start pause");
        ImGui::TextUnformatted("  R3 camera   D-pad up hide panel");
        ImGui::TextUnformatted("Keyboard: W throttle  S front brake  Space rear brake  Shift clutch");
        ImGui::TextUnformatted("  A/D steer  arrows body  Ctrl crouch  E/Q gear up/down  Enter start");
        ImGui::TextUnformatted("  Z/X dab  T pick up  R back to start  C camera  P pause  F1 panel");
        ImGui::TextUnformatted("Mouse: right-drag to look around, wheel to zoom");
        ImGui::TreePop();
    }

    ImGui::SeparatorText("Telemetry");
    if (ImGui::BeginTable("tele", 4, ImGuiTableFlags_SizingStretchProp)) {
        auto cell = [](const char* label, const char* fmt, double v) {
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", label);
            ImGui::TableNextColumn();
            ImGui::Text(fmt, v);
        };
        cell("Speed", "%.1f km/h", t.speed * 3.6);
        cell("Engine", "%.0f rpm", t.rpm);
        cell("Throttle", "%.0f %%", lastInput_.throttle * 100);
        cell("Clutch", "%.0f %%", t.clutch_engaged * 100);
        cell("Front brake", "%.0f %%", lastInput_.front_brake * 100);
        cell("Rear brake", "%.0f %%", lastInput_.rear_brake * 100);
        cell("Lean", "%.1f deg", t.lean_deg);
        cell("Steer", "%.1f deg", t.steer_deg);
        cell("Fork travel", "%.0f mm", t.fork_travel * 1000);
        cell("Shock travel", "%.0f mm", t.shock_travel * 1000);
        cell("Front load", "%.0f N", t.front_load);
        cell("Rear load", "%.0f N", t.rear_load);
        double fs = bike_.front_out.contacts ? 100.0 * bike_.front_out.sliding / bike_.front_out.contacts : 0.0;
        double rs = bike_.rear_out.contacts ? 100.0 * bike_.rear_out.sliding / bike_.rear_out.contacts : 0.0;
        cell("Front sliding", "%.0f %%", fs);
        cell("Rear sliding", "%.0f %%", rs);
        ImGui::EndTable();
    }
    if (bike_.front_out.rim_strike || bike_.rear_out.rim_strike) ImGui::TextColored(ImVec4(1, 0.35f, 0.25f, 1), "RIM STRIKE");
    ImGui::End();
}

}  // namespace

std::unique_ptr<GameMode> makeRideMode(const RideOptions& opt, Shared& sh) { return std::make_unique<RideModeImpl>(opt, sh); }
