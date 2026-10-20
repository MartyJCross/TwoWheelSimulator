// Game modes: Ride (the bike on the test area) and Wheel lab (one wheel, M0). main.cpp owns the window, renderer,
// controller and loop; each mode owns its sim, camera, input mapping, drawing and panels.
#pragma once
#include <SDL3/SDL.h>

#include <memory>
#include <string>

#include "pad.h"
#include "render.h"

struct Shared {
    Renderer* renderer = nullptr;
    Pad* pad = nullptr;
    Haptics* haptics = nullptr;
    MeshBuilder* mb = nullptr;
    bool paused = false;
    bool show_ui = true;
    float time_scale = 1.0f;
    double fps = 0;
    double tick_micros = 0;  // sim cost per 1 ms tick (measured)
    bool screenshot = false;  // fixed-step screenshot run: modes may apply scripted demo input
};

class GameMode {
public:
    virtual ~GameMode() = default;
    virtual const char* name() const = 0;
    // Every SDL event (after ImGui has seen it).
    virtual void event(const SDL_Event& ev, Shared& sh) { (void)ev; (void)sh; }
    // Read input, step the sim at 1 kHz for `frameDt` of time, update haptics. `frameDt` is already clamped.
    virtual void update(double frameDt, Shared& sh) = 0;
    // 3D scene, then ImGui panels (inside an ImGui frame).
    virtual void render(Shared& sh, int w, int h) = 0;
    virtual void ui(Shared& sh, int w, int h) = 0;
    // Text for the screenshot log.
    virtual std::string status() const { return {}; }
};

struct LabOptions {
    bool free_mode = false;
    bool flat = false;
    double speed = 4.0;
    double drive = 0;
    float cam_yaw = 70, cam_pitch = 10, cam_dist = 2.2f;
};
std::unique_ptr<GameMode> makeLabMode(const LabOptions& opt, Shared& sh);

struct RideOptions {
    double demo_throttle = -1;  // screenshot runs: hold this throttle (and roll off at the start) if >= 0
    double demo_steer = 0;
    double demo_speed = 0;      // start rolling at this speed
    int camera = 0;
};
std::unique_ptr<GameMode> makeRideMode(const RideOptions& opt, Shared& sh);
