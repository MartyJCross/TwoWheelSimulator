// Two Wheel Simulator: window, renderer, controller and the main loop. The game modes (Ride, Wheel lab) live in
// ride_mode.cpp and lab_mode.cpp.
//
// Command line (all optional):
//   --lab                      start in the wheel lab (default: Ride)
//   Wheel lab:  --mode rig|free  --scene lane|flat  --speed <m/s>  --drive <N*m>  --cam <yaw>,<pitch>,<dist>
//   Ride:       --ride-speed <m/s>  --ride-throttle <0..1>  --ride-steer <-1..1>  --ride-cam <0..3>
//   --size <w>x<h>   --position <x>,<y>   --no-ui
//   --screenshot <file.png> [--frames <n>]   renders n frames at a fixed 1/60 s each, saves, and exits
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "gl.h"
#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl3.h"
#include "modes.h"
#include "pad.h"
#include "png.h"
#include "render.h"

namespace {

struct Options {
    bool lab = false;
    LabOptions labOpt;
    RideOptions rideOpt;
    int width = 1600, height = 900;
    bool hasPos = false;
    int posX = 0, posY = 0;
    std::string screenshot;
    int frames = 120;
    bool ui = true;
};

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--lab") o.lab = true;
        else if (a == "--mode") { o.lab = true; o.labOpt.free_mode = std::strcmp(next(), "free") == 0; }
        else if (a == "--scene") { o.lab = true; o.labOpt.flat = std::strcmp(next(), "flat") == 0; }
        else if (a == "--speed") o.labOpt.speed = std::atof(next());
        else if (a == "--drive") o.labOpt.drive = std::atof(next());
        else if (a == "--cam") std::sscanf(next(), "%f,%f,%f", &o.labOpt.cam_yaw, &o.labOpt.cam_pitch, &o.labOpt.cam_dist);
        else if (a == "--ride-speed") o.rideOpt.demo_speed = std::atof(next());
        else if (a == "--ride-throttle") o.rideOpt.demo_throttle = std::atof(next());
        else if (a == "--ride-steer") o.rideOpt.demo_steer = std::atof(next());
        else if (a == "--ride-cam") o.rideOpt.camera = std::atoi(next());
        else if (a == "--size") std::sscanf(next(), "%dx%d", &o.width, &o.height);
        else if (a == "--position") o.hasPos = std::sscanf(next(), "%d,%d", &o.posX, &o.posY) == 2;
        else if (a == "--screenshot") o.screenshot = next();
        else if (a == "--frames") o.frames = std::atoi(next());
        else if (a == "--no-ui") o.ui = false;
    }
    return o;
}

void* getProc(const char* name) { return reinterpret_cast<void*>(SDL_GL_GetProcAddress(name)); }

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

    std::string title = std::string("Two Wheel Simulator v") + TWS_VERSION;
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

    Pad pad;
    pad.openFirstAvailable();
    Haptics haptics;
    MeshBuilder mb;
    Shared sh;
    sh.renderer = &renderer;
    sh.pad = &pad;
    sh.haptics = &haptics;
    sh.mb = &mb;
    sh.show_ui = opt.ui;
    sh.screenshot = !opt.screenshot.empty();

    std::unique_ptr<GameMode> mode = opt.lab ? makeLabMode(opt.labOpt, sh) : makeRideMode(opt.rideOpt, sh);
    int wantMode = -1;  // switch requested from the UI: 0 = ride, 1 = lab

    uint64_t lastCounter = SDL_GetPerformanceCounter();
    const double freq = (double)SDL_GetPerformanceFrequency();
    double fpsTimer = 0;
    int fpsFrames = 0, frame = 0;
    bool running = true;

    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            ImGui_ImplSDL3_ProcessEvent(&ev);
            if (ev.type == SDL_EVENT_QUIT) running = false;
            pad.handleEvent(ev);
            if (ev.type == SDL_EVENT_KEY_DOWN && !ImGui::GetIO().WantCaptureKeyboard && !ev.key.repeat) {
                if (ev.key.key == SDLK_ESCAPE) running = false;
                if (ev.key.key == SDLK_F1) sh.show_ui = !sh.show_ui;
                if (ev.key.key == SDLK_F2) wantMode = std::strcmp(mode->name(), "Ride") == 0 ? 1 : 0;
            }
            mode->event(ev, sh);
        }
        pad.update();

        uint64_t now = SDL_GetPerformanceCounter();
        double frameDt = (now - lastCounter) / freq;
        lastCounter = now;
        if (sh.screenshot) frameDt = 1.0 / 60.0;  // deterministic screenshots
        fpsTimer += frameDt;
        ++fpsFrames;
        if (fpsTimer >= 0.5) {
            sh.fps = fpsFrames / fpsTimer;
            fpsTimer = 0;
            fpsFrames = 0;
        }

        mode->update(std::min(frameDt, 0.1), sh);
        if (sh.paused) pad.rumble(0, 0, 0, 0);
        else pad.rumble(haptics.low, haptics.high, haptics.lt, haptics.rt);

        int w = 0, h = 0;
        SDL_GetWindowSizeInPixels(window, &w, &h);
        mode->render(sh, w, h);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        if (sh.show_ui) {
            // Mode bar.
            ImGui::SetNextWindowPos(ImVec2(w * 0.5f, 8), ImGuiCond_Always, ImVec2(0.5f, 0));
            ImGui::SetNextWindowBgAlpha(0.6f);
            ImGui::Begin("##modes", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize);
            bool ride = std::strcmp(mode->name(), "Ride") == 0;
            if (ImGui::RadioButton("Ride", ride) && !ride) wantMode = 0;
            ImGui::SameLine();
            if (ImGui::RadioButton("Wheel lab", !ride) && ride) wantMode = 1;
            ImGui::SameLine();
            ImGui::TextDisabled("(F2)   v%s", TWS_VERSION);
            ImGui::End();
        }
        mode->ui(sh, w, h);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        ++frame;
        if (sh.screenshot && frame >= opt.frames) {
            std::vector<uint8_t> px((size_t)w * h * 3), flipped((size_t)w * h * 3);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            for (int y = 0; y < h; ++y)
                std::memcpy(&flipped[(size_t)y * w * 3], &px[(size_t)(h - 1 - y) * w * 3], (size_t)w * 3);
            bool ok = writePng(opt.screenshot.c_str(), w, h, flipped.data());
            std::printf("%s %s (%dx%d), sim %.1f us/tick, renderer: %s\n", ok ? "Saved" : "FAILED to save",
                        opt.screenshot.c_str(), w, h, sh.tick_micros, (const char*)glGetString(GL_RENDERER));
            std::printf("controller: %s, haptics heavy %.2f light %.2f LT %.2f RT %.2f\n",
                        pad.connected() ? pad.name().c_str() : "none", haptics.low, haptics.high, haptics.lt, haptics.rt);
            std::string st = mode->status();
            if (!st.empty()) std::printf("%s\n", st.c_str());
            running = false;
        }
        SDL_GL_SwapWindow(window);

        if (wantMode >= 0) {
            pad.stopRumble();
            haptics.reset();
            sh.paused = false;
            sh.tick_micros = 0;
            mode.reset();
            mode = wantMode == 1 ? makeLabMode(LabOptions{}, sh) : makeRideMode(RideOptions{}, sh);
            wantMode = -1;
        }
    }

    pad.stopRumble();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DestroyContext(ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
