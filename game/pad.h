// Gamepad input and haptics (SDL3). Same approach as The Ledger: the game switches between keyboard/mouse and the
// controller by whichever was used last, and rumble only runs while the controller is the active device.
// Rumble carries sim data only (docs/04_CONTROLS.md, "Rumble and gyro"); it never decides anything.
#pragma once
#include <SDL3/SDL.h>

#include <string>

namespace tws {
class WheelLab;
}

class Pad {
public:
    // Feed every SDL event. Opens a controller when one is plugged in (or already present) and tracks which device
    // the player used last.
    void handleEvent(const SDL_Event& ev);
    void openFirstAvailable();

    bool connected() const { return pad_ != nullptr; }
    bool active() const { return connected() && usingPad_; }  // the controller is the device in use
    const std::string& name() const { return name_; }
    bool hasRumble() const { return rumble_; }
    bool hasTriggerRumble() const { return triggerRumble_; }

    // Sticks: -1..1 with a radial deadzone. Triggers: 0..1.
    float leftX() const { return stick_[0]; }
    float leftY() const { return stick_[1]; }
    float rightX() const { return stick_[2]; }
    float rightY() const { return stick_[3]; }
    float leftTrigger() const { return trig_[0]; }
    float rightTrigger() const { return trig_[1]; }
    bool held(SDL_GamepadButton b) const;
    bool pressed(SDL_GamepadButton b) const;  // went down this frame

    // Call once per frame before reading input.
    void update();
    // Motors 0..1 (low = heavy, left; high = light, right; triggers = Xbox impulse triggers where supported).
    void rumble(float low, float high, float leftTrig, float rightTrig);
    void stopRumble();

private:
    SDL_Gamepad* pad_ = nullptr;
    SDL_JoystickID id_ = 0;
    std::string name_;
    bool usingPad_ = false;
    bool rumble_ = false, triggerRumble_ = false;
    float stick_[4] = {}, trig_[2] = {};
    bool now_[SDL_GAMEPAD_BUTTON_COUNT] = {}, before_[SDL_GAMEPAD_BUTTON_COUNT] = {};
    uint64_t lastRumbleMs_ = 0;
    bool rumbling_ = false;
};

// Turns tyre/rig state into motor levels. Heavy motor: impacts and load changes (bumps, landings, rim strikes).
// Light motor: tread sliding (wheelspin, lock-up, skids) and a little texture from rolling. Triggers: sliding under
// throttle (right) or brake (left).
struct Haptics {
    bool enabled = true;
    float strength = 1.0f;
    float low = 0, high = 0, lt = 0, rt = 0;  // last output, for the UI meters

    void update(const tws::WheelLab& lab, double frameDt, bool driving, bool braking);
    void reset() { fzSlow_ = -1; rimTimer_ = 0; low = high = lt = rt = 0; }

private:
    double fzSlow_ = -1;
    double rimTimer_ = 0;
};
