#include "pad.h"

#include <algorithm>
#include <cmath>

#include "wheel_lab.h"

namespace {

constexpr float kStickDeadzone = 0.12f;
constexpr float kTriggerDeadzone = 0.03f;

float trigger(Sint16 raw) {
    float v = raw / 32767.0f;
    return v < kTriggerDeadzone ? 0.0f : (v - kTriggerDeadzone) / (1.0f - kTriggerDeadzone);
}

}  // namespace

void Pad::openFirstAvailable() {
    if (pad_) return;
    int count = 0;
    SDL_JoystickID* ids = SDL_GetGamepads(&count);
    if (ids && count > 0) {
        pad_ = SDL_OpenGamepad(ids[0]);
        if (pad_) {
            id_ = ids[0];
            const char* n = SDL_GetGamepadName(pad_);
            name_ = n ? n : "Controller";
            SDL_PropertiesID props = SDL_GetGamepadProperties(pad_);
            rumble_ = SDL_GetBooleanProperty(props, SDL_PROP_GAMEPAD_CAP_RUMBLE_BOOLEAN, false);
            triggerRumble_ = SDL_GetBooleanProperty(props, SDL_PROP_GAMEPAD_CAP_TRIGGER_RUMBLE_BOOLEAN, false);
        }
    }
    SDL_free(ids);
}

void Pad::handleEvent(const SDL_Event& ev) {
    switch (ev.type) {
        case SDL_EVENT_GAMEPAD_ADDED:
            openFirstAvailable();
            break;
        case SDL_EVENT_GAMEPAD_REMOVED:
            if (pad_ && ev.gdevice.which == id_) {
                SDL_CloseGamepad(pad_);
                pad_ = nullptr;
                name_.clear();
                usingPad_ = false;
                openFirstAvailable();  // another one may still be plugged in
            }
            break;
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
            usingPad_ = true;
            break;
        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
            if (std::abs(ev.gaxis.value) > 12000) usingPad_ = true;
            break;
        case SDL_EVENT_KEY_DOWN:
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (usingPad_) stopRumble();
            usingPad_ = false;
            break;
        default:
            break;
    }
}

void Pad::update() {
    std::copy(std::begin(now_), std::end(now_), std::begin(before_));
    if (!pad_) {
        std::fill(std::begin(stick_), std::end(stick_), 0.0f);
        std::fill(std::begin(trig_), std::end(trig_), 0.0f);
        std::fill(std::begin(now_), std::end(now_), false);
        return;
    }
    const SDL_GamepadAxis axes[4] = {SDL_GAMEPAD_AXIS_LEFTX, SDL_GAMEPAD_AXIS_LEFTY, SDL_GAMEPAD_AXIS_RIGHTX,
                                     SDL_GAMEPAD_AXIS_RIGHTY};
    for (int s = 0; s < 2; ++s) {
        float x = SDL_GetGamepadAxis(pad_, axes[s * 2]) / 32767.0f;
        float y = SDL_GetGamepadAxis(pad_, axes[s * 2 + 1]) / 32767.0f;
        float len = std::sqrt(x * x + y * y);
        float scale = len < kStickDeadzone ? 0.0f : std::min(1.0f, (len - kStickDeadzone) / (1.0f - kStickDeadzone)) / len;
        stick_[s * 2] = std::clamp(x * scale, -1.0f, 1.0f);
        stick_[s * 2 + 1] = std::clamp(y * scale, -1.0f, 1.0f);
    }
    trig_[0] = trigger(SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_LEFT_TRIGGER));
    trig_[1] = trigger(SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER));
    for (int b = 0; b < SDL_GAMEPAD_BUTTON_COUNT; ++b) now_[b] = SDL_GetGamepadButton(pad_, (SDL_GamepadButton)b);
}

bool Pad::held(SDL_GamepadButton b) const { return now_[b]; }
bool Pad::pressed(SDL_GamepadButton b) const { return now_[b] && !before_[b]; }

void Pad::rumble(float low, float high, float lt, float rt) {
    if (!pad_ || !usingPad_) return;
    // Re-issue at ~60 Hz with a duration that outlives the gap, so the motors run smoothly without flooding the
    // driver at high frame rates.
    uint64_t now = SDL_GetTicks();
    if (now - lastRumbleMs_ < 16) return;
    lastRumbleMs_ = now;
    auto u16 = [](float v) { return (Uint16)(std::clamp(v, 0.0f, 1.0f) * 65535.0f); };
    bool any = low > 0.01f || high > 0.01f || lt > 0.01f || rt > 0.01f;
    if (!any && !rumbling_) return;
    if (rumble_) SDL_RumbleGamepad(pad_, u16(low), u16(high), 60);
    if (triggerRumble_) SDL_RumbleGamepadTriggers(pad_, u16(lt), u16(rt), 60);
    rumbling_ = any;
}

void Pad::stopRumble() {
    if (!pad_) return;
    SDL_RumbleGamepad(pad_, 0, 0, 0);
    if (triggerRumble_) SDL_RumbleGamepadTriggers(pad_, 0, 0, 0);
    rumbling_ = false;
}

void Haptics::update(const tws::WheelLab& lab, double dt, bool driving, bool braking) {
    const tws::TyreOutput& o = lab.out;
    // Weight the tyre carries at rest; bumps are measured against it.
    double restLoad = (lab.params.mass + (lab.mode == tws::LabMode::Rig ? lab.rig.load_kg : 0.0)) * tws::kGravity;
    double fz = o.normal_load;
    if (fzSlow_ < 0) fzSlow_ = fz;
    fzSlow_ += (fz - fzSlow_) * std::min(1.0, dt / 0.08);  // ~80 ms average: what is left over is the jolt
    double jolt = std::fabs(fz - fzSlow_) / restLoad;
    if (o.rim_strike) rimTimer_ = 0.09;
    rimTimer_ = std::max(0.0, rimTimer_ - dt);

    double slide = o.contacts > 0 ? (double)o.sliding / o.contacts : 0.0;
    double speed = std::sqrt(lab.vel.x * lab.vel.x + lab.vel.z * lab.vel.z);
    double rolling = o.contacts > 0 ? std::min(speed / 15.0, 1.0) * 0.06 : 0.0;  // faint texture while rolling

    double l = std::min(1.0, jolt * 0.9) + (rimTimer_ > 0 ? 0.85 : 0.0);
    double h = slide * 0.85 + rolling;
    low = enabled ? (float)std::min(1.0, l * strength) : 0.0f;
    high = enabled ? (float)std::min(1.0, h * strength) : 0.0f;
    rt = enabled && driving ? (float)std::min(1.0, slide * 1.2 * strength) : 0.0f;
    lt = enabled && braking ? (float)std::min(1.0, slide * 1.2 * strength) : 0.0f;
}
