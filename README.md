# Two Wheel Simulator

A 3D off-road motorcycle simulator: hard enduro, wheelies, motocross, trials and freestyle (bicycles later).
Think MX Simulator, but the tyres and the ground interact the way they really do, you get more control over both the
bike and the rider's body, and it has quality-of-life features: instant restart, rewind, practice tools, replays and
telemetry.

**Status: M1, the first rideable bike.** A 300cc two-stroke enduro bike on a test area with jumps, rollers, logs,
a rock garden, a hill climb and a wheelie straight. The wheel lab from M0 is still there (F2). The design lives in
[`docs/`](docs/).

## How to launch

**Option A: the copy built on this PC (quickest).**
1. Open File Explorer and go to
   `C:\Users\matth\Desktop\Personal\Development\Games\TwoWheelSimulator\build\Release\`
2. Double-click **`twowheel.exe`**.
3. Optional: right-click `twowheel.exe` → *Show more options* → *Send to* → *Desktop (create shortcut)* to get a
   desktop icon.

**Option B: download the latest build from GitHub.**
1. Open the repo's **Actions** tab: <https://github.com/MartyJCross/TwoWheelSimulator/actions>
2. Click the newest run with a green tick.
3. Scroll to **Artifacts** at the bottom and click **TwoWheelSimulator-windows-x64** to download the zip.
4. Unzip it anywhere (right-click → *Extract All*) and double-click **`twowheel.exe`**.
5. If Windows shows "Windows protected your PC", click **More info → Run anyway**. The exe isn't code-signed, so this
   is expected.

Nothing needs installing. Close the game with **Esc** or the window's X.

**Controller:** plug it in (USB or Bluetooth) before or after launching; it's picked up automatically. Press any
button on it to switch to it; touching the keyboard or mouse switches back. The **Controller** section of the panel
shows its name, a haptics on/off switch, a strength slider and live meters for each motor.

## Riding (the default mode)

You start on the test area, engine idling in 1st with the auto-clutch on: just open the throttle.

- **Straight ahead:** a tabletop jump. **Left:** a double and the wheelie straight (rock stripes every 10 m).
  **Right:** logs, wet roots, the rock garden, rollers and, far right, a loam hill climb.
- Steering is **lean intent** by default: push the stick the way you want to lean and the rider countersteers for
  you. Turn it off in the panel for raw bar torque (full sim).
- **Wheelies:** sit back (right stick down), and either open the throttle hard in 1st/2nd or pull the clutch, rev
  it and dump it. Hold it on the throttle; tap the rear brake to bring the nose down.
- If you crash: **B** (or **T**) picks the bike up where it is; **View** (or **R**) goes back to the start.

**Controller (MX profile, default):**

| Input | Action |
|---|---|
| RT | Throttle |
| LT | Front brake |
| LB | Rear brake (ramps in while held) |
| RB | Clutch (ramps in while held; the auto-clutch works on its own) |
| Left stick X | Steer / lean |
| Left stick Y | Stand tall (up) / crouch (down): pump, preload, absorb |
| Right stick | Body: forward/back (wheelies, braking) and side to side |
| A / X | Shift up / down (1-N-2-3-4-5-6; neutral is a half-click up from 1st when stopped) |
| Y | Start the engine after a stall |
| B | Pick the bike up |
| D-pad left / right | Put a foot down (dab) |
| D-pad up | Hide/show the panel |
| R3 | Camera (chase, close, side, orbit) |
| View | Back to the start |
| Start | Pause |

The **hard enduro profile** (choose it in the panel) moves the clutch to **LT** (analogue) and the front brake to RB.

**Keyboard:**

| Key | Action |
|---|---|
| W / S / Space | Throttle / front brake / rear brake |
| A / D | Steer |
| Left Shift | Clutch |
| Arrow keys | Body forward/back and side to side |
| Ctrl | Crouch |
| E / Q | Shift up / down |
| Enter | Start the engine |
| Z / X | Dab left / right foot |
| T / R | Pick the bike up / back to the start |
| C | Camera |
| P | Pause |
| F1 / F2 / Esc | Panel / switch to the wheel lab / quit |
| Right mouse drag, wheel | Look around, zoom |

**Haptics** on the bike: the heavy motor carries bumps, landings, rim strikes and a light engine buzz; the light motor
carries tyre sliding (wheelspin, front washing out, locked wheels); trigger rumble (Xbox pads) buzzes RT on rear
wheelspin and LT on a sliding front.

## The wheel lab (F2)

**Two modes:**
- **Quarter-bike rig:** a wheel carrying a load on a 30 cm-travel shock, pulled along the obstacle lane at a set speed:
  a 10 cm rock step, a wet root, a 24 cm log, an off-centre rock and loam rollers. Set the speed, slip angle (steer),
  camber (lean), load, drive and brake torque, and tyre pressure, and watch the forces.
- **Free wheel:** a wheel on its own. Launch it rolling, push it, make it hop, drive or brake it. A rolling wheel stays
  up by itself; a stopped one falls over, as a real one does.

**What to look at:**
- The **footprint** window (top right) is the tyre unrolled around the bottom: each cell is one piece of tread.
  Yellow to red is contact pressure; cyan means it is sliding. Lower the pressure and the patch gets longer and wider.
- The **measurements** panel: forces, slip, deflection, patch area, elements sliding, rim strikes.
- The green line is the total force from the ground; the pink one is its horizontal part (x4).

**Wheel lab controller (Xbox layout):**

| Input | Action |
|---|---|
| RT | Drive torque (analogue) |
| LT | Brake (analogue, fine control near the top of the travel) |
| Left stick | Steer the rig (slip angle) / push the free wheel sideways |
| LB / RB | Lean the rig wheel (camber) |
| A | Hop (free wheel) |
| B | Switch rig <-> free wheel |
| X | Switch obstacle lane <-> flat ground |
| Y | Reset / relaunch |
| D-pad left / right | Tyre pressure -1 / +1 psi |
| D-pad up / down | Speed +/- 0.5 m/s |
| Right stick | Camera; R3 resets it, L3 cycles zoom |
| Start | Pause |
| Back / View | Hide the panels |

**Haptics** (sim data only, as in The Ledger):
- **Heavy motor:** jolts and load changes (bumps, rollers, the log, landings), plus a hard knock on a rim strike.
- **Light motor:** tread sliding (wheelspin, lock-up, skids), plus a faint hum while rolling.
- **Trigger rumble:** RT buzzes when the tyre spins under drive, and LT buzzes when it slides under the brake. This
  only works on controllers with impulse triggers (Xbox One/Series pads); the meters show the signal either way.

**Wheel lab keyboard and mouse:**

| Input | Action |
|---|---|
| Right mouse drag | Orbit the camera |
| Mouse wheel | Zoom |
| W / S (hold) | Drive / brake |
| R | Reset or relaunch |
| Space | Pause; `.` steps one physics tick while paused |
| F1 | Hide the panels |
| Esc | Quit |

Command line (for testing): `--lab`, `--ride-speed <m/s>`, `--ride-throttle <0..1>`, `--ride-cam <0..3>`, `--mode rig|free`, `--scene lane|flat`, `--speed <m/s>`, `--drive <N*m>`,
`--size 1600x900`, `--position x,y`, `--cam yaw,pitch,dist`, `--no-ui`, `--screenshot out.png --frames N`.

## Building

Windows, with Visual Studio Build Tools (2022 or 2026) and CMake 3.24+:

```
cmake -S . -B build -A x64
cmake --build build --config Release --parallel
build\Release\sim_tests.exe -v      # physics tests
build\Release\twowheel.exe
```

Dependencies (SDL3, Dear ImGui) download automatically, pinned by hash.

## Docs

| Doc | Contents |
|---|---|
| [Vision](docs/01_VISION.md) | Pitch, pillars, disciplines, vehicles, quality-of-life list |
| [Tech stack](docs/02_TECH_STACK.md) | Custom C++20 / SDL3 / OpenGL 3.3 engine, architecture, performance budget |
| [Physics spec](docs/03_PHYSICS_SPEC.md) | Flexible tyre, deformable terrain, bike multibody, drivetrain, rider |
| [Controls](docs/04_CONTROLS.md) | Gamepad profiles, layout, rumble, assists |
| [Roadmap](docs/05_ROADMAP.md) | Milestones to the first demo |
| [Open questions](docs/OPEN_QUESTIONS.md) | Decisions made and still to make |
