# Two Wheel Simulator

A 3D off-road motorcycle simulator: hard enduro, wheelies, motocross, trials and freestyle (bicycles later).
Think MX Simulator, but the tyres and the ground interact the way they really do, you get more control over both the
bike and the rider's body, and it has quality-of-life features: instant restart, rewind, practice tools, replays and
telemetry.

**Status: M0, the wheel lab.** One wheel with the flexible tyre on rigid ground, so the tyre physics can be seen and
tested before the bike is built on top of it. The design lives in [`docs/`](docs/).

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

## Using the wheel lab

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

**Controller (Xbox layout):**

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

**Keyboard and mouse:**

| Input | Action |
|---|---|
| Right mouse drag | Orbit the camera |
| Mouse wheel | Zoom |
| W / S (hold) | Drive / brake |
| R | Reset or relaunch |
| Space | Pause; `.` steps one physics tick while paused |
| F1 | Hide the panels |
| Esc | Quit |

Command line (for testing): `--mode rig|free`, `--scene lane|flat`, `--speed <m/s>`, `--drive <N*m>`,
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
