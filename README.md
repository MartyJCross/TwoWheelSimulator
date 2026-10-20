# Two Wheel Simulator

A 3D off-road motorcycle simulator: hard enduro, wheelies, motocross, trials and freestyle (bicycles later).
Think MX Simulator, but the tyres and the ground interact the way they really do, you get more control over both the
bike and the rider's body, and it has quality-of-life features: instant restart, rewind, practice tools, replays and
telemetry.

**Status: M0, the wheel lab.** One wheel with the flexible tyre on rigid ground, so the tyre physics can be seen and
tested before the bike is built on top of it. The design lives in [`docs/`](docs/).

## Running the wheel lab

Get `twowheel.exe` from the latest GitHub Actions run (artifact `TwoWheelSimulator-windows-x64`) or build it (below).
It needs no install.

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

**Controls:**

| Input | Action |
|---|---|
| Right mouse drag | Orbit the camera |
| Mouse wheel | Zoom |
| W / S (hold) | Drive / brake |
| R | Reset or relaunch |
| Space | Pause; `.` steps one physics tick while paused |
| F1 | Hide the panels |
| Esc | Quit |

Command line (for testing): `--mode rig|free`, `--scene lane|flat`, `--speed <m/s>`, `--size 1600x900`,
`--position x,y`, `--cam yaw,pitch,dist`, `--no-ui`, `--screenshot out.png --frames N`.

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
