# CLAUDE.md: Project Constitution (Two Wheel Simulator)

Read this file first, every session. It is the operating manual for any AI working in this repo. **Keep it current.**
When you learn something a future session would need (an owner preference, a trap, a tool), write it here. When a
rule here conflicts with a doc in `docs/`, this file wins; fix the doc.

## 1. What this project is

A **3D off-road motorcycle and bicycle simulator**: MX Simulator, but with **tyre and ground physics that work the
way they do in real life**, **more manoeuvrability and control** (analogue bike *and* rider-body control), and the
**quality of life** MX Simulator lacks (restart, rewind, practice tools, replays, telemetry, explained setup).
Hard enduro and wheelies come first; motocross, trials, downhill MTB and freestyle follow.

**Motto: "Feel the ground."**

| Doc | What it holds |
|---|---|
| `docs/01_VISION.md` | Pitch, pillars, disciplines, vehicles, the QOL list |
| `docs/02_TECH_STACK.md` | Why custom C++, libraries, architecture, the loop, performance budget, tests, CI |
| `docs/03_PHYSICS_SPEC.md` | Tyre, terrain, obstacles, vehicle multibody, drivetrain, rider, validation table |
| `docs/04_CONTROLS.md` | Gamepad profiles and layout, keyboard, rumble, assists |
| `docs/05_ROADMAP.md` | Milestones M0–M5 with a Definition of Done each, then later work |
| `docs/OPEN_QUESTIONS.md` | The owner's answered decisions and the open questions |

Owner decisions so far: **3D**, custom C++ engine, **hard enduro + wheelies first**, **motorbikes before bicycles**,
a **stylised** look, **one demo map** with jumps and obstacles for now, and online play undecided but with room left
for it.

## 2. Golden rules (do not break these)

1. **The physics is never faked.** Grip, slides, wheelies, stalls, ruts, crashes and tricks come out of `sim/`. No
   slip-curve shortcuts that replace the tyre model, no canned trick or crash animations, no hidden "stability" forces.
   Assists are allowed only as a *rider brain* acting through the same controls a real rider has (bar torque, body,
   clutch), and every assist can be turned off.
2. **Fixed timestep and determinism.** The sim runs at 1000 Hz (`SimConfig::tick_hz`). Never use frame time in sim
   code. Same seed + same inputs = the same state hash. No `/fp:fast` or `-ffast-math`; `-ffp-contract=off`.
   Rendering interpolates between sim states. Cosmetics may use frame time.
3. **`sim/` has zero engine dependencies.** No SDL, OpenGL or ImGui. It builds and tests headless.
4. **Real numbers.** Vehicles, tyres and materials are data files with real-world values. Note the source in a comment.
5. **Performance is a feature.** No FPS cap. Measure FPS when adding visuals. The default settings must hold the
   budget in `docs/02_TECH_STACK.md` on the owner's laptop.

## 3. Repository layout

See `docs/02_TECH_STACK.md` → Architecture. In short: `sim/` (physics), `game/` (app), `tests/`, `tools/`,
`data/`, `docs/`.

## 4. Build and test

- Toolchain: VS Build Tools 2026. cmake is not on PATH; use
  `C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe`
  with `-G "Visual Studio 18 2026" -A x64 -B build`.
- Dependencies come through CMake FetchContent, pinned by URL + SHA256 (the same approach as Crisp).
- Build: `cmake --build build --config Release --parallel`. Exe: `build\Release	wowheel.exe`. Tests:
  `build\Release\sim_tests.exe -v` (prints measured values). `sim_tests --trace-lane <speed>` prints the rig's
  run along the obstacle lane (handy when tuning the tyre).
- Run the tests before every push. Light testing only: no long benchmark runs while the owner is using the PC.
- Windowed test runs go off-screen (no pop-ups on the owner's screen):
  `twowheel.exe --position -4000,-4000 --screenshot <scratchpad>/x.png --frames N [--no-ui] [--cam yaw,pitch,dist]`.
  Screenshot mode steps a fixed 1/60 s per frame, so a given frame count always shows the same moment.
- Don't name a header after a C/C++ standard header (`sim/` is a public include dir: a `math.h` there broke `<cmath>`).
- Bash heredocs turn `
` inside C string literals into real newlines: edit C++ with the Write/Edit tools.

## 5. The owner and how they work

- Windows laptop: Ryzen 7 4800H with integrated Radeon graphics only, so the game is GPU/fill-rate bound.
- They want **big work split into pieces, each committed and pushed** so progress survives if a session ends.
- Short, plain reports: what changed, what to try, what was verified vs what only they can judge (feel, FPS).
- `gh` is not installed; check CI with curl on `api.github.com/repos/MartyJCross/TwoWheelSimulator/actions/runs`.
- Remote: `https://MartyJCross@github.com/MartyJCross/TwoWheelSimulator.git`, branch `main`. The username has to be
  in the URL; without it, `git push` hangs on a hidden credential prompt.

## 6. Version

`project(TwoWheelSimulator VERSION x.y.z)` in `CMakeLists.txt` is shown in the window title. Bump it for every build
handed over (patch for fixes, minor for features) and add a `CHANGELOG.md` entry. CI (`.github/workflows/build.yml`)
builds on every push and uploads the artifact `TwoWheelSimulator-windows-x64`; give the owner the run link.

## 7. Progress checklist

- [x] Design docs (2026-10-20)
- [x] M0 Skeleton and wheel lab (v0.1.0, 2026-10-20)
- [ ] M1 Rideable bike on hard ground
- [ ] M2 Rigid obstacles + hard-enduro basics + rewind/restart
- [ ] M3 Articulated rider
- [ ] M4 Soft, deformable ground
- [ ] M5 First demo (hard-enduro hill + wheelie challenge)

### Where things stand (keep this current)

- **The tyre** (`sim/tyre.*`): the carcass elements are independent radial springs, with no neighbour coupling yet.
  Envelopment over sharp edges is therefore sharper than a real carcass, and rim strikes come a little easily on
  edges. Add bending coupling between elements in M2 (obstacles), as the physics spec §1.2 describes.
- **Rolling radius:** uses R_e = R0 - deflection/3, from the previous tick.
- **The rig** is a quarter bike (load on a shock), not a rigid mass: a rigid 150 kg on a tyre bounces like a pogo
  stick, which is real but useless for testing.
- **Friction values** (`sim/terrain.cpp`) are first estimates. Tune them against feel once the bike rides (M1).
