# 02: Tech stack

## Decision: custom C++20 engine (SDL3 + OpenGL 4.5)

The owner asked for "whichever is most performant and reliable". For this game that is a small custom engine in C++,
the same family as Crisp:

| Reason | Detail |
|---|---|
| The physics is custom anyway | No off-the-shelf engine's vehicle or tyre physics comes close to the [physics spec](03_PHYSICS_SPEC.md). Unity, Unreal and Godot would only draw the picture, and we would fight their frame loop, garbage collector and physics. |
| A 1 kHz physics loop | Every tick has to be fast and have the same cost each time. C++ has no garbage-collector pauses, gives control over memory layout (arrays of tyre elements, terrain tiles) and supports SIMD later. |
| Determinism | Same seed + same inputs = same result, bit for bit, on one machine. That makes exact rewind, replays, ghosts and regression tests possible. We control the floating-point settings (`/fp:precise`, no `-ffast-math`, no FMA contraction in sim code). |
| An integrated-GPU laptop | A lean forward renderer that we fully control outperforms a general engine's default pipeline on a Ryzen 4800H with integrated Radeon graphics. |
| Known-good toolchain | Crisp already proves that C++20, SDL3, CMake, VS Build Tools 2026 and GitHub Actions work on this machine. MX Simulator itself is a small custom engine. |
| Reliability | Nothing upstream can break us: no engine upgrades and no licence changes. Dependencies are pinned by hash. |

The cost is that we write our own tools (editor UI, debug views). **Dear ImGui** makes that cheap.

## Libraries (all permissive, pinned by URL + SHA256 through CMake FetchContent)

| Library | Use |
|---|---|
| **SDL3** (static) | Window, input, gamepad + rumble + gyro, audio output, timing |
| **OpenGL 4.5 core** via **glad** | Rendering (supported by the Radeon Vega iGPU). One thin renderer layer, so Vulkan could be added later if ever needed. |
| **Dear ImGui** | Debug views, telemetry, setup screens, editor |
| **stb_image / stb_vorbis** | Textures, sounds |
| **doctest** | Unit and physics validation tests |
| **Tracy** (optional, off by default) | Profiling |
| **ENet** (later) | Online play, as in Crisp |

The maths is our own: small `Vec3`, `Quat` and `Mat3` types in `sim/`, so the simulation has no outside maths dependency
and its floating-point behaviour stays under our control.

## Architecture

```
TwoWheelSimulator/
  CMakeLists.txt
  sim/            # The physics. Plain C++20: no SDL, no OpenGL, no ImGui. Builds and tests headless.
    math/         #   Vec3, Quat, Mat3, fixed helpers
    terrain/      #   heightfield tiles, materials, deformation, rigid obstacles
    tyre/         #   flexible-ring tyre, knobs, contact, friction, soil shear
    vehicle/      #   multibody: frame, fork, swingarm, wheels, linkages, steering
    drivetrain/   #   engine, clutch, gearbox, chain, brakes, pedalling
    rider/        #   articulated rider, muscle controllers, the "brain" (assist levels)
    world.*       #   SimWorld: tick(), state save/restore (rewind), input frames
  game/           # The app: SDL3 window, renderer, audio, input mapping, camera, UI, replays, editor
  tests/          # doctest: unit tests + physics validation against real-world data
  tools/          # build scripts, data converters
  data/           # vehicles/*.json, tyres/*.json, materials/*.json, tracks/
  docs/
```

**The rule:** `sim/` never includes anything from `game/`. The game reads sim state, interpolates it for drawing and
feeds the sim one `InputFrame` per tick. The sim owns everything that affects the outcome.

## The loop

- **Physics: fixed 1000 Hz** (`SimConfig::tick_hz`). Tyre contact can substep inside a tick if stiffness demands it.
- **Integration:** semi-implicit (symplectic) Euler for the multibody, with joints solved by a small sequential-impulse
  or articulated-body solver (decided in M1 by measurement). Tyre elements are stiff springs, either integrated
  implicitly or substepped.
- **Rendering:** as fast as the GPU allows (no 60 FPS cap), interpolating between the last two sim states.
- **Input** is sampled every frame and the latest value is used for each tick. Analogue inputs are not smoothed in
  the sim; the rider's muscles do the smoothing, as in real life.

## Performance budget (owner's laptop: Ryzen 7 4800H, integrated Radeon)

| Item | Budget |
|---|---|
| Sim tick (bike + rider + 2 tyres + terrain contact) | ≤ 0.25 ms on one core → 1 kHz uses ≤ 25% of a core |
| Terrain deformation updates | Batched; ≤ 0.5 ms per rendered frame |
| Rendered frame | Target ≥ 144 FPS at 1080p on the iGPU with default settings |
| Load from launch to riding | ≤ 5 s |

Measure before adding visuals. A setting may cost FPS, but the default must not.

## Determinism and tests

- `SimWorld` state can be copied in full (`save_state` / `load_state`). Rewind is a ring buffer of snapshots plus the
  inputs between them.
- Test: run a recorded input file twice and check the state hashes are identical at every tick.
- **Physics validation tests** compare against real data: tyre load-deflection curves, rolling resistance, cornering
  stiffness, the wheelie balance-point angle, sinkage in sand, fork and shock force-velocity curves, 0-100 km/h times
  and top speed per gear.

## Build and CI

- Local: VS Build Tools 2026, `cmake -G "Visual Studio 18 2026" -A x64 -B build`, then build Release.
- GitHub Actions: a Windows job builds the game, runs the tests and uploads the `TwoWheelSimulator-windows-x64` artifact.
  A Linux job builds `sim/` + tests only (GCC warnings check).
- Static MSVC runtime, so the .exe runs without installing anything.
