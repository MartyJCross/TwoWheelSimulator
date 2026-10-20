# 05: Roadmap

Build order. Each milestone ends with something you can run and see. Every piece is committed and pushed as it lands.
The progress checklist lives in `CLAUDE.md` §7.

## M0: Skeleton and wheel lab

- CMake project: `sim/` library, `game/` app (SDL3 + OpenGL 3.3 + Dear ImGui), `tests/` (built-in runner), GitHub Actions
  (Windows build + tests + artifact; Linux sim tests).
- A 3D debug renderer: grid, heightfield, wheel, tyre elements, force arrows, a free/orbit camera.
- **One wheel** with the flexible tyre (§1 of the physics spec) on a rigid heightfield: drop it, roll it, push it
  sideways, drive it with torque, change pressure live and watch the contact patch.
- Tests: load-deflection vs pressure, rolling resistance, longitudinal and lateral force curves look right, determinism.
- **Done when:** the wheel lab runs at ≥ 144 FPS with the 1 kHz sim, and the tyre visibly wraps over a step and a root.

## M1: A rideable bike on hard ground

- The motorcycle multibody (§4): frame, steering, fork, swingarm via a leverage curve, wheels, chain force, brakes.
- Drivetrain (§5): engine map, flywheel, clutch stick/slip, gearbox, stalling, restart.
- A placeholder rider: a rigid mass that moves fore/aft and up/down, plus the lean-intent controller.
- Gamepad input, chase camera, a basic HUD (speed, gear, rpm, clutch).
- Tests: Whipple benchmark stability, wheelie balance point, 0-100 km/h.
- **Done when:** you can ride around a hardpack test area, wheelie with clutch and rear brake, stall it and restart it.

## M2: Rigid obstacles and the hard-enduro basics

- Rocks, logs and roots (§3). Bike body colliders, including the skid plate.
- Tyre envelopment over obstacles; rim strikes; mousse and tubeless options.
- Rewind, instant restart, checkpoints (QOL that pays off immediately in testing).
- **Done when:** you can clutch-pop over a log, grind the skid plate over it and roll down, and climb a rock step at
  low pressure that you can't climb at high pressure.

## M3: The articulated rider

- The 15-segment body, muscles, IK brain, grip/peg/seat contacts, dabs, crashes and ragdoll, bail.
- Body controls on the right stick and left stick Y.
- **Done when:** pumping and hops work through body movement, landings can be soaked, and bad landings throw you off.

## M4: Soft ground that remembers

- Detail terrain grid, materials, Bekker sinkage, shear traction, deformation, ruts, berms, roost, slumping, moisture.
- Terrain mesh updates for rendering, batched and checked for FPS.
- Tests: sand sinkage and drawbar pull against Wong's data.
- **Done when:** mud climbs need wheelspin control, ruts form and hold the tyre, and sand slumps back.

## M5: First demo (hard enduro + wheelie)

- **The demo map:** a stylised off-road park with jumps (table, double, step-up), a hard-enduro section (rock
  garden, roots, log steps, a mud climb), a wheelie straight and open ground. Checkpoints and a timer on the enduro
  section.
- A wheelie challenge: distance plus a set course.
- Replays (inputs + seed), a ghost of your best run, the telemetry overlay, a bike setup screen with explanations.
- Sound: an engine sound driven by rpm and load, tyre/terrain sounds, impacts.
- A visual pass that stays inside the FPS budget.
- **Done when:** the owner plays it and the tyre-to-ground feel is "right".

## Later (order to be agreed)

- **M6** More bikes (450cc four-stroke, 250cc four-stroke, 125cc two-stroke, trials bike) and an MX/SX track with deformable dirt.
- **M7** Bicycles (after the motorbike game is solid): DH, enduro, dirt jump; pedalling; a downhill track.
- **M8** Track and terrain editor and an in-game track browser.
- **M9** Online play (server-authoritative, as in Crisp) and leaderboards. Timing not decided; the architecture leaves room for it.
- **M10** Freestyle, trials scoring, more venues, weather (wetness changes material grip).
