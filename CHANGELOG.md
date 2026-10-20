# Changelog

## v0.3.0: The first rideable bike (M1)

- **Ride mode (the default):** a 300cc two-stroke enduro bike on a 160 m test area with a tabletop, a double,
  rollers, logs, wet roots, a rock garden, a hill climb and a wheelie straight. Chase, close, side and orbit cameras.
  A HUD (speed, gear, rpm, clutch) and a panel (assists, tyre pressures, controller profile, haptics, telemetry).
- **The bike is a real multibody** (Featherstone articulated-body algorithm):
  - Frame, steering, fork slider, front wheel, swingarm, rear wheel and the rider's mass, joined by exact joints.
  - Gyroscopic effects, weight transfer, fork dive and anti-squat all come from the physics.
- **Suspension:** a progressive fork and shock with preload, high/low-speed damping, stiction and bump stops.
  The bike settles to 44 mm front and 102 mm rear sag.
- **Drivetrain:**
  - 300 two-stroke torque curve, crank and flywheel inertia, idle and limiter.
  - Stalls (Y / Enter restarts it).
  - Clutch that sticks or slips, plus an auto-clutch assist.
  - 6 gears with neutral between 1st and 2nd.
  - Chain forces on the swingarm and frame.
- **Brakes** react through the fork and swingarm.
- **The rider:** a 75 kg body the muscles move fore/aft, up/down and side to side. A "lean intent" brain
  countersteers to hold the lean you ask for (full sim: raw bar torque). Auto-dab puts a foot down when stopped.
  The full articulated rider comes in M3.
- **Checked by tests:**
  - holds the lean it's asked for (24 deg)
  - clutch-pop wheelie
  - 0-100 km/h in 4.9 s
  - 50 km/h stop in 14 m at 0.7 g
  - repeatability
- **Controller:**
  - MX and hard-enduro profiles.
  - Haptics on the bike: bumps, landings, rim strikes, engine buzz, wheelspin and front slides.
- Hardpack grip raised slightly (1.0 grip / 0.85 sliding) to match knobby-tyre data.

## v0.2.1: Fast-impact fixes, lower pressures

- **Fixed: violent launches and spikes when anything fast happened.**
  - Tyre damping now builds with compression (Hunt-Crossley) and is capped at the elastic force. Before, a fast hit
    produced a huge force the instant the tread touched.
  - Rim strikes run in four substeps: about 100 bottomed-out elements on a 13 kg wheel was at the edge of what one
    1 ms tick can handle, and the wheel could gain energy and be flung.
  - The rig carriage now has real momentum and a limited tow force. It used to hold its speed with unlimited force,
    which threw the wheel straight up off the log at speed.
  - Result: at 20 m/s into the log the hub now peaks at 0.59 m (was 1.6 m), and peak forces dropped by about 3x.
- Tyre pressure goes down to **0 psi** (a flat tyre: only the carcass holds it up). The panel warns below 7 psi.
- The world beyond the test area is endless flat ground, so a fast free wheel no longer rolls off the edge.
- New dev tool: `sim_tests --stress` (rig and free wheel at 4-25 m/s, worst-case forces).

## v0.2.0: Controller and haptics

- **Controller support (SDL3; Xbox layout, GameSir tested as XInput):**
  - Picked up when plugged in, at any time.
  - The game switches to the controller on any button press and back on keyboard or mouse input, as in The Ledger.
  - Analogue triggers for drive and brake; the left stick steers the rig or pushes the free wheel; bumpers lean the
    rig; buttons for hop, mode, scene, reset, pressure and speed; the right stick moves the camera.
- **Haptics from the sim:**
  - Heavy motor: jolts and load changes, plus rim strikes.
  - Light motor: tread sliding, plus a rolling hum.
  - Trigger rumble for spin under drive and slide under the brake, on pads that support it.
  - On/off switch, strength slider and live motor meters in the panel.
- README: a step-by-step launch guide.

## v0.1.0: Wheel lab (M0)

- **The tyre:** 128 x 7 carcass elements over a round tread profile.
  - Each element has an air-pressure spring, a carcass spring and hysteresis damping, plus a brush bristle that
    sticks or slides.
  - Rim strikes, with a pinch that absorbs energy.
  - Tested values: about 90 kN/m vertical stiffness at 0.9 bar (rising with pressure) and about 3% rolling
    resistance. A locked wheel slides at the surface friction, and the side force builds with slip angle up to the
    friction limit.
  - Camber thrust, static grip and wheelspin all come out of the model.
- **The ground:** a 5 cm heightfield with materials (hardpack, loam, rock, wet root, wood), plus capsule and sphere
  obstacles.
- **The wheel lab:**
  - A quarter-bike rig on an obstacle lane: rock step, root, log, rock, rollers.
  - A free wheel you can launch, push, hop, drive and brake.
  - Live footprint map, force readouts and plots.
- **Under the hood:**
  - Fixed 1 kHz deterministic physics, about 60 µs per tick on the owner's laptop.
  - 13 physics tests.
  - CI builds the Windows exe and runs the tests.
