# Changelog

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
