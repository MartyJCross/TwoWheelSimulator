# 03: Physics spec

This is the heart of the game. Everything here lives in `sim/`, runs at a fixed 1000 Hz, and is deterministic.
**Golden rule: if a real riding technique does not work, fix the physics. Never script the technique.**

Units are SI throughout (m, kg, s, N, rad). The world is Y-up and right-handed.

---

## 1. Tyre: a flexible carcass with knobs

The tyre is what makes or breaks this game, so it gets the most detailed model. It is a simplified relative of the
FTire / flexible-ring family, tuned to run in a 1 kHz budget.

### 1.1 Geometry

- **Rim:** rigid, part of the wheel body.
- **Carcass:** a ring of `Nc` sections around the wheel (default 64). Each section is a cross-section of `Nw` points
  across the **round tread profile** (default 7). The profile matters because a motorcycle tyre's crown is round:
  as the bike leans, the contact moves out across the tread and the side knobs take the load.
- Each point has a rest radius taken from the real tyre profile (e.g. 80/100-21 front, 140/80-18 rear, 2.4" MTB).
- Only the **sections near the ground are active** (those within ±60° of the lowest point, or any section touching
  an obstacle). The rest stay at rest shape and cost nothing.

### 1.2 Carcass deflection (quasi-static)

The carcass mass is tiny compared with the wheel, so we treat it as **massless and in equilibrium every tick**. That
avoids extremely stiff, fast-vibrating node masses.

- Each point has a **radial spring** to the rim. Its stiffness comes from **inflation pressure × the area it carries**
  plus sidewall stiffness, and it has hysteretic damping (this is where rolling resistance comes from).
- Each point has **bending springs** to its neighbours, around the ring and across the profile, so a load spreads
  the way a real carcass spreads it. The contact patch shape and **envelopment** (the tyre wrapping around a rock
  or root) come from this.
- Solved with a few Gauss-Seidel relaxation passes per tick against the ground surface (§2) and rigid obstacles (§3).
- **Pressure is the most important setting.** Lower pressure gives a longer, wider patch, more envelopment and more
  grip on rocks, but more squirm and a higher risk of rim strikes.
- **Rim strike / pinch:** when radial deflection exceeds the sidewall height, the rim contacts directly through a very
  stiff spring. With a tube, there is a chance of a pinch flat (optional, off by default). **Mousse** and **tubeless**
  inserts are tyre options with their own stiffness curves (real hard-enduro equipment).

### 1.3 Tread and friction: the brush model, per element

Every active contact element carries a **bristle**: a tangential deflection (2D, in the ground plane) that is stored
across ticks.

- Each tick the bristle base moves with the carcass point. The tip stays put while it **sticks**:
  `F_t = -k_t · δ - c_t · δ̇`.
- When `|F_t| > μ(v_slide, material, wetness) · F_n`, the element **slides**: the force is clamped to the friction
  limit and δ is pulled back onto the friction circle.
- `μ` has separate static and kinetic values and a Stribeck-style velocity dependence.

Everything in the classic tyre curves emerges from this; no Pacejka fit is used:
- longitudinal slip (wheelspin, lock-up)
- lateral slip angle
- **combined slip** (you cannot brake hard and corner hard at once)
- relaxation length (grip takes a moment to build)
- the self-aligning moment

### 1.4 Knobs in soft ground

On deformable materials (§2), the knobs dig in. Contact is then a balance between the carcass spring and the soil:

- **Pressure-sinkage (Bekker):** `p = (k_c / b + k_φ) · z^n`. The element sinks until the soil pressure equals the
  carcass pressure.
- **Shear traction (Mohr-Coulomb + Janosi-Hanamoto):** `τ_max = c + σ · tan φ`, and
  `τ = τ_max · (1 - e^(-j / K))`, where `j` is the accumulated shear displacement of the element.
  - Some wheelspin is needed for traction in loose dirt (`j` must build up).
  - Too much spin digs a hole.
- **Knob height and spacing** scale the effective shear area. A worn tyre has shorter knobs.
- **Bulldozing:** a sunk tyre has to push soil out of the way, which adds resistance (sand, deep mud).
- Sinkage and shear **write back to the terrain** (§2.3).

### 1.5 Outputs per wheel

Contact force and moment on the wheel body, plus per-element data for telemetry, sound and rumble:
- load
- slip state
- sinkage
- the material the tyre is touching

---

## 2. Terrain: a material that remembers

### 2.1 Representation

- **Base heightfield** in tiles (64 m × 64 m, 0.25 m cells) for the whole map.
- **Detail grid** (default 2.5 cm cells, chunks of 64 × 64) allocated **lazily** wherever a tyre touches the ground
  or a track designer has added detail. Tracks only pay for detail where riding happens.
- Each detail cell stores:
  - `base_height`: the hard layer (bedrock or hardpack)
  - `loose_depth`: soft material on top
  - `material_id`
  - `moisture` (0–1)
  - `compaction` (0–1)
- Queries (height, normal and material at a point) are bilinear and deterministic.

### 2.2 Materials (data-driven, `data/materials/*.json`)

Each material has:
- `μ` dry and wet
- Bekker parameters `n, k_c, k_φ`
- cohesion `c` and friction angle `φ`
- shear modulus `K`
- bulk density
- angle of repose
- how moisture changes all of the above
- particle, colour and sound ids

Starting values come from the terramechanics literature (Wong, *Theory of Ground Vehicles*).

| Material | Character |
|---|---|
| Rock (granite, limestone, slate) | Rigid. Grippy dry, slick wet; limestone polishes. |
| Hardpack / clay | Barely deforms; very slick when wet. |
| Loam | The good stuff: deforms a little, grips well. |
| Loose dirt / intermediate | Ruts form; roost flies. |
| Sand | Low cohesion, deep sinkage, slumps to its angle of repose, bulldozes. |
| Mud | High moisture: sinks, low shear strength, holds ruts forever. |
| Roots (on any soil) | Slick, especially wet. These are rigid obstacles (§3). |
| Snow / grass | Later. |

### 2.3 Deformation (volume is conserved)

- **Sinkage:** soil under a loaded element compresses. Compaction rises, and the plastic part of the sinkage stays,
  forming a **rut**. A little elastic rebound is allowed.
- **Displacement:** material that cannot compress more is pushed sideways and forward into lips and berms.
- **Roost:** knobs shearing loose soil throw it backwards as particles that carry mass and volume. Where they land
  they add material. Roost can hit following riders later.
- **Slumping:** cells steeper than the angle of repose relax over time. Sand fills ruts quickly; mud keeps them.
- Deformation is applied in a deterministic order once per tick. Mesh rebuilds for rendering are batched and
  happen once per rendered frame.

---

## 3. Rigid obstacles

- **Rocks:** convex hulls, or meshes split into convex parts, static.
- **Logs:** capsules. Static first; loose logs that roll and slide come later.
- **Roots:** capsules lying on or in the soil, with a wet-slick material.
- Tyre elements contact obstacles through the same carcass solve, so the tyre wraps around a root or rock edge.
- **Bike body colliders:** convex shapes for the skid plate, frame rails, footpegs, bars, engine cases and rear fender.
  The skid plate grinding over a log is a core hard-enduro move and has to work.

---

## 4. Vehicle: a multibody

### 4.1 Bodies and joints (motorcycle)

| Body | Joined to | Joint |
|---|---|---|
| Frame + engine | (root) | free |
| Bars + upper fork (triple clamps) | frame | revolute about the steering axis (rake, offset) |
| Fork lowers | upper fork | prismatic along the fork axis (travel limits) |
| Front wheel | fork lowers | revolute |
| Swingarm | frame | revolute at the pivot |
| Rear wheel | swingarm | revolute |

- **Fork:** spring (coil or air, with progression), speed-sensitive damping (compression/rebound, low/high speed),
  **stiction**, top-out and hydraulic bottoming.
- **Shock:** the linkage is modelled as a **leverage-ratio curve** (from real bikes) between swingarm angle and shock
  stroke, with the same spring, damper, stiction and bottoming model as the fork.
- **Chain:** a tension force acts along the real chain line between the countershaft sprocket and the rear sprocket.
  **Anti-squat** and chain pull on the swingarm come out of the geometry. Change the sprocket and the bike behaves
  differently, as in real life.
- **Brakes** act on the wheel against the fork or swingarm, so fork dive and brake squat come out of the physics.
- **Steering** has a lock-to-lock limit and an optional steering damper.
- Mass and inertia come from real specs, split per body (frame, engine, fuel, wheels, suspension).
  A 300cc two-stroke is about 104 kg without fuel; a 450cc four-stroke is about 108 kg.
- Aerodynamic drag on bike + rider (CdA about 0.6–0.9 m², depending on posture).

**Bicycles** use the same bodies: frame, fork, the rear-suspension linkage as a leverage curve (or a rigid rear for a
hardtail), and wheels. Their mass and inertia are much lower, so the rider's body dominates.

### 4.2 What must emerge (and is tested)

- **Self-stability:** with no rider input, a moving bicycle steers itself upright in the right speed range. Tested
  against the **Whipple bicycle benchmark** (Meijaard et al. 2007) eigenvalues.
- **Countersteering:** pushing the right bar leans the bike right.
- **Gyroscopic air control:** throttle in the air lifts the nose and rear brake drops it.
- **Wheelies** and **stoppies** with a real balance point.
- **Rut tracking:** the tyre follows a rut and you steer by leaning into it.

---

## 5. Drivetrain

### 5.1 Engine

- A torque map by rpm and throttle from real dyno shapes. Two-strokes get a power-valve hit; four-strokes are linear.
- **Crank + flywheel inertia.** A **flywheel weight** is a setting: a heavier flywheel stalls less and gives smoother
  traction, but revs up more slowly.
- Engine braking (pumping and friction losses), idle control, rev limiter.
- **Stalling:** under load below the idle-recovery limit, the engine stops. It restarts with the start button (with a
  short delay). Stalling on a climb is a real hard-enduro failure, and it is meant to be.
- **Throttle response:** a little lag from intake and fuel delivery, from data.

### 5.2 Clutch: the skill of the game

- Wet multi-plate. The torque it can pass = `f(lever position) · plate friction`. The lever curve (free play,
  engagement point, full lock) is a setting.
- A **stick/slip model** between the engine side and the gearbox side. Slipping it puts the engine in its power band
  at low wheel speed. Dumping it pops a wheelie or a hop.
- Clutch heat and fade come later (riding the clutch on a long climb has a price).
- An **auto-clutch assist** exists for beginners (rotary/auto-clutch, which is also real equipment).

### 5.3 Gearbox, chain, brakes

- Real ratios, primary drive and sprockets (all editable).
- Shifting takes time. Clutchless upshifts work under a throttle cut.
- **Brakes:** analogue lever and pedal. Torque = input × hydraulic gain × pad μ. They can lock the wheel. No ABS.

### 5.4 Bicycle drive

- **Pedalling** comes from the rider model: leg torque limited by human power and cadence. About 250 W sustained, and
  more than 1000 W in a short sprint.
- The freewheel has engagement points. Cassette shifting takes time.
- The cranks have a real position, so **pedal strikes** happen.

---

## 6. Rider: a body, not a pose

### 6.1 Body

- About 15 rigid segments (pelvis, torso, head, upper arms, forearms, hands, thighs, shins, feet), with masses and
  inertia from anthropometric tables. A 75 kg rider by default; height and weight are adjustable.
- Joints have real limits.
- **Contacts:**
  - Hands grip the bars through a constraint that breaks if the force is too high.
  - Feet sit on the pegs or pedals, with friction, and can be bumped off.
  - The seat contacts the rider when sitting.
  - Boots contact the ground when dabbing.

### 6.2 Muscles

- Every joint is driven by a **PD controller with a torque limit** (strength) and adjustable stiffness. Stiff
  arms carry shocks into the bike; soft legs soak a landing.
- **The rider's "brain"** turns player commands into joint targets using simple inverse kinematics:
  - body position fore/aft
  - body height (crouch ↔ extend)
  - body lean relative to the bike
  - weighting the left or right peg
  - sit or stand
- The body follows those targets **within strength and reaction-time limits**. Pumping, preloading, bunny and
  trials hops, soaking a landing, the attack position, hanging off and pushing the bike down in a corner all
  come out of moving a real mass.

### 6.3 Steering and lean (the main control design)

A real rider steers with torque on the bars. The game supports that, plus a "lean intent" layer with assist levels:

| Mode | Stick X means | Who it's for |
|---|---|---|
| **Lean intent** (default) | The bike lean angle you want. The brain applies bar torque (countersteering) to get and hold it, through the real physics. At walking speed it blends into a direct steering angle (trials, pivot turns). | Most players |
| **Bar torque** (full sim) | Torque on the handlebars | Experts |

Body lean (right stick) is **separate from bike lean**. You can push the bike down under you, hang off, or counter-lean
on a slippery off-camber. That is the manoeuvrability MX Simulator lacks. In the air, body movement and bar torque
rotate the bike about its long and vertical axes (whips, table tops) under conservation of angular momentum.

### 6.4 Crashes

- A crash happens when a limit is broken:
  - the grip constraint breaks
  - the head or torso hits terrain or an obstacle hard
  - the bike lands upside down
- The muscles then go limp and the rider becomes a ragdoll.
- A **bail** button lets the rider jump off on purpose.
- Restart and rewind are one button away ([vision](01_VISION.md), quality of life).

---

## 7. Validation table (tests in `tests/`)

| Check | Real reference |
|---|---|
| Tyre radial stiffness vs pressure; load-deflection curve | Manufacturer / published tyre data |
| Rolling resistance coefficient on hardpack, loam, sand | Literature |
| Cornering stiffness and peak lateral force vs load | Published motorcycle tyre measurements |
| Sinkage and drawbar pull in sand | Wong's terramechanics data |
| Whipple bicycle self-stable speed range and eigenvalues | Meijaard et al. 2007 benchmark |
| Wheelie balance-point angle for a given bike + rider | Geometry: centre of mass over the rear contact |
| 0-100 km/h, top speed per gear | Published bike tests |
| Fork and shock force-velocity curves | The model's own input data (it reproduces them) |
| Determinism: same inputs give the same state hash for 60 s | n/a |
