# 04: Controls

Controller first, because the important inputs are analogue. Every binding can be remapped, and every axis has its
own deadzone, response curve and invert setting.

## The problem: a gamepad has only two analogue triggers

A real dirt bike has four analogue inputs: throttle, clutch, front brake and rear brake. A pad has two triggers. So
there are **profiles**. A digital button bound to an analogue input ramps at a speed you can set, so a tap gives a
light touch and holding it gives full force.

| Input | **MX profile** (default) | **Hard enduro / wheelie profile** |
|---|---|---|
| Throttle | RT (analogue) | RT (analogue) |
| Front brake | LT (analogue) | RB (ramped) |
| Rear brake | LB (ramped) | LB (ramped) |
| Clutch | RB (ramped), or auto-clutch assist | LT (analogue) |

Any extra analogue axis can be bound to any input, so pads with back paddles, racing pedals or a USB handbrake used as
a clutch all give more real control.

## Default gamepad layout

| Control | Action |
|---|---|
| **Left stick X** | Steer / bike lean (see [physics spec §6.3](03_PHYSICS_SPEC.md): "lean intent" or "bar torque") |
| **Left stick Y** | Body height: up = extend / stand tall, down = crouch / preload. Pumping and hops come from this. |
| **Right stick X** | Body lean relative to the bike (hang off, push the bike down, counter-lean) |
| **Right stick Y** | Body fore/aft (weight over the bars ↔ over the rear wheel) |
| RT / LT / LB / RB | Throttle, brakes, clutch (see the profiles above) |
| **A / X** | Shift up / shift down |
| **B** | Sit / stand toggle (left stick Y adjusts height within the posture) |
| **Y** | Engine start (after a stall) |
| **D-pad left / right** | Dab left / right foot (foot out in a corner, trials dab) |
| **D-pad down (hold)** | Rewind |
| **D-pad up** | Respawn at the last checkpoint |
| **L3** | Bail (jump off the bike) |
| **R3** | Cycle camera |
| **View / Back (hold 0.4 s)** | Instant restart |
| **Start** | Pause / menu |

### As built in M1 (v0.3.0)

The layout above is the target. M1 implements it with these differences (the full rider and rewind arrive later):
B picks the bike up (the sit/stand toggle comes with the M3 rider), View goes back to the start, D-pad up hides the
panel, R3 cycles the camera, and rewind and checkpoints are not in yet (M2). See the README for the exact current
mapping.

## Keyboard + mouse

Supported but not the main target:
- WASD: lean/steer and body fore/aft
- Mouse movement: body lean, as fine control
- Shift and Space: clutch and rear brake, ramped
- Arrow keys: throttle and front brake

## Rumble and gyro

Rumble carries **real sim data**, mapped to the two motors:
- Low-frequency motor: engine rpm and load, suspension bottoming, landings
- High-frequency motor: knobs biting or sliding, the tyre rattling over rock

DualSense haptic triggers (later): clutch friction point and brake lock-up felt in the triggers.

Gyro aiming of body lean is an optional extra (later).

## Assists (all on by default for new players; each one can be turned off)

| Assist | What it does |
|---|---|
| Lean intent | The stick sets the bike lean you want; the brain countersteers for you. Off = raw bar torque. |
| Auto-clutch | Engages and slips the clutch by rpm (like a real auto-clutch). Off = manual. |
| Auto-dab | The rider puts a foot out at very low speed instead of tipping over. |
| Body helper | The brain adds small corrections to keep the body centred on landings. |
| Stall recovery | Restarts the engine automatically after a stall. |

Full sim = everything off. The assist state is shown on leaderboards and ghosts.
