# 01: Vision

## The pitch

**Two Wheel Simulator** (working title) is a 3D off-road motorcycle and bicycle simulator in the spirit of
**MX Simulator**: the physics *is* the game. It has three differences:

1. **The wheels and the ground interact the way they really do.** The tyre is a flexible carcass with knobs, not a
   hard disc or a slip curve. The ground is a material that has its own grip, sinks, is pushed aside, ruts, dries
   out and gets churned up. Grip, slides, stalls, roost, ruts and crashes all come out of that contact.
2. **More manoeuvrability and control.** You control the bike *and* the rider's body separately and in analogue:
   lean, fore/aft weight, standing/sitting, weighting the outside peg, the clutch, both brakes, a foot dab. Riders can
   do scrubs, whips, pivot turns, splatters, wheelie-overs, stoppies and trials hops because the physics allows them,
   not because they are scripted tricks.
3. **Quality of life that MX Simulator never had.** Instant restart, rewind, checkpoints, practice tools, ghosts,
   replays, telemetry, setup screens that explain what each setting does, a track browser and a track editor.

**Motto: "Feel the ground."** When in doubt, choose whatever makes the contact between tyre and ground feel more
physical, more readable and more real.

## Pillars

| Pillar | What it means |
|---|---|
| **Real contact** | Tyre and ground are simulated, never faked. If a real riding technique does not work, fix the physics; never script the technique. |
| **Full control** | Every input is analogue where the real one is (throttle, clutch, brakes, body). The rider has a body you steer, not a pose that gets played. |
| **Readable** | Realism has to be *felt*, through sound (engine load, tyre scrub, knobs biting), camera, controller rumble, roost, ruts and an optional telemetry overlay. |
| **Respect the player's time** | Restart in one button, rewind, practice from any point, no menus between you and riding. |
| **Runs fast on modest hardware** | High FPS on the owner's integrated-GPU laptop. The physics runs on the CPU at 1 kHz and the renderer stays lean. |

## Disciplines

Hard enduro and wheelies come first (owner decision). The others follow in roadmap order.

| Discipline | Core skills | Setting |
|---|---|---|
| **Hard enduro** | Clutch control, tyre pressure, line choice, momentum, getting over logs and rock steps | Erzberg-, Romaniacs- and Red Bull Tennessee Knockout-inspired terrain: rock gardens, root chutes, mud climbs, log steps |
| **Wheelie** | Finding the balance point, rear brake, clutch, steering while on one wheel | Distance runs, slalom, set courses, wheelie-over obstacles |
| **Motocross / supercross** | Ruts, berms, scrubs, whoops, starts | MX Simulator's home ground: deformable dirt tracks |
| **Trials** | Balance, hops, splatters, pivot turns | Rock and log sections, scored by dabs |
| **Downhill / enduro MTB** | Pumping, braking points, weight shifts, line choice | Roots, rock gardens, drops, braking bumps |
| **Freestyle** | Air control, whips, flips, landings | Ramps and natural lines, judged runs |

## Vehicles

Real specifications, no brands (made-up names, true-to-life numbers):

| Class | Examples |
|---|---|
| Motorcycles | 300cc two-stroke enduro bike, 450cc four-stroke MX bike, 250cc four-stroke, 125cc two-stroke, trials bike |
| Bicycles (after the motorbike game is solid) | Downhill bike (200 mm travel), enduro MTB, trail hardtail, dirt-jump bike, BMX |

## Quality of life (the list MX Simulator players wish they had)

- **One-button restart** and **checkpoint respawn** (hard-enduro sections are long).
- **Rewind** a few seconds and take the line again. The physics is deterministic, so a rewind is exact.
- **Practice tools:** save a position and load it again, slow motion, freeze, "ghost" of your best run, a section loop.
- **Replays** with a free camera, scrubbing and export of the input file. They are tiny, because a replay is the inputs plus the seed.
- **Telemetry overlay** (optional): per-wheel load, slip, sinkage, contact patch, suspension travel, rpm, clutch
  engagement, rider centre of mass. It also teaches you *why* you crashed.
- **Bike setup with explanations:** every setting (tyre pressure, clicker settings, sag, gearing, map) says what it
  does and what you'll feel, plus a quick "softer / harder" button for each.
- **Assists you can turn off gradually**, down to full sim: lean assist, clutch assist, auto-dab, stability help.
  The default is helpful; experts go bare.
- **Controller first:** full remapping, adjustable deadzones and response curves, rumble that carries real data
  (knobs biting, engine vibration, bottoming out).
- **Track browser** inside the game, and a **track and terrain editor** that uses the same terrain the physics uses.
- **Fast loading** and no forced menus: launch the game and you are riding in seconds.

## Places and look

- **Places:** for now there is **one demo map**: a stylised off-road park with jumps (tables, doubles, a step-up),
  a hard-enduro section (rock garden, roots, log steps, a rock climb), a wheelie straight and open ground to play on.
  Real or fictional venues are decided later.
- **Look: stylised.** Clean shapes, flat-ish shading with soft lighting, strong readable colours per material (rock,
  loam, mud, sand and roots must be easy to tell apart at speed), and light fog for depth. This also keeps the
  renderer cheap on an integrated GPU. Ruts, roost and tyre marks are still drawn in detail, because they are gameplay.

## What this game is not

- Not an arcade game. There are no boost buttons, no auto-landing and no canned trick animations.
- Not a career-mode grind at first. Riding comes first and the meta-game comes later.
