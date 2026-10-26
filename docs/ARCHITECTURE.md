# 架构 · Architecture

JetCar Mini (originally 坠 · FALL) started as an offline soft-body crash film (MINI EV off a viaduct). It is growing into a small lab:
the same car gets a cold-gas thruster kit (10 nozzles, after the Tesla Roadster "Rocket"), controllers (MPC, PPO, ...)
learn to fly it off the viaduct and land, and every outcome can be rendered as film. The code is split so these
lines of work can proceed in parallel against fixed contracts.

## Layers

```
spec/*.json  ── language-neutral facts (world, presets, thrusters, vehicle, tasks) ───────────────┐
   │                                                                                             │
   ├─ src/core/spec.js ── JS view of the spec                       python/coldgas/spec.py ── Py view
   │                                                                                             │
   ├─ src/flight/         rigid 6-DOF cold-gas twin (fast, JS)  ⇆ parity ⇆  python/coldgas/  (fast, Py)
   │     propulsion.js     reference propulsion model                      env / MPC / PPO / bench
   │     contract.js       state / controller / trajectory contract        experiments/ (configs, runs, reports)
   │     mpc/              pure MPC `mpc` (JS only, no base pilot): schedule param, planner, optimizers
   │                       (sampling mppi/cem, gradient, hybrid), objectives (land, roll, gas)
   │     guidance/         dual nonlinear MPC (upper duration candidates + SQP, lower trajectory tracking)
   │     qp/               vendored OSQP WASM; no deployment solver installation required
   │     landing-modes.js  flight/ground ownership; ground controller currently brakes only
   │
   ├─ src/softbody/       XPBD node-beam crash solver (slow, high fidelity, JS only)
   │     solver.js         + `sim.actuators[]` hook for external forces (thrusters)
   │     actuators.js      thrusters on the lattice, closed loop through a Controller
   │     recording.js      compact recordings (public/sims/*.sim)
   │
   ├─ src/render/  stage (renderer + post), carRender (GPU-deformed car), effects, plumes
   ├─ src/scene/   layout (colliders from spec) + procedural viaduct set
   ├─ src/playback/view.js   a recording at sim time t: car, shards, dust, sparks, probes
   ├─ src/cinema/  Sequence (film time → sim time + camera), shot library, key moments, offline film frame
   ├─ src/programs/<id>/     self-contained experiences: crash · flight · install · ...
   └─ src/app/     shell: context + lazy assets, program registry (?program=<id>), transport bar
```

Two physics models on purpose: the **rigid twin** is what controllers are designed and trained on (thousands of
episodes per second, identical in JS and Python); the **soft body** is what gets filmed (plastic crush, glass,
parts coming off), driven by the same `Controller` interface through the actuator hook. Exact agreement is
required only between the two twins; the soft body is the "real world" the controllers are transferred to.

## Contracts (change only deliberately, with a version bump)

| contract | where | consumers |
|---|---|---|
| frames, units, nozzle order, propulsion model, launch/kick, traj format, spec hash, RNG | `spec/README.md` | everything |
| `FlightState`, `Controller`, state layout, `NU`, `CONTROL_DT` | `src/flight/contract.js` (+ Python mirror) | twins, soft-body actuators, viewers |
| `Propulsion` class (`step(u, dt)`, `valve`, `thrust`, `mdot`, `tank`) | `src/flight/propulsion.js` | twins, soft-body actuators |
| `sim.actuators[]` — `apply(sim, dt)` once per substep before integration | `src/softbody/solver.js` | `src/softbody/actuators.js` |
| `Program` — `init()`, `frame(dt)`, `film = {total, render(T, opts), describe()}` | `src/app/program.js` | shell, `tools/film/*.mjs` |
| `Sequence` / shot `cam(u, t) → CamPose` | `src/cinema/sequence.js`, `shots.js` | programs |
| `window.app` — `ready`, `error`, `stage`, `program`, `frame(dt)`, `film` | `src/app/main.js` | Node tools |

## Programs

`?program=<id>` picks `src/programs/<id>/index.js` (auto-discovered). A program builds its own scene objects on
the shared Stage, mounts its UI into `#hud`, and exposes a deterministic `film` so the same exporter renders any
of them: `npm run film -- program=<id> out=renders/x.mp4`.

- `crash` — the original viewer: six presets, live runs in a worker, director edit, HUD, audio (default).
- `flight` — thrust-assisted escape and landing: soft-body recordings with thrust logs, plumes, thruster HUD,
  and quick rigid-twin replays of trajectories from `experiments/`.
- `install` — motion-design piece: the cold-gas kit is installed on the car (hero nozzle with an internal
  fly-through, then the other nine). `nohud`: clean plates for a compositor.
- `opening`, `slam` — shots of the film 直到用计算机模拟 (`~/Downloads/直到用计算机模拟`, segments 00 / 01): `opening`
  `?shot=ghosts|recon|drive` (data `public/opening/ghosts.json`), `slam` (recording `public/slam/`). Their headers give
  the exact render commands; the film's compositors read the renders, and `film.marks(T)` (screen points per frame,
  `tools/film/film.mjs marks=1`) for overlays.

Film renders are used by finished segments of that film: a change to a program, `src/render/**`, `src/scene/**` or
`src/playback/**` shows up the next time a segment is re-rendered. Render a few frames before and after
(`tools/film/frame.mjs`) and compare. Frames are not bit-exact between two renders of the same code (accumulation
noise: a mean of ~0.2 / 255, < 0.3 % of pixels over 8 / 255).

## Creative shots

Shots are pure functions `cam(u, t)` (film progress, sim time) so realtime and offline renders agree. Reusable
moves live in `src/cinema/shots.js` (`lockOff`, `dolly`, `orbit`, `track`, `fly` along a spline, `blend`,
`withShake`), easing in `ease.js`, key moments of a launch in `moments.js`. Poses may set `roll` (dutch angle),
`focus` / `aperture` (depth of field in film renders) and `shake`. Programs keep bespoke shots in their folder;
general ones get promoted to the library.

## Experiments (tasks × algorithms)

Tasks are declarative (`spec/tasks/*.json`), controllers implement one interface in both languages, and the
benchmark runner writes per-episode metrics plus trajectories under `experiments/runs/` (git-ignored). A run is
reproducible from (spec hash, task, controller config, seed). See `experiments/README.md`.

## Known gaps (visible on film)

- The soft body's springs are unloaded: the car sits 5-11 cm below the GLB's ride height and the tyres sit up to
  ~8 cm inside the wheel arches, visible in close-ups (opt-in fix `level=1`, not adopted: `docs/rounds.md`).
- `renderFilmFrame` shutter samples straddle shot cuts: one blended frame per cut in a director edit.
- Soft-body MPC flights keep 1-2 kN of bottom thrust after the touchdown; front control arms are the weak point at
  the flight connector strength; the kit car is yaw-unstable rolling free above ~9 m/s.
- MPC planning is not real time (cold first plan 6-7 s, then 140-190 ms per plan): fine offline, the soft body waits.
