# JetCar Mini — notes for agents

Soft-body crash sim of a MINI EV off a viaduct + cold-gas thruster kit, rigid twins (JS / Python) and MPC / PPO
controllers; every outcome renders as film (three.js). The film 直到用计算机模拟 (`~/Downloads/直到用计算机模拟`,
its `sim` is a symlink to this repo) renders its 3D shots here.

Read only what the task needs:
- layers, contracts, programs, known gaps on film → `docs/ARCHITECTURE.md`
- frames, units, thrusters, propulsion and twin models, data formats → `spec/README.md` (numbered sections)
- running experiments, configs, controllers → `experiments/README.md`; results → `experiments/reports/<study>/README.md`
- why something is the way it is (dated round notes, may be stale) → `docs/rounds.md`

## Physics rules
- Numbers shared by JS and Python live in `spec/*.json`. JS reads them only via `src/core/spec.js`, Python only via
  `python/coldgas/spec.py`. Never hard-code a spec value. After changing the lattice (`src/car/carPrep.js`), the GLB,
  `settled.bin` or `spec/world.json`, run `npm run spec`.
- Nozzle order in `spec/propulsion.json` is the action-vector order everywhere.
- The JS and Python flight twins must agree to rounding: any change to `src/flight/**` physics needs the matching
  change in `python/coldgas/**`, `spec/README.md` and the parity goldens (`tests/parity/`).
- Soft body: changing the lattice (incl. triangle order) needs `npm run settle` and `npm run flight:settle`, then
  `npm run spec`, `npm run precompute:all` and `npm run flight:sim:all`, otherwise the car falls apart at the start.
  Render-only fixes go in `src/render/carRender.js`, not the physics input.
- Checks: `npm test` (generated spec up to date, JS/Python parity both ways, pytest).

## Film
- Programs: `src/programs/<id>/index.js` (`?program=<id>`); `program.film.render(T)` must be a pure function of film
  time T. Each program's header comment lists its query parameters (look-dev cameras `devcam=`, frozen time `devt=`).
- Renders need a dev server. Start your own on a free port with `FREEZE=1 npm run dev -- --port <p>` (FREEZE: no
  reload mid-render) and pass `port=<p>`. 5173 is the user's interactive server; never stop a server you did not start.
- `node tools/film/frame.mjs program=<id> port=<p> q=<query> T=1,2.5 out=<dir>`: stills to look at before a long
  render; `node tools/film/film.mjs ... out=<file.mp4>` (`marks=1`: screen points for a compositor). Relative `out=`
  paths are relative to the shell's cwd, also through the film's `sim` symlink.
- Recordings: `public/sims/*.sim` (git-ignored, regenerable: `npm run precompute:all`, `npm run flight:sim:all`,
  other flight runs per the tool lines in their reports); `flight` program `?run=<name>` plays `flight-<name>.sim`.

## Working here
- Other sessions may be working in this repo at the same time: keep to the files of your task, never revert changes
  you did not make. Commit your own work with a message that says what changed and why.
- No new npm dependencies. Style: plain ES modules, no TypeScript, dense code with comments that explain why; match
  the surrounding file.
