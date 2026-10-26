# experiments/ — 挑战任务 × 算法对比

- `configs/*.json` — experiment definitions (task, controllers with their settings, presets, seeds). Versioned.
- `runs/<experiment>/<stamp>/` — outputs, git-ignored: `config.json` (the config + spec hash + resolved seeds),
  `episodes.jsonl` (one line of metrics per episode), `summary.json` (aggregates per controller and per preset),
  `traj/<label>__<preset>__s<seed>.json` (`coldgas-traj@1`, see `spec/README.md` §7). PPO runs write
  `runs/ppo-<task>/<stamp>/` with `curve.json`, `policy.json` (`coldgas-policy@1`) and checkpoints.
  Every output records the spec hash; a run is reproducible from (spec hash, task, controller config, seed).

## Running

Python (venv at `python/.venv`, created with `uv venv --python 3.14 python/.venv && uv pip install --python
python/.venv/bin/python -e "python[test,rl]"`):

```sh
python/.venv/bin/python -m coldgas.bench experiments/configs/baseline-soft-landing.json [--workers 6] [--seeds 0:5]
python/.venv/bin/python -m coldgas.ppo --task hover-hold --steps 4000000 --envs 256
python/.venv/bin/python -m coldgas.bench experiments/configs/ppo-hover-eval.json     # evaluate an exported policy
python/.venv/bin/python -m coldgas.sanity       # physical sanity report of the twin
python/.venv/bin/python -m coldgas.throughput   # env steps/s, scalar and batched
python/.venv/bin/python -m pytest python/tests  # unit + parity tests (needs node for the JS side)
```

JS (no build step, Node >= 22):

```sh
node tools/flight/run.mjs task=escape-soft-landing controller=pd seed=0 preset=nose out=experiments/runs/js/nose.json
node tools/flight/run.mjs task=hover-hold controller=mlp policy=experiments/runs/ppo-hover-hold/<stamp>/policy.json
node tools/flight/run.mjs controller=replay traj=<coldgas-traj@1 file> task=... seed=... preset=...
node tools/flight/throughput.mjs
node tools/flight/bench.mjs experiments/configs/mppi-soft-landing.json [workers=16] [seeds=1:11] [only=mppi-soft]
                               # same config schema / outputs as coldgas.bench, episodes on worker_threads,
                               # writes runs/<name>-js/<stamp>/
node tests/parity/check.mjs    # JS re-runs the Python goldens (+ compares both golden sets)
node tests/parity/make.mjs     # rewrite the JS goldens;  python -m coldgas.parity make|check for the other side
```

## Config schema (`coldgas.bench`)

```jsonc
{
  "name": "baseline-soft-landing",          // output folder under runs/ (default: file name)
  "description": "...",                     // free text
  "task": "escape-soft-landing",            // spec/tasks/<id>.json (extends resolved)
  "controllers": [                          // each: registry name + settings (+ optional "label", default = name)
    { "name": "zero" },
    { "name": "pd", "kv": 8.0 },
    { "name": "mppi", "samples": 96, "horizon": 1.5 },
    { "name": "mlp", "label": "ppo", "policy": "runs/ppo-hover-hold/<stamp>/policy.json" }  // path: config dir or repo root
  ],
  "presets": ["flat", "nose"],              // launch presets to force; null = let the task draw them from the seed
  "seeds": { "start": 0, "count": 3 },      // or an explicit list [0, 1, 2]
  "record": "all",                          // trajectories: "all" | "first" (first seed per preset) | "none"
  "workers": 6                              // processes (episodes are independent)
}
```

Episodes are (controller × preset × seed); the task's randomisation (speed, angle, kick strength, …) still applies
per seed (spec/README.md §8.2). `summary.json` holds success / hull-strike rates, termination reasons, mean /
median / min / max of the touchdown metrics, gas, return and per-preset success; the runner prints a markdown table.

## Controller registry

Same names, settings and defaults in JS (`src/flight/controllers/index.js`) and Python (`coldgas.controllers`);
defaults in `src/flight/controllers/defaults.json`. Interface: `reset({task, seed})`, `act(FlightState, info) →
u_cmd[10] ∈ [0,1]` every 20 ms, FlightState fields only (so they also run on a state estimated from the soft body).

| name | languages | settings | what |
|---|---|---|---|
| `zero` | JS, Py | – | no thrust (the original crash) |
| `pd` | JS, Py (bit-identical) | `att_kp att_kd td_speed flare_height brake_frac kv tilt_cut hover_kh hover_vmax` | rights the car with differential bottom thrust (min-norm allocation), free-falls and then follows the braking profile `v_ref(h) = −√(td² + 2·brake_frac·a_max·(h − flare)⁺)` with feed-forward; hover tasks hold the target height, then land |
| `replay` | JS, Py | `traj` (object) or `file` | plays `u_cmd` of a `coldgas-traj@1` |
| `mlp` | JS, Py (bit-identical) | `policy` (object) or `file` | runs a `coldgas-policy@1` (PPO export) |
| `mppi` | JS, Py (bit-identical) | `samples horizon knot replan sigma temperature plan_dt base seed w_* v_ok curb_*` + soft-body options `w_slip w_td_slip w_gnd_slip w_gnd_yaw curb_steps curb_margin bottom_gate base_pd` | sampling MPC (`src/flight/controllers/mppi.js` = `coldgas/controllers/mppi.py`): K noisy residual sequences on top of `pd`, rolled out on a twin loaded from the FlightState, softmax-weighted; settings / cost documented in `defaults.json` `mppi`. JS: ~2.5 s per escape episode (Python ~18 s); runs closed loop on the soft body |
| `mpc` | JS only | `horizon knot knot_growth interp replan plan_dt mixer umax channel_scale noise_decay coast ground_cut init first_passes optimizer objective` + optimizer blocks `mppi cem grad hybrid` | pure MPC (`src/flight/mpc/`), no base pilot: an open-loop command schedule re-optimised every `replan` steps from the FlightState; optimizers `mppi` / `cem` (sampling), `grad` (finite-difference projected descent), `hybrid` (stages); objectives `land` / `roll` / `gas`. Defaults in `src/flight/mpc/defaults.json`. ~16 s per escape episode (cem); runs closed loop on the soft body (`controller='mpc:{...}'`) |

## Pure MPC (`mpc`, JS only)

`src/flight/mpc/` plans raw valve commands without any base pilot (the residual MPPI above needs pd for attitude).
Per plan, an optimizer improves the command schedule θ (warm-started from the previous plan, shifted in time) on a
rigid planning twin loaded from the current FlightState; the first part of it is executed until the next plan.

- **Schedule** (`param.js`): knots every `knot` s (growing by `knot_growth`), `interp` `linear` (default) or `zoh`;
  `mixer` `modes` (default: 7 wrench channels, bottom heave / pitch / roll, rear surge / pitch / yaw, side sway, from
  the min-norm allocation of each nozzle group about the full-tank COM) or `raw` (10 valves); `umax` ceiling per
  group; `coast` s of forced free fall after arming (plan and execution); `ground_cut` s after the touchdown from which
  the bottom valves stay closed (plan and execution; opt-in, see the barrel-roll report's 2026-10-21 note); `init` the
  cold-start schedule.
- **Optimizers** (`optim/`): `mppi` (softmax mean), `cem` (elite refit; default), `grad` (finite differences on
  the rollout cost, forward / central / SPSA, projected normalised descent with backtracking), `hybrid` (stages on
  the same θ, e.g. mppi → grad). Sampling noise is AR(1)-smooth along the knots (`rho`), scaled per channel
  (`channel_scale`) and per knot (`noise_decay`); means are updated on unclipped samples (no bias at the bounds).
  More optimizers: `optim/extra.js`. Every rollout is counted (`ctl.evals`; bench `evals`), for equal-budget
  comparisons.
- **Objectives** (`objectives/`): `land` (soft landing; terms in its header), `roll` and `gas` (owned by the two
  experiment streams of 2026-10-20 evening, see `docs/ARCHITECTURE.md`). Interface: header of `objectives/land.js`.
- **Analysis hooks**: `ctl.trace` (per plan t / cost / rollouts / ms, with `trace: true`), `ctl.predict(state)`
  (the current plan's predicted rollout), `ctl.theta` / `ctl.param` (the schedule itself).

What made it work (twin, escape-soft-landing): linear interpolation (zoh schedules: 33 %), much stronger attitude
terms than the residual MPPI's (`w_td_tilt` 400, `w_tilt` 3, `w_gnd_tilt` 20), an end-in-the-air penalty above any
landing (`w_air` 50 + 20/m, otherwise the plan hovers), small exploration on the torque channels (open-loop torque
noise random-walks the attitude), and pricing the touchdown before the crash in the same step (else every crashing
sample costs the same and the one with the least gas wins).

`mpc-baseline` (6 presets × seeds 1-3, defaults unless noted, spec of 2026-10-21: bottom pairs at z 0.34 / −1.16,
run `mpc-baseline-js/20261021-003141`; in brackets the 2026-10-20 17:00 layout, pairs at 0.55 / −0.95):

| controller | n | success | hull strike | vy_td | tilt_td | gas kg | return | corner | flat | nose | rear | roof | side | s/ep |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pd | 18 | 78% (72) | 22% | 0.80 | 0.01 | 46.55 (47.56) | 3.32 | 0% | 100% | 100% | 100% | 67% | 100% | 0.0 |
| mpc-cem | 18 | 100% (100) | 0% | 1.14 | 1.42 | 46.30 (46.13) | 5.58 | 100% | 100% | 100% | 100% | 100% | 100% | 15.4 |
| mpc-mppi | 18 | 100% (94) | 0% | 1.14 | 1.79 | 43.27 (47.14) | 5.79 | 100% | 100% | 100% | 100% | 100% | 100% | 14.7 |
| mpc-grad (2 iters) | 18 | 44% (33) | 6% | 2.13 | 3.64 | 39.43 (40.62) | -1.39 | 67% | 0% | 67% | 67% | 33% | 33% | 11.5 |
| mpc-hybrid (mppi 2 → grad 1) | 18 | 100% (94) | 0% | 1.15 | 2.17 | 44.38 (46.63) | 5.92 | 100% | 100% | 100% | 100% | 100% | 100% | 17.4 |
| mpc-cem, zoh | 18 | 28% (33) | 33% | 1.37 | 11.25 | 93.44 (107.63) | -11.28 | 33% | 0% | 0% | 33% | 33% | 67% | 10.4 |

```sh
node tools/flight/bench.mjs experiments/configs/mpc-baseline.json workers=16
node tools/flight/run.mjs task=escape-soft-landing controller='{"name":"mpc","optimizer":"hybrid"}' seed=1 preset=nose
node tools/sim/flight.mjs preset=flat controller='mpc:{}' out=x dur=5 outDir=experiments/runs/soft-mpc   # soft body
```

On the soft body (smoke test, flat, 5 s, `experiments/runs/soft-mpc/flight-mpc-flat-smoke.json`) the defaults touch
down at 1.08 m/s, 1.7°, 37 kg, but strike the hull 0.27 s later: the soft body's rear bottoms out, the model gap
the residual MPPI's soft-body settings work around (`bottom_gate`, slip / ground-yaw terms); not tuned yet.

Reports for the user live in `experiments/reports/<topic>/` (Chinese, versioned): README.md with method, setup,
results and conclusions, figures as SVG (no plotting dependency), every number traceable to a config and a run
stamp under `experiments/runs/`.

## MPPI on the soft body (closed loop)

The JS `mppi` runs inside the soft-body loop: `ThrusterRig` calls `act()` every 20 ms with its lattice state
estimate, MPPI loads it into a rigid planning twin (`RigidTwin.loadState`, `plan_dt` 2 ms) and replans every
`replan` steps. Cost: 96 samples x 1.5 s ≈ 33 ms per plan, ~3.2 s per 6.5 s flight (96 plans), i.e. ~2 % of the
~150-170 s soft-body run (ap: 146 s, mppi: 147 s side by side); no worker threads needed.

```sh
C='mppi:{"samples":96,"horizon":1.5,"knot":0.15,"replan":3,"curb_lookahead":2,"curb_steps":8,"curb_margin":1.5,"w_td_tilt":400,"w_td_rate":10,"w_slip":0.3,"w_td_slip":10,"w_gnd_slip":5,"w_gnd_yaw":1,"bottom_gate":3,"base_pd":{"td_speed":0.5}}'
node tools/sim/flight.mjs preset=nose controller="$C" out=mppi-nose dur=6.5     # -> public/sims/flight-mppi-nose.sim
node tools/sim/flight.mjs preset=roof controller="$C" out=mppi-roof dur=6.5
node tools/sim/flight.mjs preset=roof seed=1 controller="$C" out=x dur=6.5 outDir=experiments/runs/soft  # scratch
```

`outDir=` keeps scratch runs out of `public/sims` (writes `.sim` + `.json` with `meta.flight`); registry controllers
given as JSON are recorded as their short name with the settings in `meta.flight.controllerConfig`, MPPI's plan
count / time in `meta.flight.controllerStats`. Soft-body runs are deterministic for (preset, seed, controller).

Why the extra settings (all opt-in, default = the original Python cost, implemented in both languages): with the
baseline settings MPPI lands softly but strikes the hull 0.3-0.8 s later on both nose and roof. Causes found on the
soft body, none of which the twin reproduces: (1) landing with 10-30° slip makes the tyres roll the car onto a
sill (`w_slip`, `w_td_slip`); (2) a 1-2° nose-up touchdown, harmless on the twin, lands the 70 % rear-heavy car on
its rear axle and the rear floor (rest clearance 7 cm) bottoms out, where the twin's 400 kN/m bump stop leaves
3 cm to spare (`bottom_gate`: the residual on the bottom nozzles fades out below 3 m so pd flies a level touchdown;
`base_pd.td_speed` 0.5 m/s); (3) the rear-heavy car spins out ~1 s after landing (`w_gnd_slip`, `w_gnd_yaw`: the
closed loop pays for yaw damping with the rear jets); (4) the roof preset drifts across the road at 7-11 m/s and the
1 s COM probe sees the sidewalk corner only after the side jets can still stop it (`curb_lookahead` 2 s, 8 probes,
1.5 m footprint margin). A clearance-margin cost on the twin's hull points was tried and dropped (the planner
hovered instead of landing).

Soft body, escape-soft-landing criteria (`ThrusterRig.summary`), spec of 2026-10-21 (35 MPa fill, 8.7 kN bottom
nozzles, bottom pairs at z 0.34 / −1.16), lattice of 2026-10-20 16:20 (headlamp joint); gas in brackets: the
2026-10-20 17:00 layout (pairs at 0.55 / −0.95), same success / failure for every run. The replay replays
`baseline-soft-landing/20261021-003455` `mppi__nose__s2`. Rows marked † were not re-run (previous layout):

| run | controller | success | vy_td m/s | tilt_td ° | rate_td rad/s | max tilt in hold ° | hull strike | gas kg | parts off |
|---|---|---|---|---|---|---|---|---|---|
| nose | ap (`flight-soft`) | ✓ | 0.92 | 0.04 | 0.04 | 1.4 | – | 65.1 (61.4) | 0 |
| nose | pd (`flight-pd-nose`) | × | 0.80 | 0.10 | 0.16 | 49.3 | 3.64 s | 45.6 (46.0) | 8 |
| nose | replay of Python MPPI (`flight-replay-mppi-nose`) | × | 6.35 | 48.5 | 0.81 | 120.0 | 2.36 s | 57.4 (79.5) | 11 |
| nose | mppi, baseline settings † | × | 0.76 | 2.0 | 0.29 | 5.3 | 3.57 s | 60.0 | 11 |
| nose | **mppi-soft** (`flight-mppi-nose`) | ✓ | 0.56 | 1.59 | 0.22 | 1.6 | – | 60.9 (74.1) | 0 |
| roof | ap (`flight-recover`) | ✓ | 0.95 | 0.05 | 0.12 | 1.3 | – | 81.0 (80.8) | 0 |
| roof | pd (`flight-pd-roof`) | × | 0.80 | 0.09 | 0.14 | 32.6 | 3.63 s | 48.3 (49.5) | 11 |
| roof | mppi, baseline settings † | × | 0.53 | 7.2 | 0.33 | 7.0 | 3.76 s | 68.7 | 5 |
| roof | **mppi-soft** (`flight-mppi-roof`) | ✓ | 0.51 | 0.64 | 0.28 | 1.6 | – | 63.7 (62.6) | 0 |

mppi-soft on further soft-body cases (outDir runs, previous layout): nose seed 1 ✓ 0.77 m/s, nose seed 2 ✓ 0.47, roof seed 1 ✓ 0.84,
roof seed 2 ✓ 0.54, flat ✓ 0.77, rear ✓ 0.46, side ✓ 0.52, corner ✓ 0.63: 8/8 + the two recordings above (the
corner preset struck its hull 0.27 s after touchdown on the previous spec). After the hold, roof s1 / flat /
nose s2 / side still roll into something 1.2 / 2.6 / 2.9 / 3.3 s after touchdown (4-8 parts off in the first three).

Twin (`mppi-soft-landing`, 6 presets x seeds 1..10, JS and Python give identical episodes; run
`mppi-soft-landing-js/20261021-003423`):

| controller | n | success | hull strike | vy_td | tilt_td | gas kg | return | corner | flat | nose | rear | roof | side | s/ep JS / Py |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| pd | 60 | 57% | 43% | 0.80 | 0.01 | 46.43 | -0.87 | 0% | 70% | 60% | 100% | 40% | 70% | 0.0 / 1.1 |
| mppi | 60 | 93% | 0% | 0.94 | 4.19 | 51.31 | 6.13 | 90% | 100% | 80% | 100% | 90% | 100% | 2.2 / 24 |
| mppi-soft | 60 | 98% | 2% | 0.72 | 1.17 | 45.81 | 7.14 | 90% | 100% | 100% | 100% | 100% | 100% | 2.0 / 33 |

(Spec of 2026-10-21: bottom pairs at z 0.34 / −1.16; on the 2026-10-20 17:00 layout (pairs at 0.55 / −0.95) pd /
mppi / mppi-soft scored 52 / 93 / 97 % at 47.5 / 54.6 / 49.7 kg, on the spec before it (30 MPa, 6.9 kN) 42 / 90 / 93 %. With 3 seeds per preset (and seeds 0 and 1 drawing the same initial condition) the
spread between noise streams is large (`baseline-soft-landing` once gave mppi 18/18 with numpy's normal RNG, 15/18
with the shared stream), hence the 10-seed config. Python s/ep measured with 16 workers on 18 cores, previous spec.)

## Twin touchdown calibration (2026-10-21)

Soft-body landing experiments → fitted twin `coldgas-rigid@2` → failure criteria in the MPC → soft-body acceptance;
report `experiments/reports/touchdown-calibration/README.md`. Tools: `tools/flight/touchdown/` (drop rig + design,
README there), `tools/flight/calib/` (`fit.mjs` CMA-ES fit, `validate.mjs` held-out errors + thresholds, `accept.mjs`
soft-body closed loop with Wilson intervals, `root=` runs another source tree). Twin configs
`experiments/configs/touchdown-accept-{land,roll}.json`; runs `experiments/runs/touchdown-doe/`, `calib/`, `accept/`.

## Brakes and steering (2026-10-21)

Task `escape-land-stop` (land softly, then stop within 8 s of the touchdown without a wheel on a kerb; spec/README.md
§8, §5.13). The MPC plans the two drive channels with `drive: true` (zero until the touchdown) and the land objective's
drive terms; config `experiments/configs/drive-stop.json` (`mpc-drive` vs `mpc-nodrive`). Twin: 17/18 vs 0/18. Soft body:
`tools/flight/calib/accept.mjs sets=stop,rollstop` (24/26 stop cleanly). Drive tests of the soft body and the joint twin
fit: `tools/flight/drive/`, report `experiments/reports/drive-brakes-steering`.

## Two costs: lower slowly vs suicide burn (2026-10-21)

The FALL script's two experiments (report `experiments/reports/gas-two-costs`, config `gas-two-costs.json`, runner
`tools/flight/gas/twocost.mjs`): the default `mpc` with the `land` objective at `w_gas` 0 / 0.3 / 1, each from the
all-closed and from a hover cold start (`init` heave 0.33), every flight scored afterwards by each cost on its realized
trajectory (`scoreTraj`: commands replayed on a fresh twin, objective `step()` per control step from arming), and per
cost the cheaper start kept. From one start the start decides the strategy (all-closed: coast + late burn even at
`w_gas` 0, 53.6 kg; hover: lowered slowly even at 1 / kg); with both starts the cost does (upright presets: w_gas 0
keeps the slow descent 25/40, 73.8 kg; 1 / kg keeps the late burn 34/40, 43.2 kg, ignition 0.71 s before touchdown).
v2 (`gas-two-costs-v2.json`, matching the revised script: cost 1 also prices descent speed, `w_sink` 0.02; cost 2 + 2 / kg; three
cold starts; feasible-first selection): cost 1 picks the slow descent whenever it lands in time (25/30, 78.5 kg), cost 2 the late burn
(36/39, 43.2 kg); the follow-up tests (failure criteria off, gradient optimizers, ppo's LTV-MPC, a 10x cold plan) are in the report.
New opt-in MPC settings (default off, old runs bit-identical): `starts` (multi-start cold plan; not used for the
result, the cold plan's cost is dominated by feasibility) and the land objective's `w_sink` / `sink_ok`.

## Results so far

`baseline-soft-landing` (escape-soft-landing, 6 presets × 3 seeds, spec 2026-10-21: bottom pairs at z 0.34 / −1.16,
run `baseline-soft-landing/20261021-003455`; in brackets the 2026-10-20 17:00 layout):

| controller | n | success | hull strike | vy_td mean (m/s) | tilt_td mean (°) | gas (kg) | return | s/episode (Py, 8 workers) |
|---|---|---|---|---|---|---|---|---|
| zero | 18 | 0 % | 100 % | 12.66 | 150.3 | 0.0 | −32.97 | 0.5 |
| pd | 18 | 78 % (72) | 22 % | 0.80 | 0.01 | 47.0 (48.0) | 3.27 | 1.0 |
| mppi | 18 | 83 % (89) | 6 % | 0.91 | 3.48 | 49.7 (55.2) | 4.83 | 16.8 |

pd fails on every `corner` seed and on one `roof` seed: its slow powered descent carries the car onto the sidewalk
side of the crossing and it rolls into a kerb within the 1 s upright hold; MPPI sees the kerb in its rollouts and
steers / times the landing into the kerb-free crossing (its failures: nose s2 / corner s1 touch down at 7.6° / 8.9° tilt,
corner s0 strikes its hull; on the previous layout nose s2 hovered into the timeout and roof
s0 tilted past the limit: with 3 seeds that is noise, the 10-seed `mppi-soft-landing` above is the comparison). `flight-replay-mppi-nose` replays `mppi__nose__s2` of this run.

`pd-precision-pad`, `pd-tumble-recovery` (12 task-drawn seeds each): pd 0 % on the pad task (it does not aim: its
long powered descent lands 20–34 m past the pad), 25 % on tumble-recovery (it rights every car within ~0.5 m of
leaving the deck, then rolls into a kerb); zero thrust 0 % on both.

The PPO results below were trained and evaluated on the previous spec (30 MPa, 6.9 kN bottom nozzles); the
policies carry that spec hash and need retraining on the current one.

`ppo-hover-hold` (`python -m coldgas.ppo --task hover-hold --steps 3000000 --envs 256 --bc-steps 204800 --log-std -2
--lr 1e-4`): from scratch PPO (4–5 M steps) never succeeded (first it learned to hop and land early, which the task
did not penalise until `terminal_timeout` was extended to `landed_early`, then it crashed every episode); warm-started
by imitating pd (MSE 4e-4) it reached 98.5–99.5 % training success within 25–50 updates (0.2–0.4 M steps) and then
destabilised (success back to 0 by update ~100: every episode `landed_early`), so the run keeps the best checkpoint
(`policy-best.json`, copied to `policy.json`). `ppo-hover-eval` (20 held-out seeds, deterministic): zero 0 %, pd 100 %
(0.80 m/s, 83.6 kg, return 0.32), PPO 100 % (0.88 m/s, 80.8 kg, return 0.80). The same policy file in the JS `mlp`
controller (`node tools/flight/run.mjs task=hover-hold controller=mlp policy=...`) gives a bit-identical trajectory.

`ppo-escape-soft-landing` (same recipe, lr 3e-5, log std −2.3, stopped at 1 M steps): not solved. The cloned policy
keeps the car level (touchdown tilt ~1°) but starts braking too late (touchdown ~5.5 m/s, hull strike on every
preset); the sharp "free-fall, then brake" switch of pd is not captured by the 64-64 tanh MLP. Next steps: longer
runs, a time-to-impact feature in the observation, or a residual policy on pd.

## 正式 demo

只维护 `configs/nonconvex-stop.json`（落地刹停.非凸）和 `configs/nonconvex-pad.json`
（定点降落.非凸）作为默认展示案例。`npm run demos:generate` 可重新生成全部录制；
`only=nonconvex-stop` 或 `only=nonconvex-pad` 可限定单例。软体录制全部本地保存、Git-ignored。
指标见 `reports/demos/`；旧双层二开配置与历史记录移入 `archive/dual-development/`。
其他既有研究配置仍可按原入口使用，但不属于正式 demo 列表。
