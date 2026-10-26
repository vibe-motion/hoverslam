# spec/ — 跨语言唯一事实源 (single source of truth)

Everything that the browser app, the soft-body solver, the **JS flight twin** (`src/flight/`) and the
**Python flight twin** (`python/coldgas/`) must agree on lives here as JSON. Code never copies these numbers.
JS reads them only through `src/core/spec.js`; Python only through `python/coldgas/spec.py`.

| file | content | edited by |
|---|---|---|
| `world.json` | viaduct/street dimensions, gravity, air | hand |
| `launch-presets.json` | the six launch presets + kerb-kick timing | hand |
| `propulsion.json` | 10 nozzles (order = action order), COPV bank, regulator, valves, control rate, kit masses | hand |
| `vehicle.json` | rigid-twin contact/aero/integrator parameters | hand (fitted values: `_calibration`) |
| `drive.json` | brakes (ABS) and steering: the two action channels after the nozzles (§5.13) | hand |
| `tasks/*.json` | challenge tasks (initial conditions, success criteria, reward weights, metrics) | hand |
| `generated/vehicle-massprops.json` | bare-car mass/COM/inertia, wheels, settled COM — from the soft-body lattice | `npm run spec` |
| `generated/colliders.json` | static colliders of the set | `npm run spec` |

`npm run spec:check` fails if a generated file is stale (run it after touching `src/car/carPrep.js`, the GLB,
`settled.bin` or `world.json`). Keys starting with `_` are documentation only and carry no semantics.

## 1. Frames and units

SI units throughout (m, s, kg, N, Pa, K, rad). Angles in JSON are degrees only where the key says `_deg`.

- **World W**: +Y up, gravity `(0, -world.gravity, 0)`. The viaduct runs along X; deck road surface at
  `y = deck.top`; the car drives towards −X on the +Z carriageway and leaves through the parapet gap at
  `z = deck.halfWidth`. The street is the plane `y = 0` plus the boxes in `generated/colliders.json`.
- **Body B** (car body frame = soft-body lattice rest frame): +X right, +Y up, +Z forward, origin on the ground
  under the car centre (rest pose, wheels touching y = 0). All positions in `propulsion.json` / `vehicle.json`
  are in B.
- **Orientation** `q = [x, y, z, w]` (three.js order) rotates B → W: `v_W = R(q) v_B`.
- **Rigid state**: `p` = world position of the **current** centre of mass, `v` = its world velocity,
  `w` = angular velocity in **body** axes, `comB` = centre of mass in B (moves as gas is used).
  A body point `x_B` is at `x_W = p + R(q) (x_B − comB)`.
- Flat state vector (trajectories, batched code): `[p(3), q(4), v(3), w(3)]`, length 13.

## 2. Thrusters

`propulsion.nozzles` order **is** the action order everywhere (controllers, valve/thrust logs, UI, recordings); an action
may append the `ND = 2` drive channels of `spec/drive.json` `channels` (index `NU` brake, `NU + 1` steer, §5.13):

| idx | id | group | where |
|---|---|---|---|
| 0–3 | B_FL B_FR B_RL B_RR | bottom | under the floor pan, front pair z = 0.41, rear pair z = −1.09 (heave / roll / pitch): centroid −0.34 = the kit's COM after ~45 kg (−0.36 full, −0.33 after 90 kg, it moves forward as the bottles empty), so pitch-free heave throttles one pair only slightly (front 0.95 full, rear 0.96 after 90 kg): 33.1–34.8 kN over the burn (all four open 34.9 kN), thrust/weight 3.0–3.4. History: until 2026-10-20 the pairs sat at 0.55 / −0.95 (centroid −0.20, 26.6 kN pitch-free, T/W 2.36); on 2026-10-21 first 0.34 / −1.16, then the bottle rack moved one slot (0.26 m) forward, to the front seat backs (columns z −0.52 / −0.78 / −1.04; rear axle 810 → 769 kg full) and the pairs followed the COM. |
| 4–7 | R_UL R_UR R_LL R_LR | rear | 2×2 through the rear bumper (surge; yaw by left/right, pitch by upper/lower) |
| 8–9 | S_L S_R | side | on the sills at the COM (sway) |

The kit also brings stiffer rear coil-overs for the soft body (`propulsion.suspension.rear_spring_scale` = 2 on the
lattice's spring rate, applied by `ThrusterRig.applyMass`): the loaded rear axle (66 % of the car) then sags like the
front (62 / 67 mm along the springs, settled) instead of resting on its bump stops. With them come heavy-duty rear
control arms (`rear_arm_give_scale` = 2 on the give of the rear wheels' arm connectors, on top of the run's connector
factor `joints`): the stiff rear springs snapped the stock rear-left arms on the `flat` launch. Crash presets (bare car)
keep their springs and arms; the twins' suspension is their own (§5).

Gas leaves along `exhaust`; thrust acts along `−exhaust` at the `exit` point. `mount` is where the flange sits on
the skin (motion design, soft-body anchoring). Throat diameter and expansion ratio are primary; thrust is derived.

## 3. Propulsion model v1 (reference: `src/flight/propulsion.js`)

Per step of length `dt` with commanded openings `u_cmd[i]` (held over the step):

1. **Valves** (proportional, modelled as a variable effective throat area): `c = clamp(u_cmd, 0, 1)`, `c = 0` if
   `u_cmd < valve.deadband`; `τ = tau_open` if `c > s` else `tau_close`; `s ← s + (c − s)(1 − e^(−dt/τ))`.
2. **Feed pressure**: `p_c = min(regulator.p_set, p_tank − regulator.dp_min)`; if `p_c ≤ 2 p_a` then `p_c = 0`
   (cannot choke: tank exhausted). `T_c = T_tank` (isenthalpic regulator, ideal gas).
3. **Nozzle i** (choked, isentropic, frozen once per nozzle: exit Mach `M_e` from `expansion_ratio` by 80-step
   bisection on [1, 50]): `ṁ = s A_t p_c √(γ/(R T_c)) (2/(γ+1))^((γ+1)/(2(γ−1)))`,
   `v_e = M_e √(γ R T_c / (1 + (γ−1)/2 M_e²))`, `p_e = p_c (1 + (γ−1)/2 M_e²)^(−γ/(γ−1))`,
   `F = max(0, ṁ v_e + s (p_e − p_a) A_e)` with `A_e = ε A_t`, `p_a = world.air.pressure`.
4. **Tank** (all bottles = one volume `V = Σ volume`): `m ← max(0, m − Σṁ dt)`, `ρ = m/V`,
   `p = K ρ^n` with `K = p0 / ρ0^n`, `ρ0 = p0/(R T0)`, `n = tank.polytropic_n`; `T = p/(ρ R)`.

Flows and thrust use the tank state at the **start** of the step. Nominal values at full tank: ≈8.7 kN (bottom),
≈4.9 kN (rear), ≈3.0 kN (side), Isp ≈ 65 s, 145.7 kg of air; the regulator drops out after ~75 % of the gas.

## 4. Vehicle mass properties

- `m_dry = lattice.mass + Σ bottle.dry_mass + Σ nozzle.mass + Σ hardware.mass`; bottles and gas are solid
  cylinders (bottle geometry, axis `axis`), nozzles and hardware are point masses at `mount` / `center`.
- Gas mass `m_g(t)` is split over the bottles in proportion to volume.
- `comB`, total mass and inertia (about `comB`, body axes) are recomposed whenever `m_g` changes (parallel-axis
  theorem from `generated/vehicle-massprops.json` `lattice.inertia`, which is about `lattice.com`).

## 5. Rigid-body twin (JS `src/flight/`, Python `python/coldgas/`) — owned by the flight-twin work

Model id `coldgas-rigid@3` (written into trajectories as `model`; @3 = @2 + brakes and steering, §5.13, + the slip-ratio
saturation κ and the sliding blend `tyre.slide` of §5.11: with zero drive, κ = 0 and slide = 0 it reproduces the @2
trajectories bit for bit, checked on all 29 @2 parity scenarios with the @2 values; the spec carries the @3 refit,
2026-10-21, `spec/vehicle.json` `_calibration`). Reference code: `src/flight/twin.js`, mirrored
expression by expression by `python/coldgas/twin.py` (numpy, N vehicles at once). The two agree **bit for bit**
(§5.9); any change here needs both files, the goldens (`tests/parity/`) and a new model id. Since 2026-10-21 the spec
carries the contact values fitted to soft-body landings (v2: the touchdown extensions of §5.11 switched on, see
`spec/vehicle.json` `_calibration`); the previous model `coldgas-rigid@1` (v1) is the explicit parameter override
`tests/parity/vehicle-v1.json` (§5.11), which reproduces its trajectories bit for bit.

**5.1 Constants.** `dt = vehicle.integrator.dt`, `n_sub = round(propulsion.control.dt / dt)`, `g = world.gravity`,
`k_a = 0.5 · air.density · aero.CdA` (per body axis). Wheels `j = 0..3`: centre `c_j` and radius `r_j` from
`generated/vehicle-massprops.json` (body frame, unloaded), suspension `k_s c_s travel k_b` (+ `c_b` = `bump_c`,
`k_b2` = `bump_k2`), `tyre_mu`, `crr`, `toe_compliance` from `vehicle.wheels` (and the per-axle values of `tyre`,
`wheel_spin`, `drive.brake.torque_max`), each a number or `[front, rear]`
(axle of wheel `j`: front if `c_j,z > 0`), so every wheel has its own values; static `toe` per wheel (4 values,
massprops wheel order FL FR RL RR). Hull points `H_j` = `hull.points` followed by
`hull.floor_points`, with `k_h c_h mu_h` = `hull.k c mu` for the first group and `floor_k floor_c floor_mu` (null =
`hull.k c mu`) for the second. Every contact parameter is taken from the twin's vehicle object (the spec, or the spec
with an override, §5.12) when the twin is built. Nozzle `i`: exit `e_i`, thrust direction
`d_i = −exhaust_i/|exhaust_i|`. Mass properties: §4 in the linear form of `massprops.js` (dry parts folded into
first/second moments about the body origin; `M = m_dry + m`, `c = (s_dry + m s_gas)/M`,
`I = J_dry + m J_gas − M(|c|²E − c cᵀ)`, `I⁻¹` by cofactors). Model constants (`spec/vehicle.json` `twin`, not
vehicle facts): `contact.v_reg` 0.1 m/s, `contact.wheel_min_h` 0.5, `contact.street_top_max` 1.0 m,
`placement.iterations` 30, `placement.fd_step` 1e-7, `limits.floor_y` −2 m, `limits.max_range` 400 m.

**5.2 World.** Ground plane `y = ground.y` (`ground.mu`) plus the boxes of `generated/colliders.json` with
`cψ = cos yaw`, `sψ = sin yaw`. A box is *deck* (name `deck`), *street* (top `c_y + e_y < street_top_max`:
sidewalks, median) or *other* (parapets, piers); the ground plane is street. Local coordinates of a world point:
`d = x − c`, `l = (cψ d_x − sψ d_z, d_y, sψ d_x + cψ d_z)`; *inside* means `|l_x| < e_x ∧ |l_y| < e_y ∧ |l_z| < e_z`.
*Street level* under `(x, z)`: the highest of `ground.y` and the tops of street boxes whose footprint
(`|l_x| < e_x ∧ |l_z| < e_z`) contains the point.

**5.3 State.** `p, q, v, ω_B` (§1), propulsion state (valves `s`, tank `m, T, p`), mass properties of the current
`m`, integer step counter `k` (`t = k·dt`, never accumulated), and flags: kick (waiting / active with `n_left` /
done), `left_deck`, `armed`, `airborne`, touchdown snapshot, first hull strike, peak specific force. Per wheel (§5.11):
spin `ω_j` (only with `wheel_spin.enabled`; not part of the 13-float state), last-step loads `N, f_l, f_t`, peaks of
`N` and `|f_l|`, lateral impulse since touchdown, first wheel overload. Drive (§5.13): brake pressure `β`, steer angle
`δ` (0 after reset), the applied drive commands `[b, s]` of the control step, first kerb hit.

**5.4 Control step.** `u = armed ? u_cmd[0..NU) : 0`, NaN → 0 (clamping to [0, 1] and the dead band are the propulsion
model's). Drive commands (§5.13; the action may be `NU` or `NA = NU + ND` long): `b = armed ? clamp(u_cmd[NU], 0, 1) : 0`,
`s = armed ? clamp(u_cmd[NU+1], −1, 1) : 0`, NaN and missing entries → 0. Run `n_sub` integrator steps with `u, b, s`
held, then the boundary update §5.8. Readers: `u` = applied valve commands (JS `twin.u`, Python `u[e]`, length NU),
`[b, s]` = JS `twin.ud`, Python `ud[e]`.

**5.5 Integrator step** (in this order; quantities without a prime are those of the step start):

1. `R = R(q)` (row-major, the expressions of `src/core/quat.js`).
2. *Kick* (§6): if waiting and `p_z > deck.halfWidth − kick.trigger`: start with `n_left = round(kick.dur/dt)`.
   While `n_left > 0`: `ω_B += Rᵀ Δω_W`, `n_left −= 1`, with `Δω_W = ((sp r_x + sr f_x)/n, sy/n, (sp r_z + sr f_z)/n)`,
   `[sp, sy, sr] = spin`, `n = round(kick.dur/dt)` (so the full kick is added exactly once).
3. *Propulsion*: `T = Propulsion.step(u, dt)` (§3; uses the tank state at the step start, leaves `m` at `m(t+dt)`).
   3b. *Drive* (§5.13): brake pressure lag, rate-limited steer angle, front-wheel headings (nothing while all 0).
4. *Forces* at `(p, q, v, ω_B)` (ω after the kick), COM `c`: `ω_W = R ω_B`; a contact point with world offset `r`
   from the COM moves with `v_P = v + ω_W × r`.
   - **Wheels** (only if the axle `a = R x̂` has `h = √(a_x² + a_z²) > wheel_min_h`): massless wheel, tyre point =
     lowest point of the wheel disc, `r_j = R(c_j − c) + r_j δ` with `δ = (a_y a_x/h, −h, a_y a_z/h)`. Support depth
     `d = ground.y − y_P`, then for every box the point is inside of with top-face depth `d_b = e_y − l_y ≤ r_j` and
     `d_b > d`: `d = d_b` (file order; the chosen surface gives `μ_s` and the class; a street box = *kerb*, §5.13). If `d > 0`:
     `N = k_s d − c_s v_P,y`; if `d > travel`, with `e = d − travel`: `N += k_b e`, then (only evaluated when some
     wheel has `c_b ≠ 0` or `k_b2 ≠ 0`) `N += k_b2 e e − c_b v_P,y`; if `N > 0`, with `ih = 1/h`:
     rolling speed `v_t = (v_x a_z − v_z a_x) ih` (along `t = (a_z, 0, −a_x) ih`, i.e. body-backward), lateral
     `v_l = (v_x a_x + v_z a_z) ih`, `μ_e = tyre_mu μ_s`; with `tyre.model` `coulomb`, no wheel spin and no toe (v1):
     `f_t = −(crr N) v_t / max(|v_t|, v_reg)`, `f_l = −(μ_e N) v_l / max(|v_l|, v_reg)`, otherwise §5.11 (also while
     steered, §5.13; brake in the spin update);
     `F = ((f_t a_z + f_l a_x) ih, N, (f_l a_z − f_t a_x) ih)` at the tyre point. Unloaded wheels (no axle, `d ≤ 0`
     or `N ≤ 0`): loads 0, spin `ω ← ω · (1 − dt · decay)` (then braked, §5.13). Loaded wheels then update the
     diagnostics of §5.11 (kerb first, §5.13).
   - **Hull points** `r_j = R(H_j − c)`. Ground: if `d = ground.y − y_P > 0`: `N = k_h d − c_h v_P,y`; if `N > 0`:
     `φ = mu_h ground.mu N / max(√(v_x² + v_z²), v_reg)`, `F += (−φ v_x, N, −φ v_z)`. Every box the point is inside of:
     penetrations `π_a = e_a − |l_a|`; face y if `π_y ≤ π_x ∧ π_y ≤ π_z`, else x if `π_x ≤ π_z`, else z; world normal
     `(0, σ_y, 0)`, `(cψ σ_x, 0, −sψ σ_x)` or `(sψ σ_z, 0, cψ σ_z)` with `σ = l ≥ 0 ? 1 : −1`; `d = π_face`,
     `v_n = v_P·n`, `N = k_h d − c_h v_n`; if `N > 0`: `v_t = v_P − v_n n`,
     `φ = mu_h μ_box N / max(|v_t|, v_reg)`, `F += N n − φ v_t`.
   - Contact sums `F_C = Σ F_j`, `τ_C = Σ r_j × F_j` (world, about the COM): wheels 0..3 then hull points in file
     order, sequential from 0. Thrust (body) `F_T = Σ T_i d_i`, `τ_T = Σ (e_i − c) × (T_i d_i)` in nozzle order.
     Drag (body, at the COM) `v_B = Rᵀ v`, `F_A,a = −k_a,a |v_B,a| v_B,a`.
     Totals `F = F_C + R(F_T + F_A)`, `τ_B = τ_T + Rᵀ τ_C`.
5. *Integrate*: `v_x += dt F_x/M`, `v_y += dt (F_y/M − g)`, `v_z += dt F_z/M`; with
   `f(ω) = I⁻¹(τ_B − ω × Iω)`: `ω_½ = ω + (dt/2) f(ω)`, `ω' = ω + dt f(ω_½)` (midpoint: the free tumble keeps
   energy and |L| to ~1e-9 over 6 s); `p' = p + dt v'`; `q' = normalize(q + (dt/2) q ⊗ (ω', 0))`.
6. *Gas used*: mass properties of `m(t+dt)`; `Δc = c' − c`; with `R' = R(q')`: `p' += R' Δc`, `v' += R'(ω' × Δc)`
   (the body frame stays put, the COM moves inside it; `ω` is continuous since the exhaust carries its own
   angular momentum and the jet force is already the thrust). Skipping this when `m` did not change is exact.
7. `k += 1`. Events and flags of the step carry `t = k·dt` of the step start.

**5.6 Contact classes and flags** (per integrator step, ORed over a control step): *any* (a wheel with `d > 0` or a
hull point inside any surface), *street* (wheel on a street support, hull point in the ground or a street box),
*deck*, *other* (parapet/pier), *hull strike* (hull point in the ground or any non-deck box; the first one is
recorded with the point index). **Touchdown** = first integrator step with street contact while `airborne`; its
snapshot is `(t, p, q, v, ω)` of that step's start. Peak specific force `max |F|/M` (excludes gravity).
*Wheel overload*: §5.11 (event `wheel_overload`). *Kerb*: a loaded wheel on a street box (§5.13, event `kerb`).

**5.7 Placement.** *launch*: §6 with `cos θ`, `sin θ` from `fmath`, COM horizontal at `(x_c, z_c)`, heading
`q_h` = yaw with `cos φ = sin θ`, `sin φ = −cos θ` (half-angle without trig: if `cos > −0.5`:
`w = √((1+cos)/2)`, `y = sin/(2w)`, else `y = ±√((1−cos)/2)` with the sign of `sin`, `w = sin/(2y)`), `v = s f`,
`ω = 0`, full tank, kick waiting, `left_deck = false`. *rest*: COM above the task's `(x, z)`, yaw `ψ` = heading
about +Y (forward `(sin ψ, 0, cos ψ)`), at rest, `left_deck = true`. Both then solve the **static suspension
equilibrium** on the support plane `y_s` (highest box top or ground under the COM): unknowns `(y_c, a, b)`,
`q = normalize((a, 0, b, 1) ⊗ q_h)`, `p = (x_c, y_c, z_c)`; residual `(Σ N_j − M g, −Σ r_z N_j, Σ r_x N_j)` with
the wheel geometry of 5.5 and the static part of the step's wheel force, per wheel, for `d > 0`: `N_j = k_s d`, and
if `e = d − travel > 0`: `N_j += k_b e`, then (as in 5.5) `N_j += k_b2 e e` (no damping/friction); Newton from
`(y_s + c_y − M g/(2 (k_s,front + k_s,rear)), 0, 0)` (= `M g/(4 k_s)` exactly for equal axles) with
forward-difference Jacobian (`fd_step`), solved by cofactors, exactly `iterations` times (residual ~1e-11 N).
Wheel spin (§5.11) after placement: *launch*/*rest* rolling-matched, *state* `init.wheel_spin` (4 values, rad/s)
if given, else `spin_ratio_air` × rolling-matched. *state*: explicit `p, q, v, ω`, optional `gas` (tank on the polytrope:
`p = K ρⁿ`, `T = p/(ρR)`), `valve`, `leftDeck` (default true), `kick.spin_w` (world, rad/s).

**5.8 Control-step boundary** (after reset and after every control step, in this order): `left_deck` (launch:
first boundary with `p_z > deck.halfWidth`, event `edge`); `airborne` (first boundary with `k > 0`, `left_deck`
and no contact at all during the step just run); `armed` (task `arm`: `'start'` at once, `'edge'` once
`left_deck`; never disarmed); `phase` = `'deck'` before `left_deck`; at `k = 0` `'ground'` for rest and `'air'` for
an explicit state; else `'air'` without contact in the last step, `'deck'` if it touched the deck, otherwise `'ground'`.
Events: `kick`, `edge`, `armed`, `airborne`, `touchdown` (with wheel bit mask), `hull_strike` (point index),
`regulator_dropout` (first step with `p_c < p_set`), `wheel_overload` (`wheel`, `quantity`: §5.11), `kerb` (`wheel`:
§5.13). The wheel peaks of
§5.11 restart at the `edge` boundary (the deck run is the launcher, not the experiment).

**5.9 Numerics and parity.** Float64, no fused multiply-add, sums strictly sequential in the stated order, and every
expression in the operand order of `twin.js` (numpy evaluates the same ufunc sequence lane by lane; masked lanes add
exact zeros; the broadphase only skips boxes that cannot contain a contact point, which never changes a sum).
Only `+ − × ÷ √` and `pow` reach the state: V8, CPython and numpy agree on `√` and `pow` (2 M samples checked); the
valve-lag and brake-lag `exp` constants agree; `sin/cos` (placement, steer angle; numpy lanes `fmath.sin_np/cos_np`) and
`acos/tanh` (rewards, policies) go through `src/flight/fmath.js` / `coldgas/fmath.py`, the fdlibm algorithms written out identically in both languages (V8's own
trig and Apple's libm disagree in the last bit for 3–18 % of inputs). Parity tolerance (relative to the largest
value of the component over the trajectory): **1e-9 on contact-free rows, 1e-6 on rows with contact** — penalty
contacts amplify a 1-ulp difference (e.g. a platform whose `pow` rounds differently) by orders of magnitude within a
second, so contact segments cannot promise more than that; measured on all 40 scenarios (incl. the `v2-*` overrides and
the `drive-*` / `land-stop-pd` drive scenarios, all columns incl. `drive`, `drive_cmd`):
**0 (bit-identical)**; events are compared with their payloads.
`round()` on exact halves differs (JS up, Python to even): no rounded quantity here sits on a half.

**5.10 Limitations** (by design, the soft body is the high-fidelity model): convex box edges only meet hull
points (the deck edge can pass between them); wheels only stand on box tops, and a loaded wheel's tyre point lies below
a 0.16 m sidewalk box, so a car rolling into a kerb hits it with its floor points first (hull strike; the wheel stands on
the kerb only once the hull has lifted the car, ~16 m/s at 60° to the kerb line, or when it lands on it); massless
wheels (the optional spin of §5.11 has no gyro and no reaction on the body beyond the tyre force; the brake acts
through the spin only, §5.13, no drive torque); parallel steer without Ackermann, self-aligning torque or kingpin
geometry; aero without moments or wind; no jet damping; the damper acts from the first millimetre of
penetration, so the normal load jumps to `c_s |v_y|` in the first contact step, before the spring has compressed (no
tyre compliance; v1: 13 kN at 5 m/s, v2 rear: 40 kN). v1 only: the bump stop is undamped beyond `c_s` (hard landings
bounce), the tyres are Coulomb across (a sideslip touchdown loads them to `μ N` at once), and with the kit the static
rear-axle load compresses the rear springs past `travel` (static compression 0.111 m rear / 0.055 m front: the rear
sits 1.4 mm on the bump stop, pitch 1.6° nose up), so a firm landing goes straight into the bump stops. The fitted v2
springs carry the static load within travel (0.075 m rear / 0.108 m front at full tank, pitch 0.9° nose up).

**5.11 Touchdown model extensions** (system identification against soft-body landings). Parameters in
`spec/vehicle.json`, per axle where noted. The spec carries the values fitted on 2026-10-21 (`coldgas-rigid@3`:
per-axle suspension with bump damping and progressive term, slip tyre with slip-ratio saturation and sliding blend,
wheel spin, toe and compliance steer, 15 upper hull points + 30 floor points, failure thresholds null; fitted jointly to
both soft-body landing designs and the soft-body drive tests; provenance and held-out errors in `_calibration`; the
first fit, `coldgas-rigid@2`, used the first landing design only, without κ / slide).
**v1** is the explicit override `tests/parity/vehicle-v1.json`: `wheels` `spring_k` 30000, `damper_c` 2600,
`travel` 0.11, `bump_k` 400000, `bump_c` 0, `bump_k2` 0, `tyre_mu` 1.0, `crr` 0.015, `toe` [0, 0, 0, 0],
`toe_compliance` 0; `tyre.model` `coulomb`; `wheel_spin.enabled` false; `failure` all null; `hull.points` the original
20 points (listed in the file: 4 bumper corners and the floor centre at y 0.2, sides and corners at 0.8–1.35, roof at
1.68–1.7), `k` 400000, `c` 9000, `mu` 0.55, no `floor_points`; `aero.CdA`
[2.6, 3.3, 0.75]; `twin.contact` `v_reg` 0.1, `wheel_min_h` 0.5; `twin.placement` 30 / 1e-7. Switched-off features
are not evaluated (the v1 expressions run unchanged), and the twins reproduce the `coldgas-rigid@1` trajectories bit
for bit with it (`tests/parity/v1-reference.json`, checked by `python/tests/test_twin_v2.py`, which builds all of its
mechanism tests on top of this override so they do not depend on the fitted values).
- *Bump stop* (`wheels`, per axle): `bump_c` (N s/m, damping while `d > travel`), `bump_k2` (N/m², progressive
  term); force law in 5.5, static part in 5.7. Per-axle `spring_k damper_c travel bump_k tyre_mu crr` likewise.
- *Tyre lateral force* `tyre.model`: `coulomb` (5.5) or `slip`: slip `s = v_l / max(|v_t|, v_slip_min)`,
  `σ_y = c_alpha s / μ_e` (`c_alpha` 1/rad per unit load, `v_slip_min` m/s, both per axle), `D = √(1 + σ_y σ_y)`,
  `f_l = −(μ_e N) σ_y / D` (= `−c_alpha N s` for small slip, saturating at `μ_e N`); without wheel spin `f_t` is
  the v1 rolling resistance.
- *Wheel spin* `wheel_spin.enabled` (default false): spin `ω` (rad/s, + = rolling forward, so rolling-matched is
  `ω r = −v_t`). Loaded wheel: longitudinal slip velocity `v_t + ω r`, `σ_x = (v_t + ω r)/v_long_sat`, and
  `D = √(1 + σ_x σ_x)` (`coulomb`) or `√(1 + σ_y σ_y + σ_x σ_x)` (`slip`: one friction circle; `f_l = −(μ_e N) σ_y / D`
  with this D, Coulomb `f_l` otherwise). With the secant stiffness `k = (μ_e N)/(v_long_sat D)`, `A = inertia/dt`,
  rolling-resistance torque `τ = −(crr N) r (ω r) / max(|ω r|, v_reg)`: `ω' = (A ω − k r v_t + τ)/(A + k r r)`,
  `f_t = −k (v_t + r ω')`, `ω ← ω'` (implicit in ω: the slip velocity contracts by `1/(1 + k r²/A)` and never
  overshoots, however stiff the tyre; the body gets exactly the force that spun the wheel). So `f_t` is the
  friction-circle force `−μ_e N σ_x / D` at the implicit slip (`−μ_e N sat((v_t + ω r)/v_long_sat)`,
  `sat(x) = x/√(1 + x²)`, without lateral slip and for `A ≫ k r²`); once rolling it is the v1 rolling resistance.
  Unloaded wheels keep `ω` (× `1 − dt·decay` per step). Unknown spin: reset *launch*/*rest* → rolling-matched,
  *state* → `init.wheel_spin` or `spin_ratio_air` × rolling-matched (soft-body wheels lose about half their spin in a
  long flight); `loadState` → `wheel_spin` of the FlightState if present (a twin's own state carries it), else by
  phase (`air`: `spin_ratio_air` × matched, `ground`/`deck`: matched). Rolling-matched uses `v_t` of the tyre point
  (rigid-body velocity at 5.5's tyre point, 0 if the axle is steeper than `wheel_min_h`): `ω = ratio · (−v_t / r)`.
  `inertia`, `v_long_sat` per axle. *Slip-ratio saturation* `slip_ratio_sat` κ (per axle, ≥ 0, default 0 = off and
  not evaluated for that wheel): a loaded wheel's saturation velocity becomes `v_ls = max(v_long_sat, κ |v_t'|)`
  (`a = κ·(v_t' < 0 ? −v_t' : v_t')`, `v_ls = a > v_long_sat ? a : v_long_sat`; `v_t'` the tyre-axis rolling speed),
  used for `σ_x` and `k` above: above `v_long_sat/κ` the longitudinal force saturates at a slip ratio of κ instead of
  a fixed slip velocity, so a wheel braked at the ABS slip bound (§5.13) keeps lateral force on the friction circle.
  *Sliding blend* `tyre.slide` (per axle, in [0, 1], default 0 = off and not evaluated for that wheel; with wheel spin):
  after the lateral law above, with the step-start `σ_x` and slip velocity `v_s0 = v_t' + ω r` (before the implicit
  update): `ξ = slide·(σ_x σ_x)/(1 + σ_x σ_x)`, `f_s = −(μ_e N)·v_l' / max(√(v_s0 v_s0 + v_l' v_l'), v_reg)`,
  `f_l ← (1 − ξ)·f_l + ξ·f_s` (`f_t` unchanged; `wh.lat` / diagnostics / the body force use the blended `f_l`). A wheel
  that slips longitudinally (locked or ABS-braked, spinning up at an impact) then grips sideways like isotropic Coulomb
  sliding along its slip velocity, the same per unit load front and rear (the soft body's sliding tyre nodes), instead
  of keeping the `c_alpha` front / rear ratio; a rolling wheel (`σ_x` ≈ 0) keeps the slip law.
  Only `+ − × ÷ √` (no new transcendental, both languages bit-identical).
- *Toe* (`wheels.toe`, 4 values FL FR RL RR, rad; `wheels.toe_compliance`, rad/N, per axle): each wheel's tyre axes
  are turned about the vertical by `δ = toe + δ_c`, `+` = front edge of the wheel to the car's right (the same
  sign for left and right wheels: on FL `+` is toe-in, on FR toe-out). Static part `c₀ = cos toe`, `s₀ = sin toe`
  (fmath, once per twin); compliance steer `tan δ_c = −toe_compliance · F_lat,prev` with `F_lat,prev` the wheel's
  tyre-axis lateral force of the previous integrator step (0 after a reset / loadState or while unloaded, `+` =
  towards the wheel's right), composed exactly: `d = −κ F`, `c_c = 1/√(1 + d d)`, `s_c = d c_c`, `c = c₀ c_c − s₀ s_c`,
  `s = s₀ c_c + c₀ s_c` (only evaluated when some compliance ≠ 0; else `c, s = c₀, s₀`). Positive compliance steers
  the wheel so as to reduce its lateral force: understeer on the front axle, oversteer on the rear (a rear wheel
  loaded towards the inside of a turn steers out of it). Tyre-axis velocities `v_t' = c v_t − s v_l`,
  `v_l' = c v_l + s v_t`; every force law (Coulomb or slip lateral, rolling resistance, spin slip `v_t' + ω r`) uses
  them, and the forces go back as `f_t = c f_t' + s f_l'`, `f_l = c f_l' − s f_t'` into 5.5's `F`. Rolling-matched
  spin uses `v_t'` of the static toe. Toe and compliance 0 (v1) skip all of this (v1 axes, nothing rotated).
- *Floor points* (`hull.floor_points`, `floor_k floor_c floor_mu`): extra hull points with their own penalty
  parameters (5.1); any number of hull points works, including none.
- *Wheel loads and failure* (always computed): per loaded wheel and integrator step `N`, `f_l`, `f_t` in the tyre
  axes (`f_l'`, `f_t'`: across / along the toed wheel; identical to the body-aligned values without toe; last step
  readable), peaks of `N` and `|f_l|` (restarted at the deck edge), lateral impulse `Σ |f_l| dt` over the steps that
  start after the touchdown step. With `failure.wheel_normal_max` (N), `wheel_lat_max` (N), `wheel_lat_impulse_max`
  (N s) (null = off, the default): at the first step off the deck (`left_deck`) where a loaded wheel has `N >`,
  `|f_l| >` or impulse `>` its threshold (wheels in order, then normal / lateral / impulse), event
  `{type: 'wheel_overload', wheel, quantity: 'normal'|'lat'|'lat_impulse'}`, recorded once per episode (pushed in
  the wheel pass, i.e. before a `hull_strike` / `touchdown` of the same step). Tasks: §8.5.
- Readers: JS `twin.wheelLoad {N, lat, long}`, `twin.wheelPeak {N, lat, latImpulse}`, `twin.wheelSpin`,
  `twin.wheelOverload {t, wheel, quantity} | null`; Python (lane `e`) `wheel_load(e)`, `wheel_peak(e)` `{N, lat,
  lat_impulse}`, `wheel_spin[e]`, `wheel_overload(e)`. Per-wheel resolved parameters: `contactParams().wheels[j]`
  (`kss` = κ, `sld` = slide, `Tb` = brake torque limit, …) / `contact_params()['wheel']` (`slip_ratio_sat`, `slide`,
  `brake_torque`, …). Parity: every scenario (they run on the fitted spec; `v2-*`
  add their overrides on top of it) and `python/tests/test_twin_v2.py` (v1 path, loadState, lanes, rules).

**5.12 Parameter override** (fitting, experiments and the `v2-*` parity scenarios; everything else runs the spec): JS
`new RigidTwin({vehicle})`, `new ColdGasEnv(task, {vehicle})`, `runEpisode({vehicle})`; Python
`Twin(n, vehicle=)`, `ColdGasEnv(task, vehicle=)`, `BatchColdGasEnv(task, vehicle=)`, `run_episode(vehicle=)`.
`vehicle` is a partial `vehicle.json` object deep-merged over the spec (objects key by key; arrays, numbers and null
replace): `wheels`, `tyre`, `wheel_spin`, `failure`, `hull`, `aero`, `twin.contact` (`v_reg`, `wheel_min_h`; not
`street_top_max`, which classifies the world), `twin.placement`. Keys the spec does not have are rejected (a typo
would otherwise fit nothing), so is `integrator` (the `dt` option sets the step). Trajectories keep the model id;
record the override with the run. The v1 model: `vehicle` = the contents of `tests/parity/vehicle-v1.json`. The drive
hardware (`spec/drive.json`, §5.13) is not part of the override.

**5.13 Brakes and steering** (`spec/drive.json`: the car's own hardware, shared with the soft body; not part of the
`vehicle` override). Constants per twin: `k_β = 1 − exp(−dt/brake.tau)` (portable `exp`), brake torque limit
`T_max,j = brake.torque_max[axle_j]` (N m, axle as in 5.1), `keep = 1 − brake.abs_slip`, steer lock
`δ_max = steer.max_deg·π/180`, per-step rate `r_δ = steer.rate_deg_s·π/180·dt` (this operand order in both languages).
- *Commands* (5.4): action entries `NU` (brake `b ∈ [0, 1]`) and `NU + 1` (steer `s ∈ [−1, 1]`; contract `DRIVE_IDX`,
  `NA = NU + ND`); clamped, NaN / missing → 0, forced to 0 while not armed (like the valves), held for the control step.
- *State*: brake pressure `β` and front road-wheel angle `δ` (rad, `+` = front edge of the wheel to the car's right = a
  right turn, the sign of `wheels.toe`), both 0 after reset(); loadState: the FlightState's `drive {brake, steer}` if
  present, else 0. FlightState (§7) `drive: {brake: β, steer: δ}`; trajectories `drive`, `drive_cmd`.
- *Per integrator step* (5.5 step 3b, after the propulsion, before the forces; each part skipped while its command and
  state are 0): brake `e = b − β`, `β = (e < 1e-9 ∧ e > −1e-9) ? b : β + e·k_β` (the snap, a model constant, lets `β` reach
  its command exactly: without it a release leaves a subnormal tail, `β·k_β` rounding to 0, and the brake could never
  be skipped again); steer `e = s·δ_max − δ`, `δ += (e > r_δ ? r_δ : e < −r_δ ? −r_δ : e)`. When `δ` changed: with
  `c_δ = cos δ`, `s_δ = sin δ` (fmath), every front wheel (axle 0) gets the heading `c₀' = c₀ c_δ − s₀ s_δ`,
  `s₀' = s₀ c_δ + c₀ s_δ` on top of its static toe `c₀, s₀` (5.11); `δ = 0` sets `c₀' = c₀`, `s₀' = s₀` exactly (no
  composition). Rear wheels keep `c₀, s₀`.
- *Steering*: the compliance steer of 5.11 composes on `(c₀', s₀')` exactly as on the static toe, and the tyre-axis
  rotation (with the 5.11 force laws) runs whenever toe / compliance are on **or** `δ ≠ 0`, also on the v1 override
  (Coulomb tyre, no spin, no toe: then the rotated axes are its only 5.11 element; rear wheels rotate by `(1, 0)`);
  with `δ = 0` and no toe nothing is rotated. Rolling-matched spin (5.11, reset / loadState) uses the current heading
  `(c₀', s₀')`.
- *Brake* (only with `wheel_spin.enabled`; otherwise `β` follows its command but nothing is braked): `T_j = β·T_max,j`.
  Loaded wheel, in the spin update of 5.11 with `den = A + k r r`: the free implicit update `ω* = (A ω − k r v_t' + τ)/den`;
  if `T_j > 0`: `d = T_j/den`, ABS target `g = keep·(−v_t'/r)` (`keep` × rolling-matched along the tyre); if `ω* > 0`:
  `g = g > 0 ? g : 0`, and if `ω* > g`: `ω* = ω* − d > g ? ω* − d : g`; else if `ω* < 0`: `g = g < 0 ? g : 0`, and if
  `ω* < g`: `ω* = ω* + d < g ? ω* + d : g`; then (as without brake) `f_t = −k (v_t' + r ω*)`, `ω ← ω*`. An implicit
  Coulomb brake: it changes the spin by at most `T_j/den` per step, never reverses it, never pulls it below the ABS target
  (slip ratio ≤ `abs_slip`; a slower wheel, e.g. one locked in the air, is released and spins up), and the body gets
  exactly the tyre force that decelerated the wheel. Unloaded wheel, after `ω ← ω·(1 − dt·decay)`: if `T_j > 0`:
  `d = T_j·dt/I`, `ω = ω > d ? ω − d : ω < −d ? ω + d : 0`. With the fitted tyre (`v_long_sat` 0.036 m/s, κ = 0) a wheel
  at the ABS bound saturates its friction circle longitudinally (full brake + steer 0.8 at 18 m/s: heading change −2° in
  0.8 s, i.e. none, vs +22° with κ = 0.08): `wheel_spin.slip_ratio_sat` (5.11) is the knob for the fit, and
  `tyre.slide` (5.11) for the yaw balance of a braked car (straight full braking from 18 m/s: heading drift 14.5° with
  κ = 0.08, 8.4° with κ = 0.08 and slide = 1).
- *Kerb*: in the wheel support search of 5.5, a loaded wheel (`N > 0`) whose chosen support is a street box (sidewalk
  or median top, not the ground plane) sets the step flag *kerb* (trajectory contact bit 32); the first one after
  `left_deck` is the event `{type: 'kerb', wheel}` (once per episode, pushed in the wheel pass, before that wheel's
  `wheel_overload`: events of one step by wheel, kerb first). See 5.10 for when a wheel can stand on a kerb.
- *Readers*: JS `twin.drive` `{brake, steer}` (fields `twin.beta`, `twin.delta`), `twin.ud` (applied `[b, s]`),
  `twin.kerb` `{t, wheel} | null`; Python (lane `e`) `drive(e)`, arrays `drive_brake`, `drive_steer` (N), `ud` (N, 2),
  `kerb(e)`. Zero drive: an episode whose drive commands are all 0 evaluates none of this (the @2 trajectories bit for
  bit). Parity: the `drive-*` and `land-stop-pd` scenarios (scenario key `drive`: piecewise-constant commands
  over the controller's action), `python/tests/test_drive.py` (spin never reversed, ABS bound, rate limit / clamp,
  unarmed, loadState incl. JS, lanes = scalar, κ, slide, kerb / stop task rules).

## 6. Launch placement and kerb kick (shared with the soft body, `CrashSim.setup`)

For a preset (`speed` km/h, `angle` deg, `spin` = [pitch, yaw, roll] rad/s, `lead` s, default
`launch-presets.defaults.lead`):

- `θ = angle·π/180`, `s = speed/3.6`, forward `f = (−cos θ, 0, sin θ)`, right `r = (sin θ, 0, cos θ)`, up `(0,1,0)`;
  the body axes (+X, +Y, +Z) map to (r, up, f); initial velocity `s·f`, no initial rotation rate.
- The vehicle's **full** centre of mass starts at horizontal `x_c = (gap[0]+gap[1])/2 + 1.5 − s f_x lead`,
  `z_c = halfWidth − s f_z lead − 0.9`, with the wheels resting on the deck (`y = deck.top`) at static suspension
  equilibrium. (The soft body gets its height from its settled pose; the twins from their own suspension —
  only the JS and Python twins must agree exactly.)
- **Kick**: once the COM passes `z > deck.halfWidth − kick.trigger`, for `kick.dur` seconds the world angular
  velocity grows at the constant rate `(spin[0]·r + spin[2]·f + spin[1]·up) / kick.dur` (r, f fixed at launch).

## 7. Interchange formats

- **Trajectory** `coldgas-traj@1` (JSON): `{ format, spec_hash, model, producer: 'js'|'python', task, controller,
  seed, preset, dt, n, t[n], state[n][13], comB[n][3], tank[n][3] (m, T, p), u_cmd[n][10], valve[n][10],
  thrust[n][10], events: [{t, type, ...}], metrics: {...} }`, sampled at the control rate (`propulsion.control.dt`).
  Viewers place the car with §1; the JS app can open trajectories from `experiments/runs/` in the dev server.
  Row `k` is the state at `t_k = k·dt` (the last row is the final state); `u_cmd[k]` is the command applied over
  `[t_k, t_k+1)` after arming (zeros in the last row); `valve[k]`/`thrust[k]` are those of the last integrator step
  before `t_k` (zeros at `k = 0`). Optional, written by both twins: `init` (the reset() initial condition incl. the
  RNG draws), `contact[n]` (bit mask of contacts during the step that ended at `t_k`: 1 any, 2 street, 4 deck,
  8 parapet/pier, 16 hull strike, 32 kerb: a loaded wheel on a street box, §5.13), `drive_cmd[n][2]` (applied
  `[brake, steer]` over `[t_k, t_k+1)`, zeros in the last row, like `u_cmd`; `u_cmd` stays `[n][10]`), `drive[n][2]`
  (`[β, δ]` at `t_k`, §5.13), `scenario` (parity goldens). Writers: `src/flight/traj.js`, `coldgas/traj.py`
  (`applied(u, ud)`: valve and drive commands of the twin, `twin.u` / `twin.ud`).
- **Soft-body thrust log** `coldgas-log@1`: `meta.propulsion` of a flight recording (`public/sims/flight-*.sim`),
  a superset of the trajectory columns (plus feed pressure, phase, armed, mount clusters, kit masses), sampled at
  `hz` from t = 0; `meta.flight` holds controller, task, events, landing metrics and damage. Authoritative
  description: header of `src/softbody/actuators.js`.
- **Spec hash**: FNV-1a 64-bit over every `*.json` under `spec/` (recursive), sorted by relative path with `/`
  separators; for each file feed `path` UTF-8 bytes, a 0x00 byte, the raw file bytes, a 0x00 byte. Lower-case hex,
  16 digits. Trajectories, policies and benchmark results record it; consumers warn on mismatch.
- **RNG** for task randomisation: mulberry32 (as in `src/softbody/solver.js`) seeded with `seed >>> 0`; draws in the
  order the task lists them (preset index first, then the `randomize` keys in file order), `uniform(a, b) =
  a + (b − a)·r()`. Seed 0 is mapped to 1 (`seed >>> 0 || 1`), so episode seeds 0 and 1 draw the same initial
  condition. Standard normals (MPPI noise): `gauss() = Σ₁₂ r() − 6` (Irwin–Hall; the sum of twelve multiples of
  2⁻³² is exact, so JS `rng.js` and numpy `Mulberry32.gauss_array` agree bit for bit).
- **FlightState** (what controllers see; twin or soft-body estimate): `{t, p[3], q[4], v[3], w[3] (body),
  comB[3], tank: {m, T, p}, valve[10], armed, phase: 'deck'|'air'|'ground', drive?: {brake, steer}}` (§1 frames;
  `t` from the episode start; `drive` = brake pressure 0..1 and front road-wheel angle in rad, §5.13, reported by the
  twins and the soft-body rig, read by loadState, 0 if absent). The twin adds `pc`, `thrust`, `step` (and `wheel_spin`
  when wheel spin is modelled, §5.11); controllers must not need `drive` or the twin extras.
- **Controller** (same names, settings and numbers in both languages; JS `src/flight/controllers/index.js`
  `createController(nameOrConfig)` / async `loadController`, Python `coldgas.controllers.create_controller`):
  `reset({task, seed})` once per episode (task = resolved task object, may be null), then `act(state, info)` →
  `u_cmd ∈ [0,1]^10` every `CONTROL_DT`, optionally followed by the drive commands brake ∈ [0, 1], steer ∈ [−1, 1]
  (an `NA = 12` long action, §5.13), called also while unarmed (the twin forces 0 then). A config is a name or
  `{name, ...settings}`; defaults live in `src/flight/controllers/defaults.json` (read by both). Registry:
  `zero`; `pd` (attitude + braking-profile autopilot on the bottom nozzles, uses only `+ − × ÷ √`, bit-identical in
  both languages); `replay` (`{traj}` or `{file}`: plays `u_cmd[round(t/dt)]`, zeros after the end, followed by
  `drive_cmd[round(t/dt)]` if the trajectory has it: an NA-long action); `mlp`
  (`{policy}` or `{file}`: runs a `coldgas-policy@1`); `mppi` (sampling MPC, residual on `pd`, rollouts on a twin
  loaded from the FlightState (`RigidTwin.loadState` / `Twin.load_state`) with integrator step `plan_dt`; noise
  from `gauss()` seeded `seed·1000003 + episode seed + 1`, reductions summed sequentially, softmax through the
  portable `exp`, so the two languages are bit-identical; settings and cost in `defaults.json` `mppi`). Python
  only: PPO-trained policies exported as `coldgas-policy@1`. JS only: `mpc` (pure MPC without a base pilot,
  `src/flight/mpc/`, settings in `src/flight/mpc/defaults.json`; deterministic, not parity-tested).
- **Policy** `coldgas-policy@1` (JSON): `{format, spec_hash, model, task, obs: {format: 'coldgas-obs@1', names[],
  mean[], std[], clip}, layers: [{W[out][in], b[out], act: 'tanh'|'relu'|'linear'}], action: {transform:
  'clip01'|'tanh01'}, meta: {...}}`. Evaluation: `x = clip((o − mean)/std, −clip, clip)` with `o` the task's
  observation (§8); per layer `y_j = act((Σ_i W[j][i] x_i, summed in order from 0) + b_j)`; `u = clip(y, 0, 1)`
  (`clip01`) or `(tanh(y)+1)/2` (`tanh01`); 0 while unarmed. `tanh(x) = sign(x)(1 − 2/(exp(2|x|)+1))` (saturated
  beyond |x| = 20) with the portable `exp` of `fmath`, so JS and Python evaluate a policy bit-identically.

## 8. Tasks, observations, rewards — owned by the flight-twin work

`tasks/*.json` are declarative; JS `src/flight/tasks.js` + `obs.js` and Python `coldgas/tasks.py` + `obs.py`
implement the same semantics (parity-tested on full episodes, rewards and metrics included). JS imports each task
file explicitly in `src/core/spec.js`; Python scans `spec/tasks/`.

**8.1 Resolution.** `extends: <id>` merges recursively: objects key by key (the parent's key order first, new keys
appended), arrays and scalars replaced by the child, except `metrics`, which is the parent's list followed by the
child's new entries. Keys starting with `_` are documentation.

**8.2 Initial state** for `(task, seed)`: `r = mulberry32(seed)`. *launch*: `k = floor(r()·len(presets))` (always
drawn; an explicit preset override ignores it), then each `randomize` key in order with `x = uniform(a, b)`:
`speed_kmh` adds km/h, `angle_deg` adds degrees, `spin_scale` multiplies `spin`; `lead` is the preset's or
`defaults.lead`. *rest*: `position` (x, z of the COM; y is found by the equilibrium), `heading_deg`, randomize
`yaw_deg` (added). Placement then follows §5.7.

**8.3 Arming** `arm`: `'edge'` (thrusters enabled at the first control-step boundary after the COM passed
`z > deck.halfWidth`), `'start'` (from t = 0).

**8.4 Per control step** (after the step; `Δm` = gas used in it; tilt = angle between body up and world up,
`acos(1 − 2(q_x² + q_z²))` with `fmath.acos`; height `h` = body-origin height above the street level under the COM,
`p_y − (R c)_y − street(p_x, p_z)`):
`gas_per_kg · Δm`; `tilt_per_rad_after_edge · tilt` once `left_deck`; hover tasks (`target`): until the hold is
complete, `height_error_per_m · |h − target.height|`, and the hold timer `+CONTROL_DT` while `|h − height| ≤
tolerance`, reset to 0 otherwise; the hold is complete once the timer reaches `target.hold` (−1e-9).
`recover_by_height`: the vehicle is *recovered* at the first step (after the edge) with COM height above the street
`≥ recover_by_height`, tilt `≤ recover_tilt_deg` and `|ω| ≤ recover_angular_rate`.

**8.5 Termination** (checked in this order after every control step): `hull_strike` if `success.hull_strike` is
false and a hull strike happened (§5.6) → terminated; `wheel_overload` if a wheel overload happened (§5.11, only with
failure thresholds set) and `success.wheel_overload` (default: `success.hull_strike`) is false → terminated; `kerb` if
`success.kerb` is false and the kerb event happened (§5.13) → terminated; `out_of_bounds` (COM below `limits.floor_y`
or farther than `limits.max_range` horizontally from its start) → terminated; `landed_early` (hover task touched down
before the hold was complete) → terminated; then, once touched down and `t ≥ t_touchdown + success.upright_hold` (0
if absent): stop tasks (`success.stop_speed` set) `stopped` as soon as the COM's horizontal speed `√(v_x² + v_z²) ≤
stop_speed` (`t_stop` = that step's `t`; no `landed` in stop tasks), other tasks `landed` → terminated; `timeout` at
`t ≥ horizon` → truncated.

**8.6 Success** (only for `landed` and `stopped`), every key optional, evaluated on the touchdown snapshot (§5.6):
`|v_y| ≤ max_vertical_speed`; tilt `≤ max_tilt_deg` at touchdown and at every control step after it until the end (the
hold window; up to the stop for `stopped`); `|ω| ≤ max_angular_rate`; pad distance (COM `(x, z)` at touchdown to
`pad.center`) `≤ max_pad_distance`; horizontal speed `√(v_x² + v_z²) ≤ max_horizontal_speed`; `recovered` if
`recover_tilt_deg` is given; hover tasks need the hold complete;
`stopped` also needs `t_stop − t_touchdown ≤ stop_within` if given. No hull strike (or kerb) is implied by reaching
`landed` / `stopped` unless the task forbids it.

**8.7 Terminal reward** (added on the final step): `terminal_success` if successful; `terminal_hull_strike` for
`hull_strike`; `terminal_wheel_overload` (default `terminal_hull_strike`) for `wheel_overload`; `terminal_kerb`
(default `terminal_hull_strike`) for `kerb`; nothing extra for an unsuccessful `stopped`; `terminal_timeout`
for the other failures (`timeout`, `out_of_bounds`, `landed_early`) so that neither hovering out the clock nor
hopping back down early pays; if a touchdown happened:
`terminal_vertical_speed_per_mps · |v_y,td| + terminal_tilt_per_rad · tilt_td + terminal_pad_distance_per_m ·
pad_distance`.

**8.8 Metrics** (the task's `metrics` list selects; always added: `return`, `reason`, `t_end`, `touchdown_t`):
`success`, `touchdown_vertical_speed` |v_y|, `touchdown_horizontal_speed`, `touchdown_tilt_deg`,
`touchdown_angular_rate` |ω|, `touchdown_wheels` (count), `peak_accel_g` (max specific force / g over the episode,
deck run included), `hull_strike`, `wheel_overload`, `wheel_peak_normal`, `wheel_peak_lat`, `wheel_lat_impulse` (max
over the wheels, §5.11), `gas_used_kg`, `landing_point` [x, z], `hold_time` (longest continuous hold),
`rms_height_error` (over the steps until the hold completed), `pad_distance`, `recovered`, `recover_time`,
`recover_height`, `stopped` (reason `stopped`), `stop_time` (`t_stop − t_touchdown`), `stop_distance` (horizontal COM
distance from the touchdown snapshot to the stop), `kerb` (the kerb event happened, §5.13); `null` when undefined.

**8.9 Observation** `coldgas-obs@1` (default for policies; `obs.js` / `obs.py`), from FlightState + task only:
`[up_B (3) = Rᵀ ŷ, v_B (3) = Rᵀ v, ω_B (3), v_y, h, m/m0, p_c/p_set (p_c from tank.p by the regulator rule, 0 if
exhausted), armed (0/1), valve (10)]` (24), then task extras in this order: `target` → `[h − height,
min(hold_timer/hold, 1)]` (the observer keeps its own hold timer, updated per observation with the rule of 8.4),
`pad` → `[pad.x − p_x, pad.z − p_z]`, `recover_by_height` → `[COM height above street − recover_by_height]`.
Raw SI values; scaling is the policy's `obs.mean/std`.

**8.10 Environments.** JS `src/flight/env.js` and Python `coldgas.env.ColdGasEnv(task, seed=…, preset=…)`:
`reset(seed?, options{preset, init}) → (obs, info)`, `step(u) → (obs, reward, terminated, truncated, info)` with `u`
the NU valve commands or the NA-long action incl. brake and steer (§5.13; `env.nu` = NU, `env.na` = NA; the gymnasium
`action_space` stays the NU valves);
`info = {t, phase, armed, preset, seed}` plus `reason`, `metrics` on the last step. Python
`coldgas.vecenv.BatchColdGasEnv(task, n, seed)` (actions `(n, NU)` or `(n, NA)`) runs n lanes on one numpy twin, lane `i`'s `j`-th episode with seed
`seed + i + j·n`, same-step auto-reset (final obs/metrics in `info['final'][i]`); every lane equals the scalar env
with the same seed bit for bit.


### 正式刹停演示任务

`level-land-stop` 继承 `escape-land-stop`，接地倾角与落地保持倾角上限收紧至 5°，接地角速度上限为 0.5 rad/s。
不设落点或接地水平速度约束，仍要求无车身/路缘碰撞并在原窗口内刹停。此任务只改变验收条件，
硬件、接触模型和刹车参数不变；JS 在 `src/core/spec.js` 注册，Python 按原规则自动加载任务文件。
