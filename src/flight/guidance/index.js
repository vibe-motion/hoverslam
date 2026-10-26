// Two-level, receding-horizon nonlinear guidance. The contact-free FlightModel is only a planner; the rigid twin or
// soft-body state remains authoritative. The upper plan sets the landing path and actual valve schedule; the
// short lower solve tracks a moving window of that plan from the latest measured state.
import defaults from './defaults.json' with { type: 'json' };
import { FlightModel, ND, NU } from './model.js';
import { optimizeTrajectory } from './optimizer.js';
import { recoveryConfig, commandLimits, optimizeRecovery } from './recovery.js';
import { createPD } from '../controllers/pd.js';
import { createGroundStop } from '../controllers/ground-stop.js';
import { LandingModes } from '../landing-modes.js';
import { CONTROL_DT, NA } from '../contract.js';
import { WORLD } from '../../core/spec.js';
import { streetLevel } from '../world.js';
import { heightOf } from '../tasks.js';

const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const finitePositive = (x, name) => {
  if (!(Number.isFinite(x) && x > 0)) throw new RangeError(`guidance ${name} must be finite and positive`);
  return x;
};

function blendState(model, a, b, f) {
  const delta = model.minus(b, a);
  for (let i = 0; i < ND; i++) delta[i] *= f;
  return model.plus(a, delta);
}

function landingDuration(s, task, c) {
  const x = task?.pad?.center?.[0] ?? s.p[0] + s.v[0] * 0.5;
  const z = task?.pad?.center?.[2] ?? s.p[2] + s.v[2] * 0.5;
  const ground = streetLevel(x, z) + s.comB[1] + c.terminal_height;
  const h = Math.max(0, s.p[1] - ground), g = WORLD.gravity;
  const t = (s.v[1] + Math.sqrt(Math.max(0, s.v[1] * s.v[1] + 2 * g * h))) / g;
  return clamp(t + c.landing_time_margin, c.upper_time_min, c.upper_time_max);
}

function makeReferences(model, s, task, seedStates, dts, c) {
  const refs = [Float64Array.from(seedStates[0])], N = dts.length;
  const T = dts.reduce((a, b) => a + b, 0), x0 = s.p[0], z0 = s.p[2];
  const pad = task?.pad?.center;
  const brakeHorizontal = !!pad || c.terminal_horizontal_speed !== undefined;
  const speed = Math.hypot(s.v[0], s.v[2]);
  const scale = pad ? 0 : Math.min(1, (c.terminal_horizontal_speed ?? speed) / Math.max(speed, 1e-9));
  const vxEnd = s.v[0] * scale, vzEnd = s.v[2] * scale;
  const targetX = pad ? pad[0] : x0 + (brakeHorizontal ? (s.v[0] + vxEnd) * 0.5 : s.v[0]) * T;
  const targetZ = pad ? pad[2] : z0 + (brakeHorizontal ? (s.v[2] + vzEnd) * 0.5 : s.v[2]) * T;
  const q0 = Float64Array.from(seedStates[0].subarray(6, 10));
  const heading = Math.atan2(-2 * (q0[0] * q0[2] - q0[1] * q0[3]), 1 - 2 * (q0[1] * q0[1] + q0[2] * q0[2]));
  const qUp = new Float64Array([0, Math.sin(heading / 2), 0, Math.cos(heading / 2)]);
  const upright = Float64Array.from(seedStates[0]); upright.set(qUp, 6);
  const touchdownY = streetLevel(targetX, targetZ) + s.comB[1] + c.terminal_height;
  let elapsed = 0;
  for (let k = 1; k <= N; k++) {
    elapsed += dts[k - 1];
    const f = clamp(elapsed / T, 0, 1), r = Float64Array.from(seedStates[k]);
    // Cubic Hermite position uses the same initial and terminal vertical speeds as the velocity reference below;
    // ballistic position paired with a separate linear velocity target created an internally inconsistent path.
    const f2 = f * f, f3 = f2 * f;
    const h00 = 2 * f3 - 3 * f2 + 1, h10 = f3 - 2 * f2 + f;
    const h01 = -2 * f3 + 3 * f2, h11 = f3 - f2;
    // Coast by default; pad tasks brake to zero, while stop demos can request a finite arrival speed without a pad.
    r[0] = brakeHorizontal ? h00 * x0 + h10 * T * s.v[0] + h01 * targetX + h11 * T * vxEnd : x0 + s.v[0] * elapsed;
    r[2] = brakeHorizontal ? h00 * z0 + h10 * T * s.v[2] + h01 * targetZ + h11 * T * vzEnd : z0 + s.v[2] * elapsed;
    r[1] = h00 * s.p[1] + h10 * T * s.v[1] + h01 * touchdownY + h11 * T * c.terminal_vy;
    r[3] = brakeHorizontal ? ((6 * f2 - 6 * f) / T) * x0 + (3 * f2 - 4 * f + 1) * s.v[0] + ((-6 * f2 + 6 * f) / T) * targetX + (3 * f2 - 2 * f) * vxEnd : s.v[0];
    r[5] = brakeHorizontal ? ((6 * f2 - 6 * f) / T) * z0 + (3 * f2 - 4 * f + 1) * s.v[2] + ((-6 * f2 + 6 * f) / T) * targetZ + (3 * f2 - 2 * f) * vzEnd : s.v[2];
    r[4] = ((6 * f2 - 6 * f) / T) * s.p[1] + (3 * f2 - 4 * f + 1) * s.v[1] +
      ((-6 * f2 + 6 * f) / T) * touchdownY + (3 * f2 - 2 * f) * c.terminal_vy;
    const attitudeFraction = f;
    const qref = blendState(model, seedStates[0], upright, attitudeFraction);
    r.set(qref.subarray(6, 10), 6);
    // Damp angular rates along the upright transition. Gas and valve references stay on the PD rollout, which
    // supplies a useful actuator/mass trajectory without pretending that the PD rollout is the optimized answer.
    for (let j = 0; j < 3; j++) r[10 + j] *= 1 - attitudeFraction;
    refs.push(r);
  }
  return refs;
}

function validateConfig(input) {
  const c = { ...defaults, ...input };
  delete c._doc;
  if (c.capturePlans === undefined) c.capturePlans = false;
  if (typeof c.capturePlans !== 'boolean') throw new TypeError('guidance capturePlans must be boolean');
  c.recovery = recoveryConfig(input.recovery);
  if (c.terminal_horizontal_speed !== undefined && !(Number.isFinite(c.terminal_horizontal_speed) && c.terminal_horizontal_speed >= 0)) throw new RangeError('guidance terminal_horizontal_speed must be finite and nonnegative');
  c.ground = { ...defaults.ground, ...(input.ground || {}) };
  delete c.ground._doc;
  if (typeof c.ground.enabled !== 'boolean') throw new TypeError('guidance ground.enabled must be boolean');
  c.upper_nodes = Math.trunc(finitePositive(c.upper_nodes, 'upper_nodes'));
  c.lower_nodes = Math.trunc(finitePositive(c.lower_nodes, 'lower_nodes'));
  c.upper_iterations = Math.trunc(finitePositive(c.upper_iterations, 'upper_iterations'));
  c.lower_iterations = Math.trunc(finitePositive(c.lower_iterations, 'lower_iterations'));
  for (const k of ['upper_replan', 'upper_time_min', 'upper_time_max', 'lower_dt', 'ballistic_time_allowance']) finitePositive(c[k], k);
  if (!(Number.isFinite(c.duration_penalty) && c.duration_penalty >= 0)) throw new RangeError('guidance duration_penalty must be finite and nonnegative');
  c.lower_replan_steps = Math.trunc(finitePositive(c.lower_replan_steps, 'lower_replan_steps'));
  if (typeof c.trust === 'number') finitePositive(c.trust, 'trust');
  else if (c.trust && typeof c.trust === 'object') {
    const checkRadii = (r, n, name) => {
      if (typeof r === 'number') return finitePositive(r, name);
      if (!(Array.isArray(r) || ArrayBuffer.isView(r)) || r.length !== n ||
          Array.from(r).some(x => !(Number.isFinite(x) && x >= 0)) || !Array.from(r).some(x => x > 0))
        throw new RangeError(`guidance ${name} must have ${n} nonnegative finite radii with at least one positive entry`);
    };
    checkRadii(c.trust.state ?? 0.25, ND, 'trust.state');
    checkRadii(c.trust.control ?? 0.25, NU, 'trust.control');
  } else throw new TypeError('guidance trust must be a positive scalar or radii object');
  if (c.upper_time_max < c.upper_time_min) throw new RangeError('guidance upper_time_max must be >= upper_time_min');
  if (!Array.isArray(c.upper_time_candidates) || c.upper_time_candidates.some(x => !(Number.isFinite(x) && x > 0)))
    throw new TypeError('guidance upper_time_candidates must contain positive finite scales');
  return c;
}

function stageWeights(c) {
  if (c.weights) return c.weights;
  return { state: { p: c.w_position, v: c.w_velocity, rotation: c.w_attitude,
    w: c.w_angular_rate, gas: c.w_gas, valve: c.w_valve }, control: c.w_control };
}

function terminalWeights(c) {
  if (c.terminalWeights) return c.terminalWeights;
  return { p: c.terminal_position, v: c.terminal_velocity, rotation: c.terminal_attitude,
    w: c.terminal_angular_rate, gas: c.terminal_gas, valve: c.terminal_valve };
}

/** Create the two-level nonlinear flight controller. QP initialization is owned by the controller registry. */
export function createDual(cfg = {}) {
  const c = validateConfig(cfg), model = new FlightModel(), pd = createPD(cfg.pd || {});
  const outputSize = c.ground.enabled ? NA : NU;
  const zero = new Float64Array(NU);
  const groundController = c.ground.enabled ? createGroundStop() : null;
  const flightDrive = new Float64Array(NA - NU);
  let task = null, episodeSeed = 0, upper = null, upperAt = -Infinity, lowerAt = -Infinity, lowerAction = null,
    upperPath = [], upperTimes = [], lowerPath = [], lowerTimes = [], upperCandidates = [], landingDeadline = null, tracedMode = 'flight';
  const modes = new LandingModes({ onChange: () => {
    resetFlight();
    groundController?.reset({ task });
  }, onDeparture: () => flightDrive.fill(0) });
  const stats = { upperSolves: 0, lowerSolves: 0, durationCandidates: 0, lastUpperCost: null, lastLowerCost: null,
    lastUpperDiagnostics: null, lastLowerDiagnostics: null, lastDiagnostics: null };
  const ctl = { name: 'dual', config: c, plans: 0, evals: 0, planMs: 0, lastCost: null, diagnostics: null, planningTrace: [] };
  if (c.recovery.enabled) ctl.commandLimits = commandLimits(c.recovery);
  let previousBurn = false, lastShutdownTime = null; // Last executed gate; replanning must not clear this boundary state.
  const now = () => globalThis.performance?.now?.() ?? Date.now();

  // Clear live planner state on either transition; cumulative solve counters remain episode diagnostics.
  function resetFlight() {
    previousBurn = false; lastShutdownTime = null;
    upper = null; upperAt = -Infinity; lowerAt = -Infinity; lowerAction = null;
    upperPath = []; upperTimes = []; lowerPath = []; lowerTimes = [];
    upperCandidates = []; landingDeadline = null;
    pd.reset({ task, seed: episodeSeed });
    for (const k of Object.keys(stats)) if (k.startsWith('last')) stats[k] = null;
    ctl.lastCost = null; ctl.diagnostics = null;
  }

  function pathOf(states) {
    return (states || []).map(x => [x[0], x[1], x[2]]);
  }

  function recordPlanning(t, mode) {
    if (!c.capturePlans) return;
    ctl.planningTrace.push({ t, mode,
      upper: mode === 'flight' ? upperPath : [], upperTimes: mode === 'flight' ? upperTimes : [],
      lower: mode === 'flight' ? lowerPath : [], lowerTimes: mode === 'flight' ? lowerTimes : [],
      candidates: mode === 'flight' ? upperCandidates : [],
      terminalConstraints: mode === 'flight' ? stats.lastUpperDiagnostics?.terminalConstraints : null,
      ...(c.recovery.enabled ? { recovery: mode === 'flight' ? {
        upper: stats.lastUpperDiagnostics?.recovery ?? null, lower: stats.lastLowerDiagnostics?.recovery ?? null,
      } : null } : {}),
    });
  }

  function rolloutPD(s, dts) {
    const x0 = model.pack(s), states = [x0], controls = [];
    let x = x0;
    pd.reset({ task, seed: 0 });
    let t = s.t;
    for (const dt of dts) {
      const view = model.state(x, t);
      const u = pd.act(view);
      controls.push(Float64Array.from(u));
      x = model.step(x, u, dt);
      states.push(Float64Array.from(x)); t += dt;
    }
    return { states, controls };
  }

  function estimatePDLandingDuration(s) {
    const x0 = model.pack(s);
    let x = x0, elapsed = 0;
    pd.reset({ task, seed: 0 });
    while (elapsed < c.upper_time_max) {
      const view = model.state(x, s.t + elapsed);
      if (heightOf(view.p, view.q, view.comB) <= 0.03 && elapsed > 0) return clamp(elapsed, c.upper_time_min, c.upper_time_max);
      x = model.step(x, pd.act(view), CONTROL_DT);
      elapsed += CONTROL_DT;
    }
    return landingDuration(s, task, c);
  }

  function initialLandingDuration(s) {
    const pdTime = estimatePDLandingDuration(s);
    const ballistic = landingDuration(s, task, { ...c, landing_time_margin: 0 });
    // Cap a conservative PD rollout horizon by freefall time plus a finite braking allowance. This limits horizontal
    // drift into distant streets when launch attitude changes the PD model's predicted thrust projection.
    return Math.min(pdTime, ballistic + c.ballistic_time_allowance);
  }

  function rolloutControls(s, controls, dts) {
    const states = [model.pack(s)];
    for (let k = 0; k < controls.length; k++) states.push(model.step(states[k], controls[k], dts[k]));
    return states;
  }

  function planUpper(s) {
    const t0 = now();
    try { return computeUpper(s); }
    finally { ctl.planMs += now() - t0; }
  }

  function terminalConstraints() {
    if (!task?.pad || task.success?.max_horizontal_speed === undefined) return undefined;
    return { penalty: c.terminal_constraint_penalty ?? 1000, position: { center: [task.pad.center[0], task.pad.center[2]], radius: task.success.max_pad_distance ?? task.pad.radius },
      velocity: { center: [0, 0], radius: task.success.max_horizontal_speed } };
  }

  function computeUpper(s) {
    const nominal = landingDeadline === null
      ? initialLandingDuration(s)
      : Math.max(CONTROL_DT * c.lower_replan_steps, landingDeadline - s.t);
    const overdue = landingDeadline !== null && s.t >= landingDeadline;
    let best = null;
    const candidates = [];
    const candidateTrajectories = [];
    for (const scale of c.upper_time_candidates) {
      stats.durationCandidates++;
      const minDuration = Math.max(CONTROL_DT * c.lower_replan_steps,
        Math.min(c.upper_time_min, Math.max(nominal * 0.5, CONTROL_DT * c.lower_replan_steps)));
      const duration = clamp(nominal * scale, minDuration, c.upper_time_max);
      const dts = Array.from({ length: c.upper_nodes }, () => duration / c.upper_nodes);
      const pdSeed = rolloutPD(s, dts), oldDuration = upper?.dts.reduce((a, b) => a + b, 0) ?? 0;
      const controls = upper ? dts.map((dt, i) => {
        const t = s.t + i * dt - upperAt;
        return t < oldDuration ? sampleUpper(s.t + i * dt, 'control') : pdSeed.controls[i];
      }) : pdSeed.controls;
      const states = rolloutControls(s, controls, dts);
      const references = makeReferences(model, s, task, states, dts, c);
      let result;
      try {
        const options = { model, initial: states[0], controls, dts, references,
          iterations: c.upper_iterations, weights: stageWeights(c), terminalWeights: terminalWeights(c), trust: c.trust,
          terminalConstraints: terminalConstraints() };
        result = c.recovery.enabled ? optimizeRecovery(options, c.recovery, 'upper', { previousBurn, lastShutdownTime, time: s.t }) : optimizeTrajectory(options);
      } catch (error) {
        const failure = { duration, error: String(error), solver: error.diagnostics ?? null };
        candidates.push(failure);
        continue;
      }
      ctl.evals += result.diagnostics?.recovery?.evaluations ?? result.diagnostics?.iterations ?? 0;
      if (c.capturePlans && candidateTrajectories.length < 8) candidateTrajectories.push(pathOf(result.states));
      const violation = result.diagnostics?.terminalConstraints?.violation ?? 0;
      const feasible = result.diagnostics?.terminalConstraints?.feasible ?? true;
      const score = result.cost + (result.diagnostics?.recovery?.switchingCost ?? 0) + (terminalConstraints()?.penalty ?? 0) * violation + c.duration_penalty * (duration - nominal) ** 2;
      const candidate = { duration, cost: result.cost, score, terminalConstraints: result.diagnostics?.terminalConstraints, stop: result.diagnostics?.stop ?? null,
        ...(c.recovery.enabled ? { recovery: result.diagnostics.recovery } : {}) };
      candidates.push(candidate);
      if (!best || (feasible && !best.feasible) || (feasible === best.feasible && score < best.score)) best = { result, references, dts, duration, score, feasible };
    }
    if (!best) {
      const error = new Error('dual guidance upper optimization failed for every duration candidate');
      error.diagnostics = { stage: 'upper', predictionOnly: true, candidates };
      ctl.diagnostics = error.diagnostics; stats.lastUpperDiagnostics = error.diagnostics; stats.lastDiagnostics = error.diagnostics;
      throw error;
    }
    upper = best;
    upperAt = s.t;
    if (c.capturePlans) {
      upperPath = pathOf(best.result.states);
      upperTimes = [upperAt];
      for (const dt of best.dts) upperTimes.push(upperTimes[upperTimes.length - 1] + dt);
      upperCandidates = candidateTrajectories;
    }
    if (landingDeadline === null) landingDeadline = s.t + best.duration;
    lowerAt = -Infinity;
    lowerAction = null;
    stats.upperSolves++;
    ctl.plans++;
    stats.lastUpperCost = best?.result.cost ?? null;
    const terminalError = model.minus(best.result.states.at(-1), best.references.at(-1));
    stats.lastUpperDiagnostics = { stage: 'upper', duration: best.duration, durationPrior: nominal,
      terminalError: { p: Array.from(terminalError.subarray(0, 3)), v: Array.from(terminalError.subarray(3, 6)),
        rotation: Array.from(terminalError.subarray(6, 9)), w: Array.from(terminalError.subarray(9, 12)) },
      landingDeadline, overdue, score: best.score, predictionOnly: true, candidates, ...best.result.diagnostics };
    stats.lastDiagnostics = stats.lastUpperDiagnostics;
    ctl.diagnostics = stats.lastUpperDiagnostics;
  }

  function sampleUpper(time, field) {
    const plan = upper?.result;
    if (!plan) return field === 'state' ? null : Float64Array.from(zero);
    const ts = [0];
    for (const dt of upper.dts) ts.push(ts[ts.length - 1] + dt);
    const elapsed = time - upperAt, duration = ts[ts.length - 1];
    if (field === 'control' && elapsed >= duration) return Float64Array.from(zero);
    if (field === 'state' && elapsed > duration) {
      // Continue beyond the optimized horizon as a simple landing tail: coast the terminal velocity, rotate to the
      // terminal upright heading, damp body rates, and command closed valves. Clamping state while reusing the last
      // thrust made the lower objective ask for mutually inconsistent terminal position, speed and actuation.
      const x = Float64Array.from(plan.states[plan.states.length - 1]), tail = elapsed - duration;
      for (let j = 0; j < 3; j++) x[j] += x[3 + j] * tail;
      const q = x.subarray(6, 10);
      const yaw = Math.atan2(-2 * (q[0] * q[2] - q[1] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
      const upright = Float64Array.from(x); upright.set([0, Math.sin(yaw / 2), 0, Math.cos(yaw / 2)], 6);
      const f = clamp(tail / 0.25, 0, 1), aligned = blendState(model, x, upright, f);
      aligned[10] *= 1 - f; aligned[11] *= 1 - f; aligned[12] *= 1 - f;
      aligned.fill(0, 14, 24);
      return aligned;
    }
    const t = clamp(elapsed, 0, duration);
    let i = 0;
    while (i + 1 < ts.length && ts[i + 1] < t) i++;
    if (field === 'control') return Float64Array.from(plan.controls[Math.min(i, plan.controls.length - 1)]);
    const a = plan.states[i], b = plan.states[Math.min(i + 1, plan.states.length - 1)];
    const f = ts[i + 1] > ts[i] ? (t - ts[i]) / (ts[i + 1] - ts[i]) : 0;
    return blendState(model, a, b, f);
  }

  function lowerPlan(s) {
    // Match the selected upper trajectory endpoint, not the initial duration prior.
    const remaining = upper ? upperAt + upper.duration - s.t : Infinity;
    // In recovery mode the first predicted command spans its entire execution hold. This makes the selected
    // gate/throttle executable even when a caller configures a lower mesh finer than the controller hold.
    const firstDt = c.recovery.enabled ? Math.max(c.lower_dt, c.lower_replan_steps * CONTROL_DT) : c.lower_dt;
    const horizon = c.recovery.enabled ? firstDt + (c.lower_nodes - 1) * c.lower_dt : c.lower_nodes * c.lower_dt;
    const reachesLanding = !!terminalConstraints() && remaining > 0 && remaining <= horizon;
    // Keep the executed first command interval intact; only the final stage is truncated.
    const count = reachesLanding ? Math.min(c.lower_nodes, c.recovery.enabled
      ? 1 + Math.max(0, Math.ceil((remaining - firstDt) / c.lower_dt - 1e-10))
      : Math.ceil(remaining / c.lower_dt)) : c.lower_nodes;
    const dts = Array.from({ length: count }, (_, i) => {
      const dt = i === 0 ? firstDt : c.lower_dt, elapsed = i === 0 ? 0 : firstDt + (i - 1) * c.lower_dt;
      // Keep the original lower mesh byte-for-byte equivalent while the optional recovery mode is disabled.
      return reachesLanding ? Math.min(dt, remaining - (c.recovery.enabled ? elapsed : i * c.lower_dt)) : dt;
    });
    const initial = model.pack(s), controls = [], references = [initial], times = [s.t];
    for (const dt of dts) {
      controls.push(sampleUpper(times.at(-1), 'control'));
      times.push(times.at(-1) + dt);
      references.push(sampleUpper(times.at(-1), 'state'));
    }
    let result;
    const t0 = now();
    try {
      const options = { model, initial, controls, dts, references, iterations: c.lower_iterations,
        weights: stageWeights(c), terminalWeights: terminalWeights(c), trust: c.trust,
        terminalConstraints: reachesLanding ? terminalConstraints() : undefined };
      result = c.recovery.enabled ? optimizeRecovery(options, c.recovery, 'lower', { previousBurn, lastShutdownTime, time: s.t }) : optimizeTrajectory(options);
    } catch (error) {
      ctl.diagnostics = { stage: 'lower', predictionOnly: true, error: String(error), solver: error.diagnostics ?? null,
        upper: stats.lastUpperDiagnostics };
      stats.lastLowerDiagnostics = ctl.diagnostics;
      stats.lastDiagnostics = ctl.diagnostics;
      throw error;
    } finally {
      ctl.planMs += now() - t0;
    }
    stats.lowerSolves++;
    if (c.recovery.enabled) ctl.evals += result.diagnostics?.recovery?.evaluations ?? 0;
    stats.lastLowerCost = result.cost;
    stats.lastLowerDiagnostics = { stage: 'lower', predictionOnly: true, ...result.diagnostics };
    stats.lastDiagnostics = stats.lastLowerDiagnostics;
    ctl.lastCost = result.cost;
    ctl.diagnostics = stats.lastLowerDiagnostics;
    if (c.capturePlans) {
      lowerPath = pathOf(result.states);
      lowerTimes = times;
    }
    return result.controls[0] || zero;
  }

  Object.assign(ctl, {
    reset({ task: t, seed = 0 } = {}) {
      task = t || null;
      if (task?.target) throw new Error('dual guidance currently supports airborne landing tasks; hover-hold targets are unsupported');
      if (task?.success?.stop_speed !== undefined && !c.ground.enabled) throw new Error('dual guidance ground stopping requires ground.enabled=true');
      if (task?.drive) throw new Error('dual guidance does not support externally scripted ground driving tasks');
      episodeSeed = seed;
      modes.reset();
      tracedMode = modes.mode;
      ctl.planningTrace.length = 0;
      flightDrive.fill(0);
      resetFlight();
      groundController?.reset({ task });
      for (const k of Object.keys(stats)) stats[k] = k.startsWith('last') ? null : 0;
      ctl.plans = 0; ctl.evals = 0; ctl.planMs = 0; ctl.lastCost = null; ctl.diagnostics = null;
    },
    act(s) {
      const out = new Float64Array(outputSize);
      modes.observe(s);
      if (!s.armed) return out;
      if (modes.mode === 'ground') {
        if (tracedMode !== 'ground') { recordPlanning(s.t, 'ground'); tracedMode = 'ground'; }
        // Exclusive ownership: flight planning is disabled and no cached valve command can escape.
        return groundController ? groundController.act(s) : out;
      }
      let replanned = false;
      if (!upper || s.t - upperAt >= c.upper_replan) { planUpper(s); replanned = true; }
      if (!lowerAction || s.t - lowerAt >= c.lower_replan_steps * CONTROL_DT - 1e-9) {
        lowerAction = Float64Array.from(lowerPlan(s));
        lowerAt = s.t;
        replanned = true;
      }
      if (replanned) recordPlanning(s.t, 'flight');
      tracedMode = 'flight';
      out.set(lowerAction.subarray(0, NU));
      if (c.recovery.enabled) {
        const burn = ctl.commandLimits.bottomIndices.some(i => out[i] > 0);
        if (previousBurn && !burn) lastShutdownTime = s.t;
        previousBurn = burn;
      }
      if (c.ground.enabled) out.set(flightDrive, NU);
      return out;
    },
    predict(s) {
      if (!upper) return { states: [], controls: [], duration: null, cost: null, diagnostics: null };
      return { states: upper.result.states.map((x, i) => model.state(x, upperAt + upper.dts.slice(0, i).reduce((a, b) => a + b, 0))),
        controls: upper.result.controls.map(u => Float64Array.from(u)), duration: upper.duration,
        times: upper.dts.reduce((a, dt) => { a.push(a[a.length - 1] + dt); return a; }, [0]),
        cost: upper.result.cost, diagnostics: stats.lastUpperDiagnostics };
    },
  });
  // Higher-level missions may explicitly request flight again; contact bounce alone does not rearm flight.
  ctl.setMode = mode => modes.setMode(mode);
  Object.defineProperty(ctl, 'mode', { get: () => modes.mode });
  Object.defineProperty(ctl, 'statistics', { enumerable: true, get: () => ({ ...stats,
    mode: modes.mode, modeTransitions: modes.transitions, departureResets: modes.departures,
    flightActive: modes.mode === 'flight', ground: {
      enabled: !!groundController, active: !!groundController && modes.mode === 'ground',
      stopped: groundController?.stopped ?? false,
    },
  }) });
  ctl.groundController = groundController;
  return ctl;
}

export { createDual as createGuidance };
