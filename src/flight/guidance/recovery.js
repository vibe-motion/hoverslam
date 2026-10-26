// Bounded switching-time search over the existing nonlinear predictor. The four installed bottom nozzles act
// as one ignition group (with independent active throttle for attitude); the six other nozzles remain RCS.
// Coast closes every command. Burn requires every bottom command >= minThrottle. Valve lag is still physics:
// this is a command constraint, not an assertion that actual thrust jumps instantaneously above weight.
import { PROPULSION } from '../../core/spec.js';
import { NU } from './model.js';
import { optimizeTrajectory } from './optimizer.js';

export const bottomIndices = PROPULSION.nozzles.flatMap((n, i) => n.group === 'bottom' ? [i] : []);

export function recoveryConfig(input = {}) {
  if (!input || typeof input !== 'object' || Array.isArray(input)) throw new TypeError('guidance recovery must be an object');
  const c = { enabled: false, upperCandidates: 9, lowerCandidates: 5, switchPenalty: 0, ...input };
  if (!(Number.isFinite(c.switchPenalty) && c.switchPenalty >= 0)) throw new RangeError('guidance recovery.switchPenalty must be finite and nonnegative');
  if (typeof c.enabled !== 'boolean') throw new TypeError('guidance recovery.enabled must be boolean');
  if (c.enabled && !(Number.isFinite(c.minThrottle) && c.minThrottle > 0 && c.minThrottle <= 1))
    throw new RangeError('guidance recovery.minThrottle must be explicit and in (0,1]');
  for (const k of ['upperCandidates', 'lowerCandidates'])
    if (!(Number.isInteger(c[k]) && c[k] >= 3 && c[k] <= 16)) throw new RangeError(`guidance recovery.${k} must be an integer in [3,16]`);
  if (c.enabled && bottomIndices.length !== 4) throw new Error('guidance recovery requires the existing four bottom nozzles');
  return c;
}

export function commandLimits(c) {
  return { kind: 'coast-or-burn', minThrottle: c.minThrottle, bottomIndices: [...bottomIndices],
    min: Array.from({ length: NU }, (_, i) => bottomIndices.includes(i) ? c.minThrottle : 0), max: 1,
    bottomIds: bottomIndices.map(i => PROPULSION.nozzles[i].id), coastAllOff: true, sharedIgnition: true };
}

// A single contiguous burn arc gives at most an ignition and a cutoff. Search their nearby stage boundaries,
// rather than enumerating 2^N gates or rounding the controller output after optimization.
function burnArc(controls, minimum) {
  const gates = controls.map(u => bottomIndices.reduce((sum, i) => sum + u[i], 0) / bottomIndices.length >= minimum / 2);
  const start = gates.indexOf(true);
  return start < 0 ? [controls.length, controls.length] : [start, gates.lastIndexOf(true) + 1];
}

export function recoveryPatterns(controls, relaxed, c, stage = 'upper') {
  const N = controls.length, cap = stage === 'lower' ? c.lowerCandidates : c.upperCandidates;
  const patterns = [], seen = new Set();
  const add = (a, b, source) => {
    a = Math.max(0, Math.min(N, a)); b = Math.max(a, Math.min(N, b));
    const pattern = Array.from({ length: N }, (_, k) => k >= a && k < b);
    const key = pattern.map(Number).join('');
    if (!seen.has(key)) { seen.add(key); patterns.push({ pattern, source }); }
  };
  const seed = burnArc(controls, c.minThrottle), arc = burnArc(relaxed, c.minThrottle);
  add(...seed, 'sampled-plan');
  if (stage === 'lower') {
    // First stage spans the actual execution hold; permit its gate to react to measured-state error. Extending
    // an ignition to now or delaying it one full stage keeps the candidate a single burn arc.
    add(seed[0] === 0 ? 1 : 0, seed[0] === seed[1] ? Math.max(1, seed[1]) : seed[1], 'switch-executed-hold');
  }
  add(...arc, 'relaxed');
  // A coast prefix and earlier cutoff are explicit candidates even when the relaxed schedule burns throughout.
  add(arc[0] + 1, arc[1], 'later-ignition');
  add(arc[0], arc[1] - 1, 'earlier-cutoff');
  add(arc[0] - 1, arc[1], 'earlier-ignition');
  add(arc[0], arc[1] + 1, 'later-cutoff');
  add(N, N, 'coast');
  add(0, N, 'burn');
  // All-coast seeds still need useful finite pulses; all-burn seeds need a clear initial coast alternative.
  add(Math.floor(N / 2), N, 'late-burn');
  add(1, N, 'coast-prefix');
  return patterns.slice(0, cap);
}

export function recoveryBounds(pattern, minimum) {
  const lower = pattern.map(() => new Float64Array(NU));
  const upper = pattern.map(on => new Float64Array(NU).fill(on ? 1 : 0));
  for (let k = 0; k < pattern.length; k++) if (pattern[k]) for (const i of bottomIndices) lower[k][i] = minimum;
  return { lower, upper };
}

// Only off->on events pay: lambda / seconds since the most recent shutdown.
// The first ignition has no shutdown history and is free. Predicted shutdowns are local to each candidate.
export function restartCost(pattern, dts, { previousBurn = false, lastShutdownTime = null, time = 0 } = {}) {
  let on = previousBurn, shutdown = lastShutdownTime, t = time, reciprocal = 0, switchCount = 0;
  const restarts = [];
  for (let k = 0; k < pattern.length; k++) {
    const next = !!pattern[k];
    if (next !== on) {
      switchCount++;
      if (!next) shutdown = t;
      else if (shutdown !== null) {
        const offSeconds = t - shutdown;
        reciprocal += offSeconds > 0 ? 1 / offSeconds : Infinity;
        restarts.push({ time: t, offSeconds });
      }
    }
    on = next; t += dts[k];
  }
  return { reciprocal, switchCount, restarts };
}

export function optimizeRecovery(options, c, stage = 'upper', history = {}) {
  let relaxed, relaxationError = null, evaluations = 0;
  try {
    relaxed = optimizeTrajectory(options);
    evaluations += relaxed.diagnostics?.iterations ?? 0;
  } catch (error) { relaxationError = String(error); }
  const candidates = [], patterns = recoveryPatterns(options.controls, relaxed?.controls ?? options.controls, c, stage);
  let best = null;
  for (const { pattern, source } of patterns) {
    try {
      // The optimizer projects the seed before its first nonlinear rollout, so even zero accepted SQP steps
      // return a trajectory simulated under exactly this disjoint command constraint.
      const result = optimizeTrajectory({ ...options, controls: relaxed?.controls ?? options.controls,
        controlBounds: recoveryBounds(pattern, c.minThrottle) });
      evaluations += result.diagnostics?.iterations ?? 0;
      const terminal = result.diagnostics?.terminalConstraints, feasible = terminal?.feasible ?? true;
      const { reciprocal, switchCount, restarts } = restartCost(pattern, options.dts, history);
      const switchingCost = c.switchPenalty === 0 ? 0 : c.switchPenalty * reciprocal;
      const score = result.cost + switchingCost + (options.terminalConstraints?.penalty ?? 1e4) * (terminal?.violation ?? 0);
      const item = { source, pattern: pattern.map(Number), cost: result.cost, score, feasible, switchCount, switchingCost, restarts,
        terminalConstraints: terminal ?? null, stop: result.diagnostics?.stop ?? null };
      candidates.push(item);
      if (!best || (feasible && !best.feasible) || (feasible === best.feasible && score < best.score))
        best = { result, feasible, score, item };
    } catch (error) { candidates.push({ source, pattern: pattern.map(Number), error: String(error), solver: error.diagnostics ?? null }); }
  }
  if (!best) {
    const error = new Error('guidance recovery failed for every fixed ignition pattern');
    error.diagnostics = { stage, relaxationError, candidates }; throw error;
  }
  best.result.diagnostics.recovery = { ...commandLimits(c), stage, search: 'bounded-single-burn-arc',
    ...history, penaltyKind: 'inverse-off-duration', switchPenalty: c.switchPenalty, switchCount: best.item.switchCount, switchingCost: best.item.switchingCost,
    selected: best.item.pattern, source: best.item.source, candidateCount: candidates.length, evaluations,
    relaxationError, relaxedCost: relaxed?.cost ?? null, candidates };
  return best.result;
}
