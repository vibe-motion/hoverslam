// Challenge tasks (spec/tasks/*.json; semantics in spec/README.md section 8): `extends` resolution, seeded
// initial-state sampling, and the per-episode tracker (rewards, termination, success, metrics). Mirrored by
// python/coldgas/tasks.py; both are parity-tested on the same episodes.
import { TASKS, LAUNCH, WORLD } from '../core/spec.js';
import { mulberry32 } from './rng.js';
import { acos } from './fmath.js';
import { streetLevel } from './world.js';
import { MODEL } from './model.js';

const isObj = x => x && typeof x === 'object' && !Array.isArray(x);
/** Deep merge: objects merge key by key (parent key order first), arrays and scalars are replaced. */
function merge(a, b) {
  const out = { ...a };
  for (const [k, v] of Object.entries(b)) out[k] = isObj(v) && isObj(a[k]) ? merge(a[k], v) : v;
  return out;
}

/** Resolved task: `extends` chains merged (child wins), `metrics` lists concatenated (parent first, unique). */
export function loadTask(idOrTask) {
  const raw = typeof idOrTask === 'string' ? TASKS[idOrTask] : idOrTask;
  if (!raw) throw new Error(`unknown task ${idOrTask}`);
  if (!raw.extends) return structuredClone(raw);
  const parent = loadTask(raw.extends);
  const { extends: _, ...child } = raw;
  const t = merge(parent, child);
  t.metrics = [...new Set([...(parent.metrics || []), ...(child.metrics || [])])];
  t.extends = raw.extends;
  return t;
}

/**
 * Initial condition for (task, seed): RNG draws in task order (preset index first, then the `randomize` keys in
 * file order). `opts.preset` forces a preset (the index is still drawn so the stream stays aligned).
 * @returns {object} RigidTwin.reset() init, plus {preset, draws}
 */
export function sampleInitial(task, seed, opts = {}) {
  const r = mulberry32(seed), I = task.initial, rnd = I.randomize || {}, draws = {};
  if (I.type === 'launch') {
    const k = Math.floor(r() * I.presets.length);
    const name = opts.preset ?? I.presets[k], P = LAUNCH.presets[name];
    if (!P) throw new Error(`unknown preset ${name}`);
    let speed = P.speed, angle = P.angle, scale = 1;
    for (const [key, [a, b]] of Object.entries(rnd)) {
      const x = r.uniform(a, b);
      draws[key] = x;
      if (key === 'speed_kmh') speed += x; else if (key === 'angle_deg') angle += x; else if (key === 'spin_scale') scale *= x;
      else throw new Error(`unknown randomize key ${key}`);
    }
    return { type: 'launch', speed, angle, spin: P.spin.map(s => s * scale), lead: P.lead ?? LAUNCH.defaults.lead, preset: name, draws };
  }
  if (I.type === 'rest') {
    let heading = I.heading_deg ?? 0;
    for (const [key, [a, b]] of Object.entries(rnd)) {
      const x = r.uniform(a, b);
      draws[key] = x;
      if (key === 'yaw_deg') heading += x; else throw new Error(`unknown randomize key ${key}`);
    }
    return { type: 'rest', x: I.position[0], z: I.position[2], heading_deg: heading, preset: null, draws };
  }
  throw new Error(`unknown initial type ${I.type}`);
}

/** Tilt (rad) between body up and world up from q, via the portable acos (parity). */
export function tiltOf(q) {
  const x = q[0], z = q[2], c = 1 - 2 * (x * x + z * z);
  return acos(c > 1 ? 1 : c < -1 ? -1 : c);
}
/** Height of the body-frame origin (bottom of the unloaded wheels) above the street below the COM. */
export function heightOf(p, q, comB) {
  const [x, y, z, w] = q;
  const r3 = 2 * (x * y + z * w), r4 = 1 - 2 * (x * x + z * z), r5 = 2 * (y * z - x * w);
  return p[1] - (r3 * comB[0] + r4 * comB[1] + r5 * comB[2]) - streetLevel(p[0], p[2]);
}
const norm3 = v => Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
const DEG = 180 / Math.PI;

/**
 * Episode bookkeeping for one vehicle (spec/README.md section 8). Call reset(twin) after twin.reset(), then
 * update(twin) after every control step: returns {reward, terminated, truncated, reason}. metrics() at the end.
 */
export class TaskTracker {
  constructor(task) {
    this.task = task;
    const S = task.success || {}, Rw = task.reward || {};
    this.S = S; this.Rw = Rw;
    this.cdt = null;
  }

  reset(twin) {
    this.cdt = twin.dt * twin.nsub;
    this.m0 = twin.prop.m; this.mPrev = twin.prop.m;
    this.ret = 0; this.done = false; this.reason = null; this.success = false;
    this.td = null; this.holdMaxTilt = 0;
    this.holdT = 0; this.holdBest = 0; this.holdDone = false; this.hErr2 = 0; this.hErrN = 0;
    this.recovered = false; this.recoverT = null; this.recoverH = null;
    this.stopT = null; this.stopD = null;
    this.origin = [twin.p[0], twin.p[2]];
  }

  update(twin) {
    const T = this.task, S = this.S, Rw = this.Rw, t = twin.k * twin.dt;
    let r = 0;
    // per-step terms
    const dm = this.mPrev - twin.prop.m;
    this.mPrev = twin.prop.m;
    if (Rw.gas_per_kg) r += Rw.gas_per_kg * dm;
    const tilt = tiltOf(twin.q);
    if (Rw.tilt_per_rad_after_edge && twin.leftDeck) r += Rw.tilt_per_rad_after_edge * tilt;
    const h = heightOf(twin.p, twin.q, twin.mp.com);
    if (T.target) {
      const err = h - T.target.height;
      if (!this.holdDone) {
        if (Rw.height_error_per_m) r += Rw.height_error_per_m * Math.abs(err);
        this.hErr2 += err * err; this.hErrN++;
        this.holdT = Math.abs(err) <= T.target.tolerance ? this.holdT + this.cdt : 0;
        if (this.holdT > this.holdBest) this.holdBest = this.holdT;
        if (this.holdT >= T.target.hold - 1e-9) this.holdDone = true;
      }
    }
    if (T.recover_by_height !== undefined && !this.recovered && twin.leftDeck) {
      const hc = twin.p[1] - streetLevel(twin.p[0], twin.p[2]);
      if (hc >= T.recover_by_height && tilt <= S.recover_tilt_deg / DEG && norm3(twin.w) <= S.recover_angular_rate) {
        this.recovered = true; this.recoverT = t; this.recoverH = hc;
      }
    }
    // touchdown snapshot comes from the integrator step of first street contact
    if (twin.touchdown && !this.td) {
      const d = twin.touchdown;
      this.td = { t: d.t, vy: Math.abs(d.v[1]), vh: Math.sqrt(d.v[0] * d.v[0] + d.v[2] * d.v[2]), tilt: tiltOf(d.q), rate: norm3(d.w), x: d.p[0], z: d.p[2], wheels: d.wheels };
    }
    if (this.td && t > this.td.t && tilt > this.holdMaxTilt) this.holdMaxTilt = tilt;
    // termination
    let terminated = false, truncated = false, reason = null;
    const hold = S.upright_hold ?? 0;
    if (twin.hullStrike && S.hull_strike === false) { terminated = true; reason = 'hull_strike'; }
    // a wheel overload (twin failure criterion, spec/README.md 5.11; never with the default null thresholds) is a
    // crash wherever hull strikes are; success.wheel_overload overrides
    else if (twin.wheelOverload && (S.wheel_overload ?? S.hull_strike) === false) { terminated = true; reason = 'wheel_overload'; }
    // a loaded wheel on a kerb (sidewalk / median top off the deck, 5.13) where the task forbids it
    else if (S.kerb === false && twin.kerb) { terminated = true; reason = 'kerb'; }
    else if (twin.p[1] < MODEL.limits.floor_y || Math.sqrt((twin.p[0] - this.origin[0]) ** 2 + (twin.p[2] - this.origin[1]) ** 2) > MODEL.limits.max_range) { terminated = true; reason = 'out_of_bounds'; }
    else if (this.td && T.target && !this.holdDone) { terminated = true; reason = 'landed_early'; }
    // stop tasks (success.stop_speed): after the hold, the episode ends once the COM's horizontal speed is low enough
    else if (this.td && t >= this.td.t + hold - 1e-9 && (S.stop_speed === undefined || Math.sqrt(twin.v[0] * twin.v[0] + twin.v[2] * twin.v[2]) <= S.stop_speed)) {
      terminated = true; reason = S.stop_speed === undefined ? 'landed' : 'stopped';
    } else if (t >= T.horizon - 1e-9) { truncated = true; reason = 'timeout'; }
    if (terminated || truncated) {
      if (reason === 'stopped') {
        const dx = twin.p[0] - this.td.x, dz = twin.p[2] - this.td.z;
        this.stopT = t - this.td.t; this.stopD = Math.sqrt(dx * dx + dz * dz);
      }
      this.success = (reason === 'landed' || reason === 'stopped') && this.passes() && !(reason === 'stopped' && S.stop_within !== undefined && !(this.stopT <= S.stop_within));
      r += this.terminalReward(reason);
      this.done = true; this.reason = reason;
    }
    this.ret += r;
    return { reward: r, terminated, truncated, reason };
  }

  /** Success tests on the touchdown snapshot and the hold window (all keys optional). */
  passes() {
    const S = this.S, d = this.td;
    if (!d) return false;
    if (S.max_vertical_speed !== undefined && !(d.vy <= S.max_vertical_speed)) return false;
    if (S.max_tilt_deg !== undefined && !(d.tilt <= S.max_tilt_deg / DEG && this.holdMaxTilt <= S.max_tilt_deg / DEG)) return false;
    if (S.max_angular_rate !== undefined && !(d.rate <= S.max_angular_rate)) return false;
    if (S.max_horizontal_speed !== undefined && !(d.vh <= S.max_horizontal_speed)) return false;
    if (S.max_pad_distance !== undefined && !(this.padDistance() <= S.max_pad_distance)) return false;
    if (S.recover_tilt_deg !== undefined && !this.recovered) return false;
    if (this.task.target && !this.holdDone) return false;
    return true;
  }

  padDistance() {
    const pad = this.task.pad;
    if (!pad || !this.td) return null;
    const dx = this.td.x - pad.center[0], dz = this.td.z - pad.center[2];
    return Math.sqrt(dx * dx + dz * dz);
  }

  /** Terminal reward of the final step (this.success already decided). */
  terminalReward(reason) {
    const Rw = this.Rw, d = this.td;
    let r = 0;
    if (this.success) r += Rw.terminal_success ?? 0;
    if (reason === 'hull_strike') r += Rw.terminal_hull_strike ?? 0;
    if (reason === 'wheel_overload') r += Rw.terminal_wheel_overload ?? Rw.terminal_hull_strike ?? 0;
    if (reason === 'kerb') r += Rw.terminal_kerb ?? Rw.terminal_hull_strike ?? 0;
    if (reason === 'timeout' || reason === 'out_of_bounds' || reason === 'landed_early') r += Rw.terminal_timeout ?? 0;
    if (d) {
      if (Rw.terminal_vertical_speed_per_mps) r += Rw.terminal_vertical_speed_per_mps * d.vy;
      if (Rw.terminal_tilt_per_rad) r += Rw.terminal_tilt_per_rad * d.tilt;
      if (Rw.terminal_pad_distance_per_m) r += Rw.terminal_pad_distance_per_m * this.padDistance();
    }
    return r;
  }

  metrics(twin) {
    const d = this.td, out = {};
    const all = {
      success: this.success,
      touchdown_vertical_speed: d ? d.vy : null,
      touchdown_horizontal_speed: d ? d.vh : null,
      touchdown_tilt_deg: d ? d.tilt * DEG : null,
      touchdown_angular_rate: d ? d.rate : null,
      touchdown_wheels: d ? popcount(d.wheels) : null,
      peak_accel_g: twin.peakSf / WORLD.gravity,
      hull_strike: !!twin.hullStrike,
      wheel_overload: !!twin.wheelOverload,
      wheel_peak_normal: maxOf(twin.wheels, 'pN'), wheel_peak_lat: maxOf(twin.wheels, 'pL'), wheel_lat_impulse: maxOf(twin.wheels, 'pI'),
      gas_used_kg: this.m0 - twin.prop.m,
      landing_point: d ? [d.x, d.z] : null,
      hold_time: this.task.target ? this.holdBest : null,
      rms_height_error: this.task.target && this.hErrN ? Math.sqrt(this.hErr2 / this.hErrN) : null,
      pad_distance: this.padDistance(),
      recovered: this.task.recover_by_height !== undefined ? this.recovered : null,
      recover_time: this.recoverT, recover_height: this.recoverH,
      stopped: this.reason === 'stopped', stop_time: this.stopT, stop_distance: this.stopD, kerb: !!twin.kerb,
    };
    for (const k of this.task.metrics || []) out[k] = all[k] ?? null;
    Object.assign(out, { return: this.ret, reason: this.reason, t_end: twin.k * twin.dt, touchdown_t: d ? d.t : null });
    return out;
  }
}
const maxOf = (W, k) => Math.max(W[0][k], W[1][k], W[2][k], W[3][k]);
const popcount = m => ((m & 1) + ((m >> 1) & 1) + ((m >> 2) & 1) + ((m >> 3) & 1));
