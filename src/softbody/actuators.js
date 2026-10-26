// Cold-gas thrusters on the soft body: the 10 nozzles of spec/propulsion.json acting on the XPBD lattice through
// the solver's `sim.actuators` hook, closed-loop through any Controller (src/flight/contract.js). The soft body is
// the "real world" the controllers designed on the rigid twin are transferred to, so everything a real vehicle
// would have to measure is estimated from the lattice here, and the propulsion is the shared reference model.
//
// Design (see also the report in docs / the flight program):
//  * Mount clusters. Each nozzle is anchored in BODY-piece nodes (never the bolt-on plastic: the rear nozzles sit
//    in the bumper visually, their thrust structure reaches into the rear panel / bulkhead behind it). Nodes are
//    ranked by distance to the thrust line through the mount, with depth along the line (into the structure)
//    counting only AXIAL of its length, and the nearest CLUSTER nodes form the mount. The thrust is shared over
//    the cluster mass-weighted (uniform Δv: no needless internal strain), tilted linearly so the share-weighted
//    centroid lies exactly on the thrust line: the soft body then sees the same moment arm as the rigid twin.
//  * Direction. Every 1 ms the cluster's current rotation is found by shape matching (rest -> current, warm
//    started) and the nozzle's body-frame thrust axis is rotated with it. A crushed floor or a bent rear panel
//    therefore points its jet where the metal points; a cluster that lost most of its nodes falls back to the
//    body-chunk rotation.
//  * Impulses. Once per substep each node n of nozzle i gets Δv = F_i·dt·share_n/m_n along that axis, i.e. the
//    total impulse per substep is F_i·dt. The thrust comes from `Propulsion` stepped every PROP_EVERY substeps
//    (1 ms at 8 kHz, the rigid twin's dt) with the command held for a control period (CONTROL_DT).
//  * Kit masses. Bottles (dry + gas share), nozzles and the plumbing are added to body nodes around where they
//    sit (bottles: the rack footprint over floor / wheel arches / side walls, centroid tilted onto each bottle's
//    centre; nozzles: their mount cluster; plumbing: the nearest nodes). The car's own mass array is never
//    touched (`assets.car()` is cached and shared): the rig keeps its own copy and writes 1/m into `sim.w`.
//  * Gas drains. Σṁ·dt leaves the bottle nodes every propulsion step (velocities are kept: the gas leaves at the
//    vehicle's velocity, its relative momentum is already the thrust). Per step the change is ~1e-4 of a node
//    mass, far below anything that upsets XPBD; total effect ≈ 5-10 % of the vehicle mass over a flight.
//  * Estimator (every control period): whole-vehicle (alive nodes of attached pieces) mass-weighted COM p and
//    velocity v; orientation q by shape matching the body chunk against its rest pose (rest frame = body frame B);
//    body-axis angular velocity from the angular momentum and the inertia tensor of the node cloud; comB = the
//    mass-weighted rest-frame COM (moves forward as the rear bottles empty); tank / valves from Propulsion;
//    armed by the task rule ('edge': COM past the deck edge, 'start': always); phase deck / air / ground.
//
// Recording log `meta.propulsion` (format 'coldgas-log@1'), sampled at `hz` (default 100 Hz) from t = 0:
//   { format, hz, n, nozzles: ids (action order), t[n], u_cmd[n][NU], valve[n][NU], thrust[n][NU] (N),
//     tank[n][3] (m kg, T K, p Pa), pc[n] (feed pressure, Pa), state[n][13] ([p, q, v, w] as spec/README §1),
//     comB[n][3], phase[n] ('d' deck | 'a' air | 'g' ground), armed[n] (0/1),
//     mounts: [{ id, group, nodes: [...], share: [...], exit: B, mount: B, dir: B thrust axis, exhaust: B }],
//     kit: { dry, gas0, total0 } (kg) }
//   Plume pose at time t: rigid fit of the mount nodes' recorded positions to their rest positions (share
//   weighted), applied to `exit` / `exhaust`. Intensity: `thrust` (linear in time between samples).
//   drive_cmd[n][2] (brake 0..1, steer -1..1 commanded), drive[n][2] (brake pressure 0..1, steer angle rad),
//   wheel_spin[n][4] (rad/s relative to the body, + rolling forward; FL FR RL RR) — brakes / steering: ./drive.js
// `meta.flight` (see ThrusterRig.summary): controller, task, events [{t, type}], metrics (landing), damage.
// Drive: a controller output longer than NU carries the brake / steer commands (spec/drive.json channels, indices
// NU, NU + 1); forced to 0 while unarmed like the valves. Kerb: a vehicle node touching a sidewalk / median box.
import { PROPULSION, DRIVE } from '../core/spec.js';
import { Propulsion } from '../flight/propulsion.js';
import { loadTask } from '../flight/tasks.js';
import { NU, CONTROL_DT, NOZZLE_IDS } from '../flight/contract.js';
import { quatToMat, quatMul, quatNorm } from '../core/quat.js';
import { DECK } from '../scene/layout.js';
import { BEAM } from '../car/carPrep.js';
import { SoftDrive } from './drive.js';

const CLUSTER = 128;     // body nodes per nozzle mount (~12 kg of lattice: spreads a bottom nozzle's 8.7 kN without denting the floor)
const AXIAL = 0.35;      // weight of depth along the thrust line when ranking mount nodes
const PROP_EVERY = 8;    // substeps per propulsion step (1 ms at 8 kHz)
const STREET_Y = 1.0;    // contacts below this height are with the street (task "touchdown" rule)
const KERB_MARGIN = 0.02; // m: a node pushed out of a kerb's side face sits on the face (tyre nodes have no allowance)
const ND = DRIVE.channels.length;
const deg = 180 / Math.PI;

// ------------------------------------------------------------------------------------------------ geometry
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const norm = a => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
/** Two unit vectors spanning the plane perpendicular to d. */
function perpBasis(d) {
  const a = Math.abs(d[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const e1 = norm(cross(d, a));
  return [e1, cross(d, e1)];
}
function solveSym(A, b) { // tiny dense solve (Gauss-Jordan, partial pivot); A n×n row-major arrays
  const n = b.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const v = M[c][c]; if (Math.abs(v) < 1e-12) return null;
    for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c] / v; for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
  }
  return M.map((r, i) => r[n] / r[i]);
}

/**
 * Non-negative node shares (Σ = 1) ∝ base mass, tilted linearly so the weighted centroid matches `target`
 * along the `basis` directions (2 = a thrust line, 3 = a point). Clamping negative shares and re-solving on
 * the clamped weights converges to a positive weighting whenever the target lies inside the cloud.
 */
export function centroidShares(nodes, rest, base, target, basis, iters = 12) {
  const n = nodes.length, w = Float64Array.from(nodes, k => base[k]);
  for (let it = 0; it < iters; it++) {
    let M = 0; const c = [0, 0, 0];
    for (let i = 0; i < n; i++) { const k = nodes[i] * 3; M += w[i]; c[0] += w[i] * rest[k]; c[1] += w[i] * rest[k + 1]; c[2] += w[i] * rest[k + 2]; }
    c[0] /= M; c[1] /= M; c[2] /= M;
    const err = basis.map(e => dot(e, sub(c, target)));
    if (Math.max(...err.map(Math.abs)) < 1e-5) break;
    // covariance projected on the basis
    const C = basis.map(() => basis.map(() => 0));
    for (let i = 0; i < n; i++) {
      const k = nodes[i] * 3, r = [rest[k] - c[0], rest[k + 1] - c[1], rest[k + 2] - c[2]];
      const pr = basis.map(e => dot(e, r));
      for (let a = 0; a < basis.length; a++) for (let b = 0; b < basis.length; b++) C[a][b] += w[i] * pr[a] * pr[b];
    }
    const g = solveSym(C, err.map(e => -M * e));
    if (!g) break;
    for (let i = 0; i < n; i++) {
      const k = nodes[i] * 3, r = [rest[k] - c[0], rest[k + 1] - c[1], rest[k + 2] - c[2]];
      let f = 1; for (let a = 0; a < basis.length; a++) f += g[a] * dot(basis[a], r);
      w[i] = Math.max(0, w[i] * f);
    }
  }
  let s = 0; for (let i = 0; i < n; i++) s += w[i];
  for (let i = 0; i < n; i++) w[i] /= s;
  return w;
}

const bodyChunk = car => car.chunks.find(c => c.piece === 'body');

/** Mount cluster of one nozzle: CLUSTER body nodes around the thrust line, shares on that line. */
export function nozzleMount(car, z, k = CLUSTER) {
  const R = car.nodes.rest, body = bodyChunk(car);
  const d = norm(z.exhaust.map(v => -v)), m = z.mount;
  const scored = [];
  for (let n = body.nodeStart; n < body.nodeStart + body.nodeCount; n++) {
    const r = [R[n * 3] - m[0], R[n * 3 + 1] - m[1], R[n * 3 + 2] - m[2]];
    const ax = dot(r, d), p2 = dot(r, r) - ax * ax;
    scored.push([p2 + (ax > 0 ? AXIAL * AXIAL * ax * ax : ax * ax), n]);
  }
  scored.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const nodes = Int32Array.from(scored.slice(0, k), s => s[1]);
  const share = centroidShares(nodes, R, car.nodes.mass, m, perpBasis(d));
  return { nodes, share, dir: d };
}

/** Distance from p to the segment c ± axis·L/2. */
function segDist(p, c, axis, L) {
  const r = sub(p, c), t = Math.max(-L / 2, Math.min(L / 2, dot(r, axis)));
  return Math.hypot(r[0] - t * axis[0], r[1] - t * axis[1], r[2] - t * axis[2]);
}

/**
 * Kit masses per node (kg): `dry` = bottles' dry mass + nozzles + hardware, `gas` = gas at a full tank.
 * Each bottle hangs in its rack on the body nodes around it (capsule around the bottle grown until it holds
 * RACK nodes: floor below, wheel arches and side walls at its ends), centroid tilted onto the bottle centre.
 */
const RACK = 1200;
export function kitMasses(car, spec = PROPULSION, mounts = null, gas0 = new Propulsion(spec).m0) {
  const N = car.nodes.count, R = car.nodes.rest, base = car.nodes.mass, body = bodyChunk(car);
  const dry = new Float64Array(N), gas = new Float64Array(N);
  const B = spec.tank.bottles, V = B.reduce((s, b) => s + b.volume, 0), I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  const rack = new Set();
  for (const b of B) {
    const ax = norm(b.axis), near = [];
    for (let n = body.nodeStart; n < body.nodeStart + body.nodeCount; n++) near.push([segDist([R[n * 3], R[n * 3 + 1], R[n * 3 + 2]], b.center, ax, b.length), n]);
    near.sort((a, c) => a[0] - c[0] || a[1] - c[1]);
    const nodes = Int32Array.from(near.slice(0, RACK), e => e[1]);
    const s = centroidShares(nodes, R, base, b.center, I3);
    const g = gas0 * b.volume / V;
    for (let i = 0; i < nodes.length; i++) { dry[nodes[i]] += b.dry_mass * s[i]; gas[nodes[i]] += g * s[i]; rack.add(nodes[i]); }
  }
  const rackNodes = Int32Array.from([...rack].sort((a, c) => a - c));
  for (const h of spec.hardware) {
    const near = [];
    for (let n = body.nodeStart; n < body.nodeStart + body.nodeCount; n++) near.push([Math.hypot(R[n * 3] - h.center[0], R[n * 3 + 1] - h.center[1], R[n * 3 + 2] - h.center[2]), n]);
    near.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    const nodes = Int32Array.from(near.slice(0, 96), e => e[1]);
    const s = centroidShares(nodes, R, base, h.center, I3);
    for (let i = 0; i < nodes.length; i++) dry[nodes[i]] += h.mass * s[i];
  }
  mounts ||= spec.nozzles.map(z => nozzleMount(car, z));
  spec.nozzles.forEach((z, i) => { const M = mounts[i]; for (let k = 0; k < M.nodes.length; k++) dry[M.nodes[k]] += z.mass * M.share[k]; });
  return { dry, gas, rack: rackNodes };
}

// ------------------------------------------------------------------------------------------------ rotation fit
const _R = new Float64Array(9);
/** Müller et al. 2016 rotation extraction: rotate q (warm start) to best align rest -> current (A = Σ w p Pᵀ). */
function fitRotation(A, q, iters) {
  for (let it = 0; it < iters; it++) {
    quatToMat(q, _R);
    let wx = 0, wy = 0, wz = 0, den = 0;
    for (let c = 0; c < 3; c++) {
      const rx = _R[c], ry = _R[3 + c], rz = _R[6 + c], ax = A[c], ay = A[3 + c], az = A[6 + c];
      wx += ry * az - rz * ay; wy += rz * ax - rx * az; wz += rx * ay - ry * ax; den += rx * ax + ry * ay + rz * az;
    }
    den = Math.abs(den) + 1e-9; wx /= den; wy /= den; wz /= den;
    const ang = Math.hypot(wx, wy, wz);
    if (ang < 1e-10) break;
    const s = Math.sin(ang / 2) / ang;
    quatMul([wx * s, wy * s, wz * s, Math.cos(ang / 2)], q, q);
    quatNorm(q);
  }
  return q;
}
/** Weighted rigid fit of node positions X (flat) to rest positions: fills c (centroid) and q (warm started). */
export function fitFrame(X, rest, nodes, wts, restC, q, c, iters = 4, A = new Float64Array(9)) {
  let W = 0; c[0] = c[1] = c[2] = 0;
  for (let i = 0; i < nodes.length; i++) { const k = nodes[i] * 3, w = wts[i]; W += w; c[0] += w * X[k]; c[1] += w * X[k + 1]; c[2] += w * X[k + 2]; }
  if (W <= 0) return 0;
  c[0] /= W; c[1] /= W; c[2] /= W;
  A.fill(0);
  for (let i = 0; i < nodes.length; i++) {
    const k = nodes[i] * 3, w = wts[i];
    const px = (X[k] - c[0]) * w, py = (X[k + 1] - c[1]) * w, pz = (X[k + 2] - c[2]) * w;
    const qx = rest[k] - restC[0], qy = rest[k + 1] - restC[1], qz = rest[k + 2] - restC[2];
    A[0] += px * qx; A[1] += px * qy; A[2] += px * qz; A[3] += py * qx; A[4] += py * qy; A[5] += py * qz; A[6] += pz * qx; A[7] += pz * qy; A[8] += pz * qz;
  }
  fitRotation(A, q, iters);
  return W;
}
/** Row-major rotation matrix -> quaternion [x, y, z, w]. */
export function matToQuat(m) {
  const tr = m[0] + m[4] + m[8];
  let x, y, z, w;
  if (tr > 0) { const s = 0.5 / Math.sqrt(tr + 1); w = 0.25 / s; x = (m[7] - m[5]) * s; y = (m[2] - m[6]) * s; z = (m[3] - m[1]) * s; }
  else if (m[0] > m[4] && m[0] > m[8]) { const s = 2 * Math.sqrt(1 + m[0] - m[4] - m[8]); w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s; }
  else if (m[4] > m[8]) { const s = 2 * Math.sqrt(1 + m[4] - m[0] - m[8]); w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s; }
  else { const s = 2 * Math.sqrt(1 + m[8] - m[0] - m[4]); w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s; }
  return quatNorm([x, y, z, w]);
}
const restCentroid = (rest, nodes, wts) => {
  const c = [0, 0, 0]; let W = 0;
  for (let i = 0; i < nodes.length; i++) { const k = nodes[i] * 3; W += wts[i]; c[0] += wts[i] * rest[k]; c[1] += wts[i] * rest[k + 1]; c[2] += wts[i] * rest[k + 2]; }
  return c.map(v => v / W);
};

// ------------------------------------------------------------------------------------------------ the rig
const r4 = v => Math.round(v * 1e4) / 1e4, r1 = v => Math.round(v * 10) / 10, r5 = v => Math.round(v * 1e5) / 1e5;

export class ThrusterRig {
  /**
   * @param car  buildCar output (only typed arrays are used)
   * @param opts controller   Controller (reset/act) or null = thrusters stay closed
   *             task         task spec (spec/tasks) or id; default escape-soft-landing
   *             logHz        log rate (default 100)
   *             drainGas     remove the used gas from the bottle nodes (default true)
   *             armDelay     extra seconds between the arming event and enabling the thrusters (fault studies)
   *             drive        brakes / steering (./drive.js, spec/drive.json); false = none (inert without commands)
   */
  constructor(car, { spec = PROPULSION, controller = null, task = 'escape-soft-landing', logHz = 100, drainGas = true, armDelay = 0, seed = 0, controllerName = '', drive = true } = {}) {
    this.car = car; this.spec = spec;
    this.task = typeof task === 'string' ? loadTask(task) : task; // resolved (`extends` merged)
    this.controller = controller; this.controllerName = controllerName || controller?.name || 'none';
    this.logHz = logHz; this.drainGas = drainGas; this.armDelay = armDelay; this.seed = seed;
    this.prop = new Propulsion(spec);
    this.mounts = spec.nozzles.map(z => nozzleMount(car, z));
    const kit = (this.kit = kitMasses(car, spec, this.mounts, this.prop.m0));
    const N = car.nodes.count, R = car.nodes.rest;
    this.base = Float64Array.from(car.nodes.mass);
    this.mass = new Float64Array(N);
    this.gasNodes = Int32Array.from({ length: N }, (_, n) => n).filter(n => kit.gas[n] > 0);
    this.kitDry = kit.dry.reduce((s, v) => s + v, 0);
    // the kit's rear coil-overs: spring beams whose wheel end belongs to a rear wheel (spec suspension)
    const B = car.beams, chunk = car.nodes.chunk, rearWheel = new Set(car.wheels.filter(wh => wh.centre[2] <= 0).map(wh => wh.chunk));
    this.rearSprings = Int32Array.from({ length: B.count }, (_, b) => b).filter(b => B.kind[b] === BEAM.SPRING && (rearWheel.has(chunk[B.i[b]]) || rearWheel.has(chunk[B.j[b]])));
    this.rearSpringScale = spec.suspension?.rear_spring_scale ?? 1;
    // and its heavy-duty rear control arms: SUSP beams of the rear wheels, give x rear_arm_give_scale
    this.rearArms = Int32Array.from({ length: B.count }, (_, b) => b).filter(b => B.kind[b] === BEAM.SUSP && (rearWheel.has(chunk[B.i[b]]) || rearWheel.has(chunk[B.j[b]])));
    this.rearArmScale = spec.suspension?.rear_arm_give_scale ?? 1;
    // per-mount rotation fit
    this.mq = this.mounts.map(() => [0, 0, 0, 1]);
    this.mc = this.mounts.map(() => [0, 0, 0]);
    this.mRestC = this.mounts.map(M => restCentroid(R, M.nodes, M.share));
    this.mDirW = this.mounts.map(M => [...M.dir]);
    this.mMin = this.mounts.map(M => 0.5); // share of alive nodes below which a mount uses the body rotation
    // body chunk (orientation)
    const body = bodyChunk(car);
    this.bodyNodes = Int32Array.from({ length: Math.ceil(body.nodeCount / 2) }, (_, i) => body.nodeStart + i * 2).filter(n => n < body.nodeStart + body.nodeCount);
    this.qBody = [0, 0, 0, 1];
    this.u = new Float64Array(NU);
    this.drive = drive ? new SoftDrive(car) : null; this.wW = [0, 0, 0]; // body angular velocity, world (brake reference)
    this.state = { t: 0, p: new Float64Array(3), q: new Float64Array(4), v: new Float64Array(3), w: new Float64Array(3), comB: new Float64Array(3), tank: { m: 0, T: 0, p: 0 }, valve: this.prop.valve, armed: false, phase: 'deck', drive: { brake: 0, steer: 0 } };
    this.log = { t: [], u_cmd: [], valve: [], thrust: [], tank: [], pc: [], state: [], comB: [], phase: [], armed: [], drive_cmd: [], drive: [], wheel_spin: [] };
    this.events = [];
    this.edgeT = null; this.armT = null; this.touchT = null; this.strikeT = null; this.ignT = null; this.kerbT = null; this.stopT = null; this.stopP = null;
    this.td = null; this.hold = null; this.peakAccel = 0; this._prevV = null;
    this.maxTiltAfterTd = 0; this.strikeInHold = false; this.wheelOnly = true;
    this._A = new Float64Array(9);
  }

  /**
   * Put the kit on the car: writes kit-augmented inverse masses into sim.w (fill = gas fraction) and fits its
   * rear coil-overs (compliance of the rear springs / rear_spring_scale, from the lattice's, so repeat calls are safe)
   * and rear control arms (give x rear_arm_give_scale on top of the run's `joints` factor, as CrashSim sets it).
   */
  applyMass(sim, fill = 1) {
    const { dry, gas } = this.kit, m = this.mass, N = sim.N;
    for (let n = 0; n < N; n++) { m[n] = this.base[n] + dry[n] + gas[n] * fill; if (sim.alive[n]) sim.w[n] = 1 / m[n]; }
    const B = this.car.beams, s = this.rearSpringScale, J = sim.sc.joints ?? 1, ga = this.rearArmScale;
    for (const b of this.rearSprings) sim.comp[b] = B.comp[b] / s;
    for (const b of this.rearArms) sim.tear[b] = B.tear[b] * J * ga;
    return this;
  }

  /** Install on a CrashSim: masses, actuator hook, controller reset. Call before sim.setup(). */
  attach(sim) {
    this.sim = sim;
    this.applyMass(sim, 1);
    sim.actuators.push(this);
    this.kerbBoxes = sim.boxes.filter(B => B.name === 'sidewalk' || B.name === 'median');
    this.ctrlEvery = Math.round(CONTROL_DT / sim.dt);
    this.logEvery = Math.round(1 / this.logHz / sim.dt);
    this.propDt = PROP_EVERY * sim.dt;
    this.controller?.reset?.({ task: this.task, seed: this.seed, spec: this.spec });
    return this;
  }

  // -------------------------------------------------------------------------------------------- per substep
  apply(sim, dt) {
    const s = sim.step;
    if (!this._init) this.initFrames(sim);
    if (s % PROP_EVERY === 0) this.detect(sim);
    if (s % this.ctrlEvery === 0) this.control(sim);
    if (s % PROP_EVERY === 0) {
      this.prop.step(this.u, this.propDt);
      this.orientMounts(sim);
      if (this.drainGas) this.drain(sim);
    }
    if (s % this.logEvery === 0) this.record(sim);
    // velocity impulses: Δv = F dt share / m along the mount's current thrust axis
    const v = sim.v, w = sim.w, F = this.prop.thrust;
    for (let i = 0; i < NU; i++) {
      if (F[i] <= 0) continue;
      const M = this.mounts[i], d = this.mDirW[i], J = F[i] * dt, nodes = M.nodes, sh = M.share;
      for (let k = 0; k < nodes.length; k++) {
        const n = nodes[k], j = J * sh[k] * w[n], o = n * 3;
        v[o] += j * d[0]; v[o + 1] += j * d[1]; v[o + 2] += j * d[2];
      }
    }
    if (this.drive) this.drive.apply(sim, dt, this.wW);
  }

  /**
   * Warm starts of the rotation fits: the car starts yawed by the launch heading (~166 deg), far outside what a
   * few fit iterations recover, so seed every frame with the placement rotation (B axes -> r, up, f).
   */
  initFrames(sim) {
    this._init = true;
    const L = sim.launch;
    const q0 = L ? matToQuat([L.right[0], 0, L.forward[0], L.right[1], 1, L.forward[1], L.right[2], 0, L.forward[2]]) : [0, 0, 0, 1];
    for (const q of [this.qBody, ...this.mq]) q.splice(0, 4, ...q0);
    this.orientMounts(sim, 30);
    this.estimate(sim, 40);
  }

  /** Thrust axes in world: rotation of each mount cluster (rest -> now). */
  orientMounts(sim, iters = 3) {
    const X = sim.x, R = this.car.nodes.rest, alive = sim.alive;
    for (let i = 0; i < NU; i++) {
      const M = this.mounts[i];
      let live = 0; for (let k = 0; k < M.nodes.length; k++) if (alive[M.nodes[k]]) live += M.share[k];
      let q = this.qBody;
      if (live > this.mMin[i]) { fitFrame(X, R, M.nodes, M.share, this.mRestC[i], this.mq[i], this.mc[i], iters, this._A); q = this.mq[i]; }
      quatToMat(q, _R);
      const d = M.dir, o = this.mDirW[i];
      o[0] = _R[0] * d[0] + _R[1] * d[1] + _R[2] * d[2]; o[1] = _R[3] * d[0] + _R[4] * d[1] + _R[5] * d[2]; o[2] = _R[6] * d[0] + _R[7] * d[1] + _R[8] * d[2];
    }
  }

  drain(sim) {
    const f = this.prop.m / this.prop.m0, g = this.kit.gas, m = this.mass, w = sim.w, alive = sim.alive;
    if (f === this._fill) return;
    this._fill = f;
    for (const n of this.gasNodes) { m[n] = this.base[n] + this.kit.dry[n] + g[n] * f; if (alive[n]) w[n] = 1 / m[n]; }
  }

  /** Vehicle node test: alive and still attached (detached parts are debris, not the vehicle). */
  isVehicle(sim, n) { return sim.alive[n] && !sim.pieceDetached[sim.nodePiece[n]]; }

  // -------------------------------------------------------------------------------------------- estimation
  estimate(sim, iters = 6) {
    const N = sim.N, X = sim.x, V = sim.v, Rr = this.car.nodes.rest, m = this.mass, S = this.state;
    let M = 0, px = 0, py = 0, pz = 0, vx = 0, vy = 0, vz = 0, bx = 0, by = 0, bz = 0;
    const att = this._att ||= new Uint8Array(N);
    for (let n = 0; n < N; n++) {
      if (!(att[n] = this.isVehicle(sim, n) ? 1 : 0)) continue;
      const k = n * 3, mn = m[n];
      M += mn; px += mn * X[k]; py += mn * X[k + 1]; pz += mn * X[k + 2]; vx += mn * V[k]; vy += mn * V[k + 1]; vz += mn * V[k + 2];
      bx += mn * Rr[k]; by += mn * Rr[k + 1]; bz += mn * Rr[k + 2];
    }
    px /= M; py /= M; pz /= M; vx /= M; vy /= M; vz /= M;
    S.p[0] = px; S.p[1] = py; S.p[2] = pz; S.v[0] = vx; S.v[1] = vy; S.v[2] = vz;
    S.comB[0] = bx / M; S.comB[1] = by / M; S.comB[2] = bz / M;
    this.massNow = M;
    // angular momentum and inertia about the COM (world) -> omega_world. Wheels count as point masses at their
    // hubs: their spin about the axles (~120 kg m^2/s at 70 km/h) is not body rotation and read as a 0.2 rad/s
    // phantom pitch rate otherwise.
    let Lx = 0, Ly = 0, Lz = 0, Ixx = 0, Iyy = 0, Izz = 0, Ixy = 0, Ixz = 0, Iyz = 0;
    const wheel = sim.isWheel, lump = this._lump ||= this.car.chunks.filter(c => c.wheel && c.nodeCount);
    const add = (mn, rx, ry, rz, ux, uy, uz) => {
      Lx += mn * (ry * uz - rz * uy); Ly += mn * (rz * ux - rx * uz); Lz += mn * (rx * uy - ry * ux);
      Ixx += mn * (ry * ry + rz * rz); Iyy += mn * (rx * rx + rz * rz); Izz += mn * (rx * rx + ry * ry);
      Ixy -= mn * rx * ry; Ixz -= mn * rx * rz; Iyz -= mn * ry * rz;
    };
    for (let n = 0; n < N; n++) {
      if (!att[n] || wheel[n]) continue;
      const k = n * 3;
      add(m[n], X[k] - px, X[k + 1] - py, X[k + 2] - pz, V[k] - vx, V[k + 1] - vy, V[k + 2] - vz);
    }
    for (const ch of lump) {
      let mw = 0; const c = [0, 0, 0], u = [0, 0, 0];
      for (let n = ch.nodeStart; n < ch.nodeStart + ch.nodeCount; n++) {
        if (!att[n]) continue;
        const k = n * 3, mn = m[n]; mw += mn;
        c[0] += mn * X[k]; c[1] += mn * X[k + 1]; c[2] += mn * X[k + 2]; u[0] += mn * V[k]; u[1] += mn * V[k + 1]; u[2] += mn * V[k + 2];
      }
      if (mw > 0) add(mw, c[0] / mw - px, c[1] / mw - py, c[2] / mw - pz, u[0] / mw - vx, u[1] / mw - vy, u[2] / mw - vz);
    }
    const om = solveSym([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]], [Lx, Ly, Lz]) || [0, 0, 0];
    // orientation: body chunk shape matching (rest frame = body frame B)
    const bn = this.bodyNodes, c = [0, 0, 0], A = this._A;
    if (!this._bodyRestC) { this._bodyW = Float64Array.from(bn, n => this.base[n]); this._bodyRestC = restCentroid(Rr, bn, this._bodyW); }
    fitFrame(X, Rr, bn, this._bodyW, this._bodyRestC, this.qBody, c, iters, A);
    const q = this.qBody;
    S.q[0] = q[0]; S.q[1] = q[1]; S.q[2] = q[2]; S.q[3] = q[3];
    quatToMat(q, _R); // w_body = R^T w_world
    S.w[0] = _R[0] * om[0] + _R[3] * om[1] + _R[6] * om[2]; S.w[1] = _R[1] * om[0] + _R[4] * om[1] + _R[7] * om[2]; S.w[2] = _R[2] * om[0] + _R[5] * om[1] + _R[8] * om[2];
    S.t = sim.time;
    const tk = this.prop.tank; S.tank.m = tk.m; S.tank.T = tk.T; S.tank.p = tk.p;
    S.tilt = Math.acos(Math.max(-1, Math.min(1, _R[4])));
    this.wW[0] = om[0]; this.wW[1] = om[1]; this.wW[2] = om[2];
    if (this.drive) { S.drive.brake = this.drive.brake; S.drive.steer = this.drive.steer; }
    return S;
  }

  /** Contacts (every propulsion step): touchdown and hull strikes on the street after the edge. */
  detect(sim) {
    if (this.edgeT === null) return;
    const N = sim.N, X = sim.x, contact = sim.contact, wheel = sim.isWheel;
    let touch = false, hull = false, kerb = -1;
    for (let n = 0; n < N; n++) {
      if (!contact[n] || X[n * 3 + 1] > STREET_Y || !this.isVehicle(sim, n)) continue;
      touch = true;
      if (kerb < 0 && this.kerbT === null && this.onKerb(X, n)) kerb = n;
      if (!wheel[n]) { hull = true; if (kerb >= 0 || this.kerbT !== null) break; }
    }
    this.onStreet = touch;
    const t = sim.time;
    if (touch && this.touchT === null) {
      this.touchT = t;
      const S = this.estimate(sim);
      this.td = { t, vs: -S.v[1], hs: Math.hypot(S.v[0], S.v[2]), tilt: S.tilt * deg, rate: Math.hypot(S.w[0], S.w[1], S.w[2]), gas: this.prop.m0 - this.prop.m, p: [S.p[0], S.p[2]], hull };
      this.events.push({ t, type: 'touchdown', hull });
    }
    if (hull && this.strikeT === null) { this.strikeT = t; this.events.push({ t, type: 'hull_strike' }); }
    if (kerb >= 0) { this.kerbT = t; this.events.push({ t, type: 'kerb', wheel: !!wheel[kerb], piece: this.car.pieces[sim.nodePiece[kerb]] }); }
    if (this.touchT !== null && hull && t - this.touchT <= (this.task?.success?.upright_hold ?? 1)) this.strikeInHold = true;
  }

  /** Is node n (in contact) on or against a sidewalk / median box? */
  onKerb(X, n) {
    const x = X[n * 3], y = X[n * 3 + 1], z = X[n * 3 + 2], m = KERB_MARGIN;
    for (const B of this.kerbBoxes) {
      const cy = Math.cos(B.yaw), sy = Math.sin(B.yaw), dx = x - B.c[0], dz = z - B.c[2];
      if (Math.abs(cy * dx - sy * dz) < B.e[0] + m && Math.abs(sy * dx + cy * dz) < B.e[2] + m && y < B.c[1] + B.e[1] + m) return true;
    }
    return false;
  }

  phaseOf(S) { return this.edgeT === null ? 'deck' : this.touchT === null ? 'air' : 'ground'; }

  control(sim) {
    const S = this.estimate(sim);
    if (this.edgeT === null && S.p[2] > DECK.halfWidth) { this.edgeT = sim.time; this.events.push({ t: sim.time, type: 'edge' }); }
    const rule = this.task?.arm || 'edge';
    const armEvent = rule === 'start' ? 0 : this.edgeT;
    S.armed = armEvent !== null && sim.time >= armEvent + this.armDelay;
    if (S.armed && this.armT === null) { this.armT = sim.time; this.events.push({ t: sim.time, type: 'armed' }); }
    S.phase = this.phaseOf(S);
    // landing bookkeeping
    if (this.touchT !== null) {
      const hold = this.task?.success?.upright_hold ?? 1;
      if (sim.time - this.touchT <= hold) this.maxTiltAfterTd = Math.max(this.maxTiltAfterTd, S.tilt * deg);
      // stopped: horizontal COM speed below the task's stop_speed (default 0.5 m/s) after the hold
      const vs = this.task?.success?.stop_speed ?? 0.5;
      if (this.stopT === null && sim.time - this.touchT >= hold && Math.hypot(S.v[0], S.v[2]) <= vs) { this.stopT = sim.time; this.stopP = [S.p[0], S.p[2]]; this.events.push({ t: sim.time, type: 'stopped' }); }
    }
    if (this._prevV && this.edgeT !== null) {
      const a = Math.hypot(S.v[0] - this._prevV[0], S.v[1] - this._prevV[1] + 9.81 * CONTROL_DT, S.v[2] - this._prevV[2]) / CONTROL_DT;
      this.peakAccel = Math.max(this.peakAccel, a / 9.81);
    }
    this._prevV = [S.v[0], S.v[1], S.v[2]];
    const u = this.u;
    if (S.armed && this.controller) {
      const out = this.controller.act(S, { t: sim.time, dt: CONTROL_DT, task: this.task });
      for (let i = 0; i < NU; i++) { const c = +out[i] || 0; u[i] = c < 0 ? 0 : c > 1 ? 1 : c; }
      this.drive?.set(out.length > NU ? out[NU] : 0, out.length > NU + 1 ? out[NU + 1] : 0);
    } else { u.fill(0); this.drive?.set(0, 0); }
  }

  record(sim) {
    const L = this.log, S = this.state, P = this.prop;
    // state at log time: the control estimate is up to one control period old -> refresh (cheap at 100 Hz)
    if (sim.step % this.ctrlEvery !== 0) { this.estimate(sim); S.phase = this.phaseOf(S); }
    L.t.push(r5(sim.time));
    L.u_cmd.push(Array.from(this.u, r4)); L.valve.push(Array.from(P.valve, r4)); L.thrust.push(Array.from(P.thrust, r1));
    L.tank.push([Math.round(P.m * 1e3) / 1e3, Math.round(P.T * 100) / 100, Math.round(P.p)]); L.pc.push(Math.round(P.pc));
    L.state.push([...S.p, ...S.q, ...S.v, ...S.w].map(r5)); L.comB.push(Array.from(S.comB, r5));
    L.phase.push(S.phase[0]); L.armed.push(S.armed ? 1 : 0);
    const D = this.drive;
    L.drive_cmd.push(D ? [r4(D.cmd.brake), r4(D.cmd.steer)] : [0, 0]); L.drive.push(D ? [r4(D.brake), r5(D.steer)] : [0, 0]);
    L.wheel_spin.push(D ? D.wheelSpins(sim, this.wW).map(r1) : [0, 0, 0, 0]);
    if (this.ignT === null) for (let i = 0; i < NU; i++) if (P.thrust[i] > 200) { this.ignT = sim.time; this.events.push({ t: sim.time, type: 'ignition' }); break; }
  }

  // -------------------------------------------------------------------------------------------- outputs
  /** `meta.propulsion` (coldgas-log@1). */
  propulsionLog() {
    const L = this.log;
    return {
      format: 'coldgas-log@1', hz: this.logHz, n: L.t.length, nozzles: NOZZLE_IDS, drive_channels: DRIVE.channels, ...L,
      mounts: this.spec.nozzles.map((z, i) => ({ id: z.id, group: z.group, nodes: Array.from(this.mounts[i].nodes), share: Array.from(this.mounts[i].share, v => Math.round(v * 1e6) / 1e6), exit: z.exit, mount: z.mount, dir: this.mounts[i].dir, exhaust: z.exhaust })),
      kit: { dry: +this.kitDry.toFixed(3), gas0: +this.prop.m0.toFixed(3), total0: +(this.base.reduce((s, v) => s + v, 0) + this.kitDry + this.prop.m0).toFixed(3) },
    };
  }

  /** Landing metrics against the task's success criteria + damage of the soft body. */
  summary(sim) {
    const crit = this.task?.success || {}, td = this.td;
    const hold = crit.upright_hold ?? 1;
    const holdDone = td && sim.time - td.t >= hold;
    const m = td ? {
      touchdown_t: +td.t.toFixed(3), touchdown_vertical_speed: +td.vs.toFixed(3), touchdown_horizontal_speed: +td.hs.toFixed(3),
      touchdown_tilt_deg: +td.tilt.toFixed(2), touchdown_angular_rate: +td.rate.toFixed(3), hull_strike: !!(td.hull || this.strikeInHold),
      hull_strike_any: this.strikeT !== null, max_tilt_hold_deg: +this.maxTiltAfterTd.toFixed(2), landing_point: td.p.map(v => +v.toFixed(2)),
      pad_distance: this.task?.pad ? +Math.hypot(td.p[0] - this.task.pad.center[0], td.p[1] - this.task.pad.center[2]).toFixed(2) : null,
      gas_used_touchdown_kg: +td.gas.toFixed(2),
      kerb: this.kerbT !== null, kerb_after_td: this.kerbT !== null ? +(this.kerbT - td.t).toFixed(3) : null,
      stopped: this.stopT !== null, stop_time: this.stopT !== null ? +(this.stopT - td.t).toFixed(3) : null,
      stop_distance: this.stopT !== null ? +Math.hypot(this.stopP[0] - td.p[0], this.stopP[1] - td.p[1]).toFixed(2) : null,
    } : { touchdown_t: null, pad_distance: null, hull_strike: this.strikeT !== null, kerb: this.kerbT !== null };
    m.gas_used_kg = +(this.prop.m0 - this.prop.m).toFixed(2);
    m.peak_accel_g = +this.peakAccel.toFixed(2);
    const fails = [];
    if (!td) fails.push('no touchdown');
    else {
      if (crit.max_vertical_speed !== undefined && td.vs > crit.max_vertical_speed) fails.push('vertical_speed');
      if (crit.max_horizontal_speed !== undefined && td.hs > crit.max_horizontal_speed) fails.push('horizontal_speed');
      if (crit.max_tilt_deg !== undefined && td.tilt > crit.max_tilt_deg) fails.push('tilt');
      if (crit.max_angular_rate !== undefined && td.rate > crit.max_angular_rate) fails.push('angular_rate');
      if (crit.max_pad_distance !== undefined && (!this.task?.pad || Math.hypot(td.p[0] - this.task.pad.center[0], td.p[1] - this.task.pad.center[2]) > crit.max_pad_distance)) fails.push('pad_distance');
      if (crit.hull_strike === false && m.hull_strike) fails.push('hull_strike');
      if (crit.max_tilt_deg !== undefined && this.maxTiltAfterTd > crit.max_tilt_deg) fails.push('upright_hold');
      if (!holdDone) fails.push('hold_incomplete');
      // stop tasks (spec/tasks/escape-land-stop): come to rest within stop_within s, never touch a kerb
      if (crit.kerb === false && this.kerbT !== null) fails.push('kerb');
      if (crit.stop_speed !== undefined && (this.stopT === null || (crit.stop_within !== undefined && this.stopT - td.t > crit.stop_within))) fails.push('not_stopped');
    }
    m.success = fails.length === 0; m.fails = fails;
    // damage
    let tears = 0, maxPl = 0, dmg = 0;
    for (const t of sim.tearTime) if (t < 1e8) tears++;
    for (let b = 0; b < sim.NB; b++) if (sim.plastic[b] > maxPl) maxPl = sim.plastic[b];
    for (let n = 0; n < sim.N; n++) dmg += sim.damage[n];
    const ev = sim.events.breaks;
    const damage = {
      detached: ev.filter(e => e.type === 'detach').map(e => e.piece), glass: ev.filter(e => e.type === 'glass' || e.type === 'lens' || e.type === 'crack').map(e => e.group),
      torn_tets: tears, max_plastic: +maxPl.toFixed(4), damage_sum: +dmg.toFixed(2), particles: sim.np,
    };
    return { controller: this.controllerName, task: this.task?.id, pad: this.task?.pad ?? null, armDelay: this.armDelay, events: this.events.slice().sort((a, b) => a.t - b.t), metrics: m, damage };
  }
}
