// Thrust-assisted escape on the soft body: a launch preset with the cold-gas kit, closed loop through a
// Controller, written as public/sims/flight-<out>.sim (recording + meta.propulsion log + meta.flight summary).
// usage: node tools/sim/flight.mjs preset=flat controller=ap out=soft [dur=6 hz=100 task=escape-soft-landing
//        armDelay=0 seed= speed= spinScale=1 gas=1 logHz=100 level=0]
//   controller: ap | ap:{gains} | zero | pd | pd:{json} | replay:<traj.json> | mlp:<policy.json> | {json config}
//               (src/programs/flight/controllers.js; all but ap come from the flight-twin registry)
//   level=1: levelled suspension + free-spinning wheels (CrashSim `level`, opt-in model update; settles itself)
//   outDir=<dir>: write <dir>/flight-<out>.sim (+ .json with meta.flight) instead of public/sims (scratch runs)
//   mppi:{...} / mpc:{...}: closed-loop MPPI (residual on pd) / pure MPC (src/flight/mpc) on the soft body (twin
//               registry); plan count / time (/ rollouts) go into meta.flight.controllerStats
//   replay: the launch (preset, speed, angle, spin, lead) is taken from the trajectory's `init` so the open-loop
//           commands meet the same initial condition the twin saw (preset= is then ignored)
//   launch=<traj.json>: the same for any controller (e.g. a closed-loop MPC on the launch of a twin episode: the twin's
//           task randomisation of speed / angle / spin is drawn from its seed, the soft body's seed= does not draw it;
//           or zero: the car of that drive-in, unpowered); spin=p,y,r overrides the kerb kick (rad/s, car axes;
//           0,0,0 = it just drives off)
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';
import { loadCarNode } from '../lib/car.mjs';
import { parseArgs } from '../lib/args.mjs';
import { readSettled, writeSim } from '../lib/sims.mjs';
import { repo } from '../lib/paths.mjs';
import { CrashSim } from '../../src/softbody/solver.js';
import { RecordingEncoder, finalMeta, packRecording } from '../../src/softbody/recording.js';
import { ThrusterRig } from '../../src/softbody/actuators.js';
import { makeController } from '../../src/programs/flight/controllers.js';
import { PRESETS } from '../../src/scene/layout.js';
import { loadTask } from '../../src/flight/tasks.js';

const args = parseArgs();
const ctlSpec = args.controller || 'ap';
const load = async p => JSON.parse(fs.readFileSync(p.startsWith('/') && !fs.existsSync(p) ? repo(p) : p, 'utf8'));
const { controller, traj } = await makeController(ctlSpec, { load });
const launchOf = t => (t?.init?.type === 'launch' ? t.init : null);
const init0 = launchOf(traj) || (args.launch ? launchOf(await load(args.launch)) : null);
if (args.launch && !init0) throw new Error(`launch=${args.launch}: no launch init in that trajectory`);
const init = init0 && args.spin ? { ...init0, spin: args.spin.split(',').map(Number) } : init0;
const preset = init?.preset || args.preset || 'flat', P = PRESETS[preset];
if (!P) throw new Error(`unknown preset ${preset}`);
const name = `flight-${args.out || `${preset}-${ctlSpec.split(/[:{]/)[0]}`}`;
const task = loadTask(args.task || (typeof traj?.task === 'string' ? traj.task : null) || 'escape-soft-landing');
const car = await loadCarNode({ h: 0.065 });
const settled = readSettled('public/assets/settled-coldgas.bin', car);
const spinScale = +(args.spinScale || 1);
const sc = {
  ...P, spin: P.spin.map(s => s * spinScale), ...(args.speed ? { speed: +args.speed } : {}), ...(args.seed ? { seed: +args.seed } : {}),
  ...(init ? { speed: init.speed, angle: init.angle, spin: init.spin, lead: init.lead ?? P.lead } : {}),
  strength: +(args.s || P.strength || 1), joints: +(args.j || P.joints || 1), recordHz: +(args.hz || 100), duration: +(args.dur || 6),
  ...(+args.level ? { level: true } : {}),
};
const sim = new CrashSim(car, sc);
// registry controllers configured by JSON (e.g. mppi:{...}) are named by their JSON: show the short name, keep the
// settings in meta.flight.controllerConfig
const ctlConfig = typeof controller.name === 'string' && controller.name.startsWith('{') ? JSON.parse(controller.name) : null;
const rig = new ThrusterRig(car, { controller, controllerName: ctlConfig?.name || controller.name || ctlSpec, task, armDelay: +(args.armDelay || 0), logHz: +(args.logHz || 100), seed: sc.seed });
rig.attach(sim);
if (args.gas) { rig.prop.m = rig.prop.m0 * +args.gas; rig.prop.step(new Float64Array(10), 0); rig.applyMass(sim, +args.gas); } // partly filled tank
sim.setup(sc.level ? null : settled); // the cached pose is the unlevelled one
const enc = new RecordingEncoder(car);
const frames = [];
let k = 0;
const t0 = Date.now();
frames.push(enc.encode(sim, true));
sim.run(sc.duration, s => {
  k++; frames.push(enc.encode(s, k % 4 === 0));
  if (k % 25 === 0) process.stdout.write(`\r${name} t=${s.time.toFixed(2)} y=${rig.state.p[1].toFixed(2)} vy=${rig.state.v[1].toFixed(2)} tilt=${(rig.state.tilt * 57.3 || 0).toFixed(1)} gasLeft=${rig.prop.m.toFixed(1)} ${rig.state.phase} ${((Date.now() - t0) / 1000).toFixed(0)}s   `);
});
const cstats = controller.plans != null ? { plans: controller.plans, plan_s: +(controller.planMs / 1000).toFixed(2), ...(controller.evals != null ? { evals: controller.evals } : {}), ...(controller.statistics || {}) } : null;
const flight = { ...rig.summary(sim), ...(controller.commandLimits ? { commandLimits: structuredClone(controller.commandLimits) } : {}), ...(ctlConfig ? { controllerConfig: ctlConfig } : {}), ...(cstats ? { controllerStats: cstats } : {}),
  ...(ctlConfig?.capturePlans === true && Array.isArray(controller.planningTrace) ? { planningTrace: controller.planningTrace } : {}),
  preset, spinScale, init: init || null, replayOf: traj ? { file: ctlSpec.slice(7), producer: traj.producer, controller: traj.controller, seed: traj.seed, metrics: traj.metrics } : null, computeSeconds: (Date.now() - t0) / 1000, settled: sc.level ? 'levelled (own settle)' : 'settled-coldgas.bin' };
const meta = finalMeta(sim, enc, { name, label: `${P.label} · ${flight.controller}`, propulsion: rig.propulsionLog(), flight });
let size;
if (args.outDir) { // scratch output outside public/sims
  fs.mkdirSync(args.outDir, { recursive: true });
  const gz = zlib.gzipSync(Buffer.from(packRecording(meta, frames)), { level: 6 });
  fs.writeFileSync(path.join(args.outDir, `${name}.sim`), gz);
  fs.writeFileSync(path.join(args.outDir, `${name}.json`), JSON.stringify(flight, null, 1));
  size = { gz: gz.length };
} else size = writeSim(name, meta, frames);
console.log(`\n${name}: ${frames.length} frames gz ${(size.gz / 1e6).toFixed(1)} MB in ${flight.computeSeconds.toFixed(0)} s`);
console.log(JSON.stringify({ metrics: flight.metrics, damage: flight.damage, events: flight.events.map(e => `${e.type}@${e.t.toFixed(2)}`) }, null, 1));
