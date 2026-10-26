// One rigid-twin episode from the command line, written as a coldgas-traj@1 file (same as the Python bench writes).
// usage: node tools/flight/run.mjs task=escape-soft-landing controller=pd seed=0 [preset=nose] [out=renders/x.json]
//        controller=replay traj=<file>   controller=mlp policy=<file>   controller='{"name":"pd","kv":6}'
import fs from 'fs';
import path from 'path';
import { parseArgs } from '../lib/args.mjs';
import { runEpisode } from '../../src/flight/run.js';
import { loadController } from '../../src/flight/controllers/index.js';
import { specHash } from '../../src/flight/spechash.js';

const a = parseArgs();
const task = a.task || 'escape-soft-landing', seed = +(a.seed ?? 0);
let cfg = a.controller || 'zero';
cfg = cfg.startsWith('{') ? JSON.parse(cfg) : { name: cfg };
if (a.traj) cfg.file = a.traj;
if (a.policy) cfg.file = a.policy;
const controller = await loadController(cfg);
const t0 = performance.now();
const { traj, metrics } = runEpisode({ task, controller, seed, preset: a.preset ?? null, specHash: await specHash() });
traj.controller = { name: cfg.name, ...Object.fromEntries(Object.entries(cfg).filter(([k]) => !['name', 'traj', 'policy'].includes(k))) };
const ms = performance.now() - t0;
if (a.out) { fs.mkdirSync(path.dirname(path.resolve(a.out)), { recursive: true }); fs.writeFileSync(a.out, JSON.stringify(traj)); }
console.log(JSON.stringify({ task, controller: cfg.name, seed, preset: traj.preset, ms: Math.round(ms), out: a.out ?? null, metrics,
  ...(controller.statistics ? { controllerStats: { ...controller.statistics, planMs: controller.planMs } } : {}) }));
