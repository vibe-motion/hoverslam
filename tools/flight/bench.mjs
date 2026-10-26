// JS twin benchmark: the same config schema and outputs as `python -m coldgas.bench` (experiments/README.md), so
// JS controllers (mppi in particular, ~20x faster than the numpy one) are scored the same way. Episodes run on
// worker_threads (built in, no dependencies).
// usage: node tools/flight/bench.mjs experiments/configs/baseline-soft-landing.json [workers=8] [seeds=0:3|0,1]
//        [out=experiments/runs/<name>-js/<stamp>] [only=<label>,...]
import fs from 'fs';
import os from 'os';
import path from 'path';
import { Worker, isMainThread, parentPort, workerData } from 'worker_threads';
import { fileURLToPath } from 'url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');

// same aggregates as coldgas.bench summarize() / table()
const KEYS = ['touchdown_vertical_speed', 'touchdown_horizontal_speed', 'touchdown_tilt_deg', 'touchdown_angular_rate', 'peak_accel_g', 'gas_used_kg', 'pad_distance', 'hold_time', 'rms_height_error', 'return'];
function stats(xs) {
  xs = xs.filter(x => typeof x === 'number' && Number.isFinite(x)).sort((p, q) => p - q);
  return xs.length ? { mean: xs.reduce((s, x) => s + x, 0) / xs.length, median: xs[xs.length >> 1], min: xs[0], max: xs[xs.length - 1], n: xs.length } : null;
}
function summarize(rows, labels) {
  const out = { controllers: {} };
  for (const lab of labels) {
    const rs = rows.filter(r => r.label === lab);
    if (!rs.length) continue;
    const m = rs.map(r => r.metrics), reasons = {}, per = {};
    for (const x of m) reasons[x.reason] = (reasons[x.reason] || 0) + 1;
    for (const r of rs) (per[r.preset ?? 'task'] ||= []).push(!!r.metrics.success);
    out.controllers[lab] = {
      n: rs.length, success_rate: m.filter(x => x.success).length / m.length, hull_strike_rate: m.filter(x => x.hull_strike).length / m.length, reasons,
      metrics: Object.fromEntries(KEYS.filter(k => m.some(x => x[k] != null)).map(k => [k, stats(m.map(x => x[k]))])),
      per_preset_success: Object.fromEntries(Object.keys(per).sort().map(p => [p, per[p].filter(Boolean).length / per[p].length])),
      wall_s_per_episode: rs.reduce((s, r) => s + r.wall_s, 0) / rs.length,
    };
  }
  return out;
}
function table(summary) {
  const C = summary.controllers, presets = [...new Set(Object.values(C).flatMap(c => Object.keys(c.per_preset_success)))].sort();
  const head = ['controller', 'n', 'success', 'hull strike', 'vy_td mean', 'tilt_td mean', 'gas kg', 'return', ...presets, 's/ep'];
  const f = (s, k) => (s.metrics[k] ? s.metrics[k].mean.toFixed(2) : '-'), pc = x => `${(100 * x).toFixed(0)}%`;
  return [`| ${head.join(' | ')} |`, `|${'---|'.repeat(head.length)}`, ...Object.entries(C).map(([lab, s]) => `| ${[lab, s.n, pc(s.success_rate), pc(s.hull_strike_rate), f(s, 'touchdown_vertical_speed'),
    f(s, 'touchdown_tilt_deg'), f(s, 'gas_used_kg'), f(s, 'return'), ...presets.map(p => (p in s.per_preset_success ? pc(s.per_preset_success[p]) : '-')), s.wall_s_per_episode.toFixed(1)].join(' | ')} |`)].join('\n');
}

if (!isMainThread) { // ------------------------------------------------------------------------------ worker
  const { runEpisode } = await import('../../src/flight/run.js');
  const { loadController } = await import('../../src/flight/controllers/index.js');
  parentPort.on('message', async job => {
    const t0 = performance.now();
    const ctrl = await loadController(job.controller);
    const r = runEpisode({ task: job.task, controller: ctrl, seed: job.seed, preset: job.preset, specHash: workerData.hash });
    const wall = (performance.now() - t0) / 1000;
    if (job.traj) { r.traj.controller = job.shown; fs.writeFileSync(job.traj, JSON.stringify(r.traj)); }
    parentPort.postMessage({ label: job.label, controller: job.shown, task: job.task, preset: r.traj.preset, seed: job.seed, metrics: r.metrics, wall_s: wall,
      plans: ctrl.plans ?? null, plan_s: ctrl.planMs != null ? ctrl.planMs / 1000 : null, evals: ctrl.evals ?? null, traj: job.traj ? path.relative(job.out, job.traj) : null });
  });
} else { // ---------------------------------------------------------------------------------------- main
  const { parseArgs } = await import('../lib/args.mjs');
  const { specHash } = await import('../../src/flight/spechash.js');
  const a = parseArgs(), cfgPath = Object.keys(a).find(k => a[k] === true && k.endsWith('.json'));
  if (!cfgPath) throw new Error('usage: node tools/flight/bench.mjs <config.json> [workers=] [seeds=] [out=] [only=]');
  const cfg = JSON.parse(fs.readFileSync(cfgPath, 'utf8')), name = cfg.name || path.basename(cfgPath, '.json');
  const d = new Date(), p2 = x => String(x).padStart(2, '0');
  const stamp = `${d.getFullYear()}${p2(d.getMonth() + 1)}${p2(d.getDate())}-${p2(d.getHours())}${p2(d.getMinutes())}${p2(d.getSeconds())}`;
  const out = path.resolve(a.out || path.join(ROOT, 'experiments', 'runs', `${name}-js`, stamp));
  fs.mkdirSync(path.join(out, 'traj'), { recursive: true });
  const hash = await specHash();
  const seedSpec = a.seeds ? (a.seeds.includes(':') ? (([s, e]) => ({ start: s, count: e - s }))(a.seeds.split(':').map(Number)) : a.seeds.split(',').map(Number)) : cfg.seeds ?? [0];
  const seeds = Array.isArray(seedSpec) ? seedSpec : Array.from({ length: seedSpec.count }, (_, i) => (seedSpec.start ?? 0) + i);
  const presets = cfg.presets?.length ? cfg.presets : [null], record = cfg.record ?? 'all';
  const only = a.only ? new Set(a.only.split(',')) : null;
  const ctrls = cfg.controllers.filter(c => !only || only.has(c.label ?? c.name));
  fs.writeFileSync(path.join(out, 'config.json'), JSON.stringify({ ...cfg, spec_hash: hash, seeds_resolved: seeds, stamp, producer: 'js' }, null, 1));
  const resolve = c => { // policy / traj paths relative to the config or the repo
    const r = { ...c };
    for (const k of ['policy', 'traj']) if (typeof r[k] === 'string') {
      const f = [path.resolve(path.dirname(cfgPath), r[k]), path.resolve(ROOT, r[k])].find(fs.existsSync);
      r[k] = JSON.parse(fs.readFileSync(f, 'utf8'));
    }
    return r;
  };
  const jobs = [];
  for (const c of ctrls) {
    const { label: lab0, ...cc } = c, label = lab0 ?? c.name;
    const shown = Object.fromEntries(Object.entries(cc).map(([k, v]) => [k, k === 'policy' || k === 'traj' ? '<file>' : v]));
    presets.forEach(preset => seeds.forEach((seed, si) => {
      const keep = record === 'all' || (record === 'first' && si === 0);
      jobs.push({ task: cfg.task, controller: resolve(cc), shown, label, preset, seed, out,
        traj: keep ? path.join(out, 'traj', `${label}__${preset ?? 'task'}__s${seed}.json`) : null });
    }));
  }
  const nw = Math.min(+(a.workers || cfg.workers || Math.max(1, os.cpus().length - 1)), jobs.length);
  const rows = [], fj = fs.openSync(path.join(out, 'episodes.jsonl'), 'w'), t0 = performance.now();
  await new Promise((done, fail) => {
    let next = 0, live = nw;
    for (let w = 0; w < nw; w++) {
      const wk = new Worker(fileURLToPath(import.meta.url), { workerData: { hash } });
      const feed = () => { if (next < jobs.length) wk.postMessage(jobs[next++]); else { wk.terminate(); if (--live === 0) done(); } };
      wk.on('message', r => { rows.push(r); fs.writeSync(fj, JSON.stringify(r) + '\n'); process.stderr.write(`\r${rows.length}/${jobs.length} episodes`); feed(); });
      wk.on('error', fail);
      feed();
    }
  });
  fs.closeSync(fj);
  process.stderr.write('\n');
  const summary = { ...summarize(rows, [...new Set(jobs.map(j => j.label))]), name, task: cfg.task, spec_hash: hash, episodes: rows.length, wall_s: (performance.now() - t0) / 1000, stamp, producer: 'js' };
  fs.writeFileSync(path.join(out, 'summary.json'), JSON.stringify(summary, null, 1));
  console.log(table(summary));
  console.log(`\n-> ${path.relative(ROOT, out)}`);
}
