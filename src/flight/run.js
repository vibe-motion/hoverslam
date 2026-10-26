// One closed-loop episode on the JS twin: controller.act(state) every control step, optional trajectory.
// Same loop as python/coldgas/bench.py run_episode (parity: identical trajectories for pd/zero/replay).
import { ColdGasEnv } from './env.js';
import { createController } from './controllers/index.js';
import { TrajRecorder } from './traj.js';

export function runEpisode({ task, controller = 'zero', seed = 0, preset = null, init = null, record = true, specHash = null, vehicle = null }) {
  const env = new ColdGasEnv(task, { seed, preset, vehicle });
  const ctrl = typeof controller === 'object' && controller.act ? controller : createController(controller);
  env.reset(seed, init ? { init } : {});
  ctrl.reset({ task: env.task, seed });
  const cfg = typeof controller === 'string' ? { name: controller } : controller.act ? { name: controller.name, ...controller.config } : controller;
  const rec = record ? new TrajRecorder({ task: env.task.id, controller: stripObjects(cfg), seed, preset: env.init.preset, init: env.init }) : null;
  let info = env.info(), done = false, metrics = null;
  while (!done) {
    const s = env.state;
    if (rec) rec.sample(env.twin);
    const u = ctrl.act(s, info);
    const [, , term, trunc, inf] = env.step(u);
    if (rec) rec.applied(env.twin.u, env.twin.ud);
    info = inf; done = term || trunc;
    if (done) metrics = inf.metrics;
  }
  if (rec) rec.sample(env.twin);
  const planningTrace = ctrl.config?.capturePlans === true ? ctrl.planningTrace : null;
  const traj = rec ? rec.finish({ events: env.twin.events, metrics, spec_hash: specHash }) : null;
  if (traj && planningTrace) traj.planningTrace = planningTrace;
  if (traj && ctrl.commandLimits) traj.commandLimits = structuredClone(ctrl.commandLimits);
  return { metrics, events: env.twin.events, traj };
}
// trajectories record the controller config without embedded traj/policy blobs
function stripObjects(cfg) { const { traj, policy, ...rest } = cfg; return { ...rest, ...(traj ? { traj: '<embedded>' } : {}), ...(policy ? { policy: '<embedded>' } : {}) }; }
