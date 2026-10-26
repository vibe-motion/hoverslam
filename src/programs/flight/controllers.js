// Controller factory for soft-body flights (tools/sim/flight.mjs) and viewers. Spec strings:
//   ap | ap:{json gains}       this folder's landing autopilot (./autopilot.js)
//   zero | pd | pd:{json}      flight-twin registry (src/flight/controllers: createController)
//   replay:<traj.json>         open-loop replay of a coldgas-traj@1 trajectory's u_cmd (Python MPC / PPO runs)
//   mlp:<policy.json>          coldgas-policy@1 MLP policy (twin registry)
//   {json config}              any twin registry config, e.g. {"name":"pd","kv":6}
// `load(path)` is injected (Node: fs, browser: fetch) so this module stays platform-neutral. Without the twin
// registry (older trees) zero / replay fall back to the local implementations below.
import { NU, CONTROL_DT, TRAJ_FORMAT } from '../../flight/contract.js';
import { Autopilot } from './autopilot.js';

export class ZeroController {
  constructor() { this.name = 'zero'; this.u = new Float64Array(NU); }
  reset() {}
  act() { return this.u; }
}

/** Open-loop u_cmd replay (row k applied from t_k = k·dt, as the twin's replay controller). */
export class ReplayController {
  constructor(traj, name = 'replay') {
    if (traj.format !== TRAJ_FORMAT) throw new Error(`replay: need ${TRAJ_FORMAT}, got ${traj.format}`);
    this.traj = traj; this.name = name; this.dt = traj.dt || CONTROL_DT; this.u = new Float64Array(NU);
  }
  reset() {}
  act(s) {
    const r = this.traj.u_cmd[Math.round(s.t / this.dt)];
    for (let i = 0; i < NU; i++) this.u[i] = r ? +r[i] || 0 : 0;
    return this.u;
  }
}

/**
 * @param spec  string (see header) or config object
 * @param load  async path -> parsed JSON
 * @returns {Promise<{controller, traj?: object}>} traj = the replayed trajectory (its init drives the launch)
 */
export async function makeController(spec, { load } = {}) {
  const reg = await twinRegistry();
  const named = (c, name) => { c.name = name; return c; };
  if (typeof spec === 'string') {
    if (spec === 'ap' || spec.startsWith('ap:')) return { controller: new Autopilot(spec.length > 3 ? JSON.parse(spec.slice(3)) : {}) };
    const m = spec.match(/^(replay|mlp):(.+)$/);
    if (m) {
      const obj = await load(m[2]), file = m[2].split('/').pop().replace(/\.json$/, '');
      if (m[1] === 'replay') return { controller: named(reg ? reg.createController({ name: 'replay', traj: obj }) : new ReplayController(obj), `replay:${file}`), traj: obj };
      if (!reg) throw new Error('mlp controllers need the flight-twin registry (src/flight/controllers)');
      return { controller: named(reg.createController({ name: 'mlp', policy: obj }), `mlp:${file}`) };
    }
    const j = spec.match(/^(\w+):(\{.*\})$/);
    spec = j ? { name: j[1], ...JSON.parse(j[2]) } : spec.startsWith('{') ? JSON.parse(spec) : spec;
  }
  const name = typeof spec === 'string' ? spec : spec.name;
  if (!reg) { if (name === 'zero') return { controller: new ZeroController() }; throw new Error(`controller ${name}: flight-twin registry not available`); }
  return { controller: named(await reg.loadController(spec), typeof spec === 'string' ? spec : JSON.stringify(spec)) };
}

let _reg;
async function twinRegistry() {
  if (_reg !== undefined) return _reg;
  try { _reg = await import('../../flight/controllers/index.js'); } catch (e) { if (!/Cannot find|Failed to fetch|ERR_MODULE_NOT_FOUND|Failed to load/.test(String(e))) console.warn(e); _reg = null; }
  return _reg;
}
