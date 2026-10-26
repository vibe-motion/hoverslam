// Flight viewer (program "flight"): thrust-assisted escapes off the viaduct. Two sources:
//   ?program=flight&run=soft            soft-body recording public/sims/flight-<run>.sim (tools/sim/flight.mjs):
//                                       deformable car, debris, dust, plumes from the thrust log, director edit
//   ?program=flight&traj=/experiments/runs/.../x.json
//                                       rigid replay of a coldgas-traj@1 trajectory (Python / JS twins): the
//                                       undeformed car posed per spec/README §1, plumes from the logged thrust
// Other params: mode=director|free|follow, T=<film s>, paused, hud=0 (no telemetry in films),
//   look-dev: devcam=px,py,pz,tx,ty,tz[,fov] (relative to the car's COM) devt=<sim s>: film T -> sim devt + T.
//   devfollow: the devcam moves with the car; devtrack: only horizontally (fixed height: a descent crosses the frame);
//   devpan: the camera stays put and only its look point follows the car horizontally (a pan at fixed tilt), e.g. two
//   flights from the same launch shot with the same camera for a side-by-side (直到用计算机模拟 04 / 05).
// Film: node tools/film/film.mjs program=flight q=run=soft out=renders/x.mp4
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Program } from '../../app/program.js';
import { Transport } from '../../app/transport.js';
import { PlaybackView } from '../../playback/view.js';
import { applyPose } from '../../cinema/sequence.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { Plumes, trackFromRecording, trackFromTrajectory, nozzleNominal } from '../../render/plumes.js';
import { quatToMat } from '../../core/quat.js';
import { TASKS, PROPULSION, WORLD } from '../../core/spec.js';
import { NU } from '../../flight/contract.js';
import { loadTask } from '../../flight/tasks.js';
import { PRESETS } from '../../scene/layout.js';
import { FlightLog } from './timeline.js';
import { FlightEdit } from './edit.js';
import { FlightHUD } from './hud.js';
import { FlightTrajectories } from './trajectories.js';
import { FLIGHT_UI, FLIGHT_CSS, FLIGHT_GROUPS } from './ui.js';

/** Known soft-body runs (shown when their .sim exists); ?run= may name any other flight-*.sim. */
const RUNS = [
  { id: 'nonconvex-stop', label: '落地刹停.非凸', sub: '35% 最小油门 · 接地倾角 ≤ 5° · 触地后全刹车' },
  { id: 'nonconvex-pad', label: '定点降落.非凸', sub: '35% 最小油门 · 靶心误差 ≤ 1.5 m · 触地水平速度 ≤ 0.5 m/s' },
  // Original upstream examples remain available whenever their recordings are generated.
  { id: 'soft', label: '软着陆', sub: '车头俯冲 · 自动驾驶 ap · 刹停' },
  { id: 'recover', label: '翻滚改出', sub: '倒扣车顶 · 自动驾驶 ap · 刹停' },
  { id: 'hard', label: '硬着陆', sub: '倒扣车顶 · 点火延迟 0.6 s' },
  { id: 'mpc-stop-nose', label: 'MPC · 落地边刹边转', sub: '车头俯冲 · 纯 MPC 规划刹车与转向，转 43° 刹停' },
  { id: 'mpc-stop-corner', label: 'MPC · 翻滚改出 · 刹停', sub: '角部翻滚 · 纯 MPC，避开路缘刹停' },
  { id: 'mpc-roll', label: 'MPC · 跳跃横滚 360°', sub: '四轮平拍 · 纯 MPC（无 PD），落地后刹车转向' },
  { id: 'pd-nose', label: 'pd · 着陆后失控', sub: '车头俯冲 · 孪生 pd（无航向控制、不用刹车）' },
  { id: 'pd-roof', label: 'pd · 着陆后失控', sub: '倒扣车顶 · 孪生 pd（无航向控制、不用刹车）' },
  { id: 'replay-mppi-nose', label: 'MPPI 开环回放', sub: '车头俯冲 · 孪生 MPPI 轨迹开环回放' },
  { id: 'mppi-nose', label: 'MPPI 闭环 · 软着陆', sub: '车头俯冲 · JS MPPI 在软体上实时重规划（无刹车）' },
  { id: 'mppi-roof', label: 'MPPI 闭环 · 落在路缘', sub: '倒扣车顶 · JS MPPI 软触地，但正落在路缘上（无刹车）' },
  { id: 'two-costs-first-nose3', label: 'MPC · 托着往下放', sub: '车头俯冲 · 代价无用气项、下落速度计价（gas-two-costs）' },
  { id: 'two-costs-second-nose3', label: 'MPC · 自杀式点火', sub: '车头俯冲 · 代价加用气项（gas-two-costs）' },
];
const runName = id => {
  const name = id.startsWith('flight-') ? id.slice(7) : id;
  return name === 'dual-recovery35-flat' ? 'nonconvex-pad' : name; // Preserve bookmarks to the accepted pad demo.
};
const ctlLabel = c => (!c ? '' : c === 'ap' ? '自动驾驶 ap' : c === 'pd' ? '孪生 pd' : c === 'mppi' ? 'MPPI 闭环' : c === 'mpc' ? '纯 MPC（无 PD）' : c.startsWith('replay:') ? `开环回放 ${c.slice(7)}` : c);

export default class FlightProgram extends Program {
  static meta = { id: 'flight', title: 'JetCar Mini · 冷气推力着陆' };

  async init() {
    const { stage, ui, assets, params } = this.ctx;
    this.params = params;
    document.head.insertAdjacentHTML('beforeend', `<style>${FLIGHT_CSS}</style>`);
    ui.loadText('载入天空');
    const env = await assets.sky();
    ui.loadText('搭建高架场景');
    this.world = assets.world();
    ui.loadText('载入车模');
    await assets.gltf();
    ui.loadText('构建软体晶格');
    const car = (this.car = await assets.car());
    this.view = new PlaybackView(this.ctx, { car, world: this.world, env });
    this.plumes = new Plumes(stage.scene, stage);
    this.view.extraProbeHidden = this.plumes.objects;
    this.nom = nozzleNominal();
    this.hud = new FlightHUD();
    this.trajectories = new FlightTrajectories({ root: ui.root, camera: stage.camera });
    this.visualOptions = { trail: true, planning: false, exploration: false };
    this.filmHud = params.get('hud') !== '0';
    ui.mount(FLIGHT_UI);
    this.controls = new OrbitControls(stage.camera, stage.renderer.domElement);
    this.controls.enableDamping = true; this.controls.enabled = false;
    this.mode = params.get('mode') || 'director';
    this.speed = 1; this.playing = !params.has('paused');
    this.looping = false;
    this.T = +(params.get('T') || 0); this.simT = 0;
    this.setupUI();
    await this.world.ready;
    ui.loadText('载入飞行记录');
    if (params.has('traj')) await this.loadTrajectory(params.get('traj'));
    else {
      const avail = await this.listRuns();
      await this.loadRun(params.get('run') || avail[0]?.id || 'nonconvex-pad');
    }
    const self = this;
    this.film = {
      get total() { return self.edit?.total ?? 0; },
      render: (T, opts) => this.renderFilm(T, opts),
      describe: () => ({ ...this.edit?.describe(), source: this.source }),
    };
    ui.setTitle(FlightProgram.meta.title);
  }

  // ------------------------------------------------------------------ data
  async listRuns() {
    const extra = this.params.get('run');
    const list = [...RUNS];
    if (extra && !list.some(r => r.id === runName(extra))) list.push({ id: runName(extra), label: runName(extra), sub: '' });
    const ok = await Promise.all(list.map(async r => {
      try { const res = await fetch(`/sims/flight-${r.id}.sim`, { method: 'HEAD' }); return res.ok && !res.headers.get('content-type')?.includes('text/html'); } catch { return false; }
    }));
    this.runs = list.filter((_, i) => ok[i]);
    const box = this.ctx.ui.$('runs'); box.innerHTML = '';
    for (const r of this.runs) {
      const b = document.createElement('button'); b.dataset.run = r.id;
      b.innerHTML = `${r.label}<small>${r.sub}</small>`;
      b.onclick = () => this.loadRun(r.id);
      box.appendChild(b);
    }
    if (!this.runs.length) box.innerHTML = '<p id="fnote">还没有飞行记录：<code>npm run demos:generate</code></p>';
    return this.runs;
  }

  async loadRun(id) {
    id = runName(id);
    const rec = await this.ctx.assets.sim(`flight-${id}`);
    if (!rec) throw new Error(`no recording public/sims/flight-${id}.sim (run tools/sim/flight.mjs)`);
    const { meta, frames } = rec;
    if (!meta.propulsion) throw new Error(`flight-${id}.sim has no thrust log (meta.propulsion)`);
    this.source = { kind: 'softbody', run: id };
    this.rigid = false;
    const player = this.view.setRecording(meta, frames);
    this.log = new FlightLog(meta.propulsion, { flight: meta.flight, duration: player.duration });
    this.plumes.setTrack(trackFromRecording(player, meta.propulsion));
    this.metrics = meta.flight?.metrics || {};
    this.task = loadTask(TASKS[meta.flight?.task] ? meta.flight.task : 'escape-soft-landing');
    const preset = PRESETS[meta.flight?.preset];
    this.label = { title: 'QQ飛車', sub: `Jet-Car Mini · 冷气推力 · ${preset?.label || meta.flight?.preset || ''} · ${ctlLabel(meta.flight?.controller)}${meta.flight?.armDelay ? ` · 点火延迟 ${meta.flight.armDelay} s` : ''} · 软体` };
    this.afterLoad();
    document.querySelectorAll('#runs button').forEach(b => b.classList.toggle('on', b.dataset.run === id));
    const r = this.runs?.find(x => x.id === id); if (r) document.querySelector(`#runs button[data-run="${id}"]`)?.classList.add(this.metrics.success ? 'ok' : 'fail');
    const m = this.metrics, d = meta.flight?.damage;
    this.ctx.ui.$('fnote').innerHTML = m.touchdown_t !== undefined && m.touchdown_t !== null
      ? `触地 ${m.touchdown_t.toFixed(2)} s · ${m.touchdown_vertical_speed.toFixed(2)} m/s · 倾角 ${m.touchdown_tilt_deg.toFixed(1)}° · 用气 ${m.gas_used_kg.toFixed(1)} kg${d?.detached?.length ? ` · 脱落 ${d.detached.length} 件` : ''}`
      : '';
  }

  async loadTrajectory(url) {
    const traj = await fetch(url).then(r => { if (!r.ok) throw new Error(`trajectory ${url}: HTTP ${r.status}`); return r.json(); });
    if (!traj.state?.length) throw new Error(`trajectory ${url}: no state`);
    this.source = { kind: 'rigid', traj: url, producer: traj.producer, controller: traj.controller, model: traj.model };
    this.rigid = true;
    this.view.player = null;
    this.log = new FlightLog(traj, { flight: { events: traj.events || [], metrics: traj.metrics, planningTrace: traj.planningTrace }, duration: traj.t[traj.t.length - 1] });
    this.plumes.setTrack(trackFromTrajectory(traj));
    this.metrics = traj.metrics || {};
    { const id = typeof traj.task === 'string' ? traj.task : traj.task?.id; this.task = loadTask(TASKS[id] ? id : 'escape-soft-landing'); }
    const ctl = typeof traj.controller === 'string' ? traj.controller : traj.controller?.name || traj.controller?.type || '';
    this.label = { title: 'QQ飛車', sub: `Jet-Car Mini · 刚体回放 · ${traj.preset || ''} · ${ctl} · ${traj.producer || ''}` };
    this.ctx.ui.$('runs').innerHTML = '';
    this.ctx.ui.$('fnote').innerHTML = `刚体孪生轨迹回放（未变形车身）<br><code>${url}</code>`;
    this.afterLoad();
  }

  afterLoad() {
    this.trajectories.setLog(this.log, this.task?.pad);
    this.updateVisualizationUI();
    const success = this.metrics.success !== undefined ? !!this.metrics.success : true;
    this.edit = new FlightEdit(this.log, { success });
    if (this.edit.roll && this.label) this.label.sub += " · 跳跃 · 横滚 360°";
    const m = this.edit, E = this.log.p(m.tEdge);
    this.view.placeProbes(E[0], new THREE.Vector3(m.L[0], 0, m.L[2]));
    this.restY = this.log.log.state[0][1] - WORLD.deck.top; // COM height of the car standing on its wheels
    this._lastPlumeT = null;
    this.buildMarks();
  }

  // ------------------------------------------------------------------ UI
  setupUI() {
    const { ui } = this.ctx, $ = ui.$;
    const tr = (this.transport = new Transport(ui, {
      onPlay: () => {
        if (!this.playing && this.atPlaybackEnd()) this.restartPlayback();
        this.playing = !this.playing; tr.setPlaying(this.playing);
      },
      onReset: () => { this.restartPlayback(); this.playing = false; tr.setPlaying(false); },
      onLoop: on => { this.looping = on; tr.setLooping(on); },
      onSeek: u => {
        if (this.mode === 'director' && this.edit) this.T = u >= 1 ? Math.max(0, this.edit.total - 1e-8) : u * this.edit.total;
        else { const [a, b] = this.timeRange(); this.simT = a + u * (b - a); }
        if (u >= 1) { this.playing = false; tr.setPlaying(false); }
      },
    }));
    tr.setPlaying(this.playing);
    tr.addGroup(FLIGHT_GROUPS.modes, 'modes');
    tr.addGroup(FLIGHT_GROUPS.speeds, 'speeds');
    tr.addGroup(FLIGHT_GROUPS.view);
    ui.mount('<div id="status-dock" aria-label="飞行状态遥测"></div><div id="instrument-dock" aria-label="底部仪表区"><div id="instrument-flow"></div></div>');
    this.hud.mountLive($('instrument-flow'));
    $('status-dock').appendChild(this.hud.live.status);
    ui.mount(`<section id="visual-controls" class="visual-controls" aria-label="轨迹与规划遥测"><h2>轨迹 · 规划</h2><div class="visual-switches">${FLIGHT_GROUPS.trajectories}</div><div id="trajectory-legend" hidden></div></section>`);
    $('instrument-flow').appendChild($('visual-controls'));
    this.setupPanelLayout();
    for (const [id, key] of [['b-trail', 'trail'], ['b-plan', 'planning'], ['b-explore', 'exploration']]) {
      $(id).onclick = () => {
        this.visualOptions[key] = !this.visualOptions[key];
        this.updateVisualizationUI();
      };
    }
    this.updateVisualizationUI();
    document.querySelectorAll('#modes button').forEach(b => (b.onclick = () => this.setMode(b.dataset.mode)));
    document.querySelectorAll('#speeds button').forEach(b => (b.onclick = () => { this.speed = +b.dataset.speed; document.querySelectorAll('#speeds button').forEach(x => x.classList.toggle('on', x === b)); }));
    $('b-plume').onclick = () => { const on = !$('b-plume').classList.contains('on'); this.plumes.visible = on; $('b-plume').classList.toggle('on', on); };
    $('b-tele').onclick = () => { const on = !$('b-tele').classList.contains('on'); $('instrument-dock').hidden = !on; $('status-dock').hidden = !on; this.hud.live.verdict.style.display = on ? '' : 'none'; $('b-tele').classList.toggle('on', on); };
    const setClean = hidden => {
      document.body.classList.toggle('clean', hidden);
      // Hidden controls must also leave keyboard navigation, while the restore button stays accessible.
      for (const el of [this.transport.el, $('fpanel'), $('instrument-dock'), $('status-dock')]) el.inert = hidden;
      (hidden ? $('b-ui-restore') : $('b-ui')).focus({ preventScroll: true });
    };
    $('b-ui').onclick = () => setClean(true);
    $('b-ui-restore').onclick = () => setClean(false);
    addEventListener('keydown', e => {
      if (e.key === 'h' || e.key === 'H') {
        if (!e.repeat && !e.target?.isContentEditable && !['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target?.tagName)) setClean(!document.body.classList.contains('clean'));
      }
      if (e.key === ' ') {
        const target = e.target, tag = target?.tagName;
        if (target?.isContentEditable || tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
        e.preventDefault(); $('b-play').click();
      }
    });
    this.setMode(this.mode);
  }

  setMode(mode) {
    const prevMode = this.mode;
    this.mode = mode;
    document.querySelectorAll('#modes button').forEach(b => b.classList.toggle('on', b.dataset.mode === mode));
    this.controls.enabled = mode === 'free' || mode === 'follow';
    this.controls.enablePan = mode !== 'follow';
    document.body.classList.toggle('cinema', mode === 'director');
    if (prevMode === 'director' && mode !== 'director' && this.lastEval) this.simT = this.lastEval.t;
    if (mode === 'free' && this.log) { const c = this.log.p(this.simT); this.controls.target.set(c[0], c[1], c[2]); }
    if (mode === 'follow' && this.log) {
      const c = this.log.p(this.simT), cam = this.ctx.stage.camera;
      this.controls.target.set(c[0], c[1], c[2]);
      if (!this._followReady) cam.position.copy(this.controls.target).add(new THREE.Vector3(7.5, 3.0, 6.5));
      this._followReady = true;
      cam.fov = 45; cam.updateProjectionMatrix();
      this.controls.update();
    }
    this.buildMarks();
  }
  updateVisualizationUI() {
    this.trajectories.setOptions(this.visualOptions);
    const $ = this.ctx.ui.$, available = this.trajectories.hasPlanning;
    for (const [id, key] of [['b-trail', 'trail'], ['b-plan', 'planning'], ['b-explore', 'exploration']]) {
      const button = $(id);
      if (!button) continue;
      button.classList.toggle('on', this.visualOptions[key]);
      button.setAttribute('aria-pressed', String(this.visualOptions[key]));
      if (key !== 'trail') {
        button.disabled = !available || (key === 'exploration' && !this.visualOptions.planning);
        button.title = available ? (key === 'planning' ? '显示当前规划与短期预测' : '显示本次求解的候选路径') : '此录制未保存规划';
      }
    }
    const legend = $('trajectory-legend');
    if (legend) {
      legend.hidden = !this.visualOptions.trail && !this.visualOptions.planning;
      legend.innerHTML = `${this.visualOptions.trail ? '<span style="color:#93D3B8">━ 实际轨迹</span><span style="color:#D94A58">× 实际着陆</span>' : ''}${this.visualOptions.planning && available ? '<span style="color:#F2C879">━ 上层规划</span><span style="color:#78C9ED">━ 短期预测</span><span style="color:#397DE0">＋ 预测着陆</span>' : ''}${this.visualOptions.planning && this.visualOptions.exploration && available ? '<span style="color:#A79BBB">━ 时间候选</span>' : ''}${!available ? '<span>此录制未保存规划</span>' : ''}`;
    }
  }
  timeRange() { return [-2.5, this.log?.duration || 6]; }

  atPlaybackEnd() {
    const end = this.mode === 'director' ? this.edit?.total || 0 : this.timeRange()[1];
    const t = this.mode === 'director' ? this.T : this.simT;
    return end > 0 && t >= end - 1e-8;
  }

  restartPlayback() {
    if (this.mode === 'director') this.T = 0;
    else this.simT = this.timeRange()[0];
  }

  buildMarks() {
    if (!this.edit) { this.transport.setMarks([]); return; }
    const e = this.edit, toU = t => {
      if (this.mode === 'director') return e.timelineU(t);
      const [a, b] = this.timeRange(); return (t - a) / (b - a);
    };
    const marks = [];
    for (const [t, cls] of [[e.tEdge, 'edge'], [e.tIgn, 'ign'], [e.tTouch, 'td']]) { if (t === null) continue; const u = toU(t); if (u !== null) marks.push({ u, cls }); }
    for (const b of this.view.player?.meta.events?.breaks || []) { const u = toU(b.t); if (u !== null && b.type !== 'detach') marks.push({ u, cls: 'glass' }); }
    this.transport.setMarks(marks);
  }

  // ------------------------------------------------------------------ scene at sim time t
  /** Rigid replay: every lattice node at p + R(q)(x_rest − comB). */
  poseRigid(t) {
    const L = this.log, p = L.p(t), R = quatToMat(L.q(t), (this._R ||= new Float64Array(9))), c = L.comB(t);
    const X = this.car.nodes.rest, N = this.car.nodes.count, pos = (this._rigidPos ||= new Float32Array(N * 3));
    for (let n = 0; n < N; n++) {
      const x = X[n * 3] - c[0], y = X[n * 3 + 1] - c[1], z = X[n * 3 + 2] - c[2];
      pos[n * 3] = p[0] + R[0] * x + R[1] * y + R[2] * z; pos[n * 3 + 1] = p[1] + R[3] * x + R[4] * y + R[5] * z; pos[n * 3 + 2] = p[2] + R[6] * x + R[7] * y + R[8] * z;
    }
    this.view.carView.update(pos, t, null);
    this.view.fx.update(t);
    this.ctx.stage.focus.set(p[0], Math.min(p[1], 4), p[2]);
    this.view.pickProbe(p[1]);
  }

  applyScene(t) {
    if (this.rigid) { this.poseRigid(t); this.plumes.update(t); }
    else {
      this.view.apply(t);
      // big jumps in time (seek, replay cuts): re-seed the mount fits from the logged body orientation
      const jump = this._lastPlumeT === null || Math.abs(t - this._lastPlumeT) > 0.06;
      if (jump && this.plumes.track?.fitter) this.plumes.track.fitter.seed(this.log.q(Math.max(0, t)));
      this.plumes.update(t, t >= 0 ? this.view.player.pos : null, jump ? 12 : 3);
    }
    this._lastPlumeT = t;
  }

  hudState(t) {
    const L = this.log, v = L.v(t), w = L.w(t), tk = L.tank(Math.max(0, t)), e = this.edit;
    const thrust = Array.from({ length: NU }, (_, i) => (t < 0 ? 0 : L.thrust(t, i))), valve = Array.from({ length: NU }, (_, i) => (t < 0 ? 0 : L.valve(t, i)));
    const { p_set, dp_min } = PROPULSION.regulator;
    const pc = L.log.pc ? L.lerp('pc', Math.max(0, t), 0) : Math.min(p_set, tk.p - dp_min);
    const phase = t < e.tEdge ? 'deck' : t < e.tTouch ? 'air' : 'ground';
    const m = this.metrics;
    const verdict = t >= e.tTouch + 0.2 && m.touchdown_vertical_speed !== undefined
      ? { success: !!m.success, vs: m.touchdown_vertical_speed, tilt: m.touchdown_tilt_deg, rate: m.touchdown_angular_rate, hull: !!m.hull_strike, crit: this.task?.success, hold: m.max_tilt_hold_deg,
        padDistance: m.pad_distance, horizontalSpeed: m.touchdown_horizontal_speed,
        note: `${this.rigid ? '刚体孪生' : '软体'} · 用气 ${(m.gas_used_kg ?? 0).toFixed(1)} kg${m.peak_accel_g ? ` · 峰值 ${m.peak_accel_g.toFixed(1)} g` : ''}`,
        kerb: !!m.kerb && (m.kerb_after_td == null || t >= e.tTouch + m.kerb_after_td),
        stop: m.stopped && t >= e.tTouch + (m.stop_time ?? 0) ? `刹停 ${m.stop_time.toFixed(1)} s · ${m.stop_distance.toFixed(0)} m${m.kerb ? '   ·   路缘 ✗' : '   ·   未碰路缘 ✓'}` : null }
      : null;
    return {
      t, thrust, valve, commandLimits: L.flight?.commandLimits ?? L.log?.commandLimits, nom: this.nom.F, tank: tk, m0: L.log.tank[0][0], pc: pc > 2 * WORLD.air.pressure ? pc : 0,
      alt: L.p(t)[1] - this.restY, vy: v[1], speed: Math.hypot(v[0], v[2]), tilt: L.tilt(t), rate: Math.hypot(w[0], w[1], w[2]), phase, verdict,
      roll: e.roll ? e.roll.at(t) : null, drive: t < 0 ? (L.log.drive ? { brake: 0, steer: 0 } : null) : L.driveState(t),
      title: this.label?.title, sub: this.label?.sub,
    };
  }

  drawHud(t, film) {
    const s = this.hudState(t);
    if (!film) { this.hud.drawLive(s); return; }
    const r = this.ctx.stage.renderer.domElement;
    this.hud.resize(r.width, r.height);
    this.hud.draw(s);
  }

  setupPanelLayout() {
    const $ = this.ctx.ui.$;
    const update = () => {
      const compact = innerWidth < 900 || innerHeight <= 520;
      if (compact !== this._compactPanels) {
        this._compactPanels = compact;
        document.body.classList.toggle('compact-panels', compact);
        if (compact) $('instrument-flow').prepend($('fpanel'));
        else this.ctx.ui.root.appendChild($('fpanel'));
      }
      const bar = this.transport.el.getBoundingClientRect();
      const space = `${innerHeight - bar.top + 12}px`;
      if (space !== this._playbackSpace) {
        document.body.style.setProperty('--playback-space', space); this._playbackSpace = space;
      }
      const height = `${$('instrument-dock').getBoundingClientRect().height}px`;
      if (height !== this._instrumentHeight) {
        document.body.style.setProperty('--instrument-height', height); this._instrumentHeight = height;
      }
    };
    this.panelObserver = new ResizeObserver(update);
    this.panelObserver.observe(this.transport.el);
    this.panelObserver.observe($('instrument-dock'));
    window.addEventListener('resize', update);
    this._panelResize = update;
    update();
  }

  dispose() {
    this.panelObserver?.disconnect();
    window.removeEventListener('resize', this._panelResize);
    this.controls?.dispose();
    this.trajectories?.dispose();
    document.body.classList.remove('compact-panels');
  }

  // ------------------------------------------------------------------ frame
  frame(dt) {
    this._panelResize?.();
    const stage = this.ctx.stage, cam = stage.camera, ui = this.ctx.ui;
    if (!this.edit) { stage.render(dt); return; }
    let t;
    if (this.mode === 'director') {
      if (this.playing) this.T += dt * this.speed;
      if (this.T >= this.edit.total) {
        if (this.looping) this.T %= this.edit.total;
        else { this.T = Math.max(0, this.edit.total - 1e-8); this.playing = false; this.transport.setPlaying(false); }
      }
      const ev = (this.lastEval = this.edit.evaluate(this.T));
      t = ev.t; applyPose(cam, ev);
      ui.setShot(`镜头 ${this.edit.shots.indexOf(ev.shot) + 1}<b>${ev.shot.name}</b>`);
      this.transport.setProgress(this.T / this.edit.total);
    } else {
      const [a, b] = this.timeRange();
      if (this.playing) this.simT += dt * this.speed;
      if (this.playing && this.simT >= b) {
        if (this.looping) this.simT = a + ((this.simT - a) % (b - a));
        else { this.simT = b; this.playing = false; this.transport.setPlaying(false); }
      }
      t = this.simT;
      const c = this.log.p(t);
      if (this.mode === 'follow') {
        const tgt = new THREE.Vector3(c[0], c[1], c[2]);
        // Move the orbit centre and camera together; only user input changes the relative view.
        cam.position.add(tgt.clone().sub(this.controls.target));
        this.controls.target.copy(tgt);
        this.controls.update();
      } else { this.controls.target.lerp(new THREE.Vector3(c[0], c[1], c[2]), 1 - Math.exp(-dt * 2)); this.controls.update(); }
      ui.setShot('');
      this.transport.setProgress((t - a) / (b - a));
    }
    this.applyScene(t);
    this.trajectories.draw(t);
    if (this.world.group.userData.wind) this.world.group.userData.wind.value += dt;
    this.transport.setLabel(`${t.toFixed(2)} s`);
    this.drawHud(t, false);
    stage.render(dt);
  }

  /** Offline film frame at film time T (cinema/film.js) + the telemetry overlay. */
  renderFilm(T, { samples = 16, shutter = 0.5 / 30, aperture = 0.035 } = {}) {
    const e = this.devEdit() || this.edit, wind = this.world.group.userData.wind, ev0 = e.evaluate(T);
    this.ctx.ui.setShot(`镜头 ${e.shots.indexOf(ev0.shot) + 1}<b>${ev0.shot.name}</b>`);
    this.transport.setLabel(`${ev0.t.toFixed(2)} s`);
    renderFilmFrame(this.ctx.stage, T, x => e.evaluate(x), t => { this.applyScene(t); if (wind) wind.value = T; }, { samples, shutter, aperture });
    this.drawHud(ev0.t, true);
    if (this.filmHud) this.compositeHud();
  }

  /** Look-dev camera (devcam / devt params): fixed offset from the car's COM, sim time = devt + T. */
  devEdit() {
    const c = this.params.get('devcam');
    if (!c) return null;
    const v = c.split(',').map(Number), t0 = +(this.params.get('devt') || 2);
    const shot = { name: 'look-dev' };
    return {
      shots: [shot],
      evaluate: T => {
        const t = t0 + T, p = this.log.p(t0), d = this.log.p(t);
        const pr = this.params, o = new THREE.Vector3(d[0] - p[0], d[1] - p[1], d[2] - p[2]).multiplyScalar(pr.has('devfollow') || pr.has('devtrack') || pr.has('devpan') ? 1 : 0);
        if (pr.has('devtrack') || pr.has('devpan')) o.y = 0;
        const pos = new THREE.Vector3(p[0] + v[0], p[1] + v[1], p[2] + v[2]);
        if (!pr.has('devpan')) pos.add(o);
        return { t, shot, u: 0, shake: 0, roll: 0, pos, tgt: new THREE.Vector3(p[0] + v[3], p[1] + v[4], p[2] + v[5]).add(o), fov: v[6] || 40, aperture: 0.002 };
      },
    };
  }

  /** Draw the HUD canvas over the finished frame (the film tools read only the WebGL canvas). */
  compositeHud() {
    const r = this.ctx.stage.renderer;
    if (!this._hudQuad) {
      this._hudTex = new THREE.CanvasTexture(this.hud.canvas);
      this._hudTex.colorSpace = THREE.SRGBColorSpace;
      const mat = new THREE.MeshBasicMaterial({ map: this._hudTex, transparent: true, toneMapped: false, depthTest: false, depthWrite: false });
      this._hudQuad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), mat);
      this._hudScene = new THREE.Scene(); this._hudScene.add(this._hudQuad);
      this._hudCam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    }
    this._hudTex.needsUpdate = true;
    const ac = r.autoClear; r.autoClear = false; r.setRenderTarget(null);
    r.render(this._hudScene, this._hudCam);
    r.autoClear = ac;
  }
}
