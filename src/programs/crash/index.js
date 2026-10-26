// Crash viewer (program "crash", the default): precomputed or live soft-body runs of the six launch presets,
// with the director edit, free / follow cameras, HUD, procedural audio and offline film export.
// Params: preset=roof|nose|side|rear|corner|flat, mode=director|free|follow, T=<film s>, paused, kiosk.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Program } from '../../app/program.js';
import { Transport } from '../../app/transport.js';
import { PlaybackView } from '../../playback/view.js';
import { applyPose } from '../../cinema/sequence.js';
import { renderFilmFrame } from '../../cinema/film.js';
import { CrashAudio } from '../../audio/crashAudio.js';
import { PART_NAMES as NAMES } from '../../car/names.js';
import { PRESETS, DECK } from '../../scene/layout.js';
import { CrashEdit } from './edit.js';
import { CRASH_UI, CRASH_GROUPS } from './ui.js';

const countBelow = (arr, t) => { let n = 0; for (let i = 0; i < arr.length; i++) if (arr[i] <= t) n++; return n; };

export default class CrashProgram extends Program {
  static meta = { id: 'crash', title: 'JetCar Mini · 软体破坏仿真' };

  async init() {
    const { stage, ui, assets, params } = this.ctx;
    this.params = params;
    ui.loadText('载入天空');
    const env = await assets.sky();
    ui.loadText('搭建高架场景');
    this.world = assets.world();
    ui.loadText('载入车模');
    await assets.gltf();
    ui.loadText('构建软体晶格');
    const car = (this.car = await assets.car());
    this.view = new PlaybackView(this.ctx, { car, world: this.world, env });
    ui.mount(CRASH_UI);
    ui.$('techline').textContent = `XPBD 节点-梁晶格 · ${(car.nodes.count / 1e4).toFixed(1)} 万节点 · ${(car.beams.count / 1e4).toFixed(1)} 万梁 · 塑性屈服 · 断裂 · GPU 四面体嵌入`;
    this.controls = new OrbitControls(stage.camera, stage.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.enabled = false;
    this.mode = params.get('mode') || 'director';
    this.speed = 1; this.playing = !params.has('paused');
    this.T = +(params.get('T') || 0);
    this.simT = 0;
    this.audio = new CrashAudio();
    this.setupUI();
    this.settled = await assets.settled();
    await this.world.ready;
    ui.loadText('载入预计算结果');
    await this.loadPreset(params.get('preset') || 'roof');
    const self = this;
    this.film = {
      get total() { return self.director.total; },
      render: (T, opts) => this.renderFilm(T, opts),
      describe: () => this.director.describe(),
    };
    this.ctx.ui.setTitle(CrashProgram.meta.title);
  }

  get player() { return this.view.player; }

  // ------------------------------------------------------------------ data
  async loadPreset(name) {
    this.cancelRun();
    this.presetName = name;
    document.querySelectorAll('#presets button').forEach(b => b.classList.toggle('on', b.dataset.preset === name));
    let rec = null;
    try { rec = await this.ctx.assets.sim(name); } catch (e) { console.warn('bad recording for', name, e); }
    if (rec) this.useRecording(rec.meta, rec.frames);
    else {
      console.warn('no precomputed run for', name);
      this.runScenario({ ...PRESETS[name], label: PRESETS[name].label });
    }
  }

  useRecording(meta, frames, live = false) {
    const player = this.view.setRecording(meta, frames);
    this.live = live;
    this.director = !live && frames.length > 2 ? new CrashEdit(player) : null;
    const d = this.director;
    this.view.placeProbes(d ? player.bodyCentre(d.tEdge)[0] : 2, d ? d.L : undefined);
    this.T = live ? 0 : this.T;
    this.buildTicker(meta);
    this.buildMarks();
  }

  runScenario(sc) {
    this.cancelRun();
    const $ = this.ctx.ui.$;
    const scenario = { strength: 1, recordHz: 100, duration: 7, substepHz: 8000, ...sc };
    const worker = (this.worker = new Worker(new URL('../../softbody/worker.js', import.meta.url), { type: 'module' }));
    const c = this.car;
    const car = { h: c.h, nodes: c.nodes, chunks: c.chunks, pieces: c.pieces, beams: c.beams, tets: c.tets, groups: c.groups.map(g => ({ ...g })), wheels: c.wheels, render: { tetCount: c.render.tetCount, tetNodes: c.render.tetNodes } };
    const meta = { N: c.nodes.count, C: c.chunks.length, recordHz: scenario.recordHz, scenario, events: {}, particles: { count: 0, kind: [], spawn: [], size: [] } };
    const frames = [];
    $('progress').hidden = false;
    this.useRecording(meta, frames, true);
    this.mode === 'director' && this.setMode('follow');
    worker.onmessage = e => {
      const m = e.data;
      if (m.type === 'frames') {
        frames.push(...m.frames);
        if (!meta.launch && frames.length) meta.launch = null;
        Object.assign(meta, { groupTime: m.events.groupTime, tearTime: m.events.tearTime, groups: m.events.groups, particles: m.events.particles, events: { dust: m.events.dust, sparks: m.events.sparks, scrape: m.events.scrape, breaks: m.events.breaks } });
        this.view.refreshMeta(meta, (this._fxTick = (this._fxTick || 0) + 1) % 4 === 0);
        this.buildTicker(meta);
        $('progress').querySelector('i').style.width = `${(m.progress * 100).toFixed(1)}%`;
        $('progress').querySelector('span').textContent = `计算中 ${(m.progress * scenario.duration).toFixed(2)} / ${scenario.duration} s`;
      } else if (m.type === 'done') {
        Object.assign(meta, m.meta);
        this.useRecording(meta, frames, false);
        $('progress').hidden = true;
        this.worker.terminate(); this.worker = null;
        this.T = 0; this.setMode('director');
      }
    };
    worker.postMessage({ type: 'run', car, settled: this.settled, scenario });
  }
  cancelRun() { if (this.worker) { this.worker.terminate(); this.worker = null; this.ctx.ui.$('progress').hidden = true; } }

  // ------------------------------------------------------------------ UI
  setupUI() {
    const { ui, params } = this.ctx, $ = ui.$;
    const pre = $('presets');
    for (const [k, p] of Object.entries(PRESETS)) {
      const b = document.createElement('button');
      b.dataset.preset = k;
      b.innerHTML = `${p.label}<small>${p.speed} km/h</small>`;
      b.onclick = () => this.loadPreset(k);
      pre.appendChild(b);
    }
    for (const id of ['speed', 'angle', 'pitch', 'roll', 'yaw', 'strength']) {
      const i = $(`i-${id}`), o = $(`o-${id}`);
      i.oninput = () => { o.textContent = id === 'strength' ? (+i.value).toFixed(2) : i.value; };
    }
    $('b-random').onclick = () => {
      const r = (a, b) => a + Math.random() * (b - a);
      const set = (id, v) => { $(`i-${id}`).value = v; $(`i-${id}`).oninput(); };
      set('speed', Math.round(r(55, 95))); set('angle', Math.round(r(10, 24))); set('pitch', r(-2, 2).toFixed(2)); set('roll', r(-3.2, 1).toFixed(2)); set('yaw', r(-1.2, 1.2).toFixed(2));
    };
    $('b-run').onclick = () => {
      const v = id => +$(`i-${id}`).value;
      document.querySelectorAll('#presets button').forEach(b => b.classList.remove('on'));
      this.runScenario({ label: '自定义', speed: v('speed'), angle: v('angle'), spin: [v('pitch'), v('yaw'), v('roll')], strength: v('strength'), seed: Math.floor(Math.random() * 1e6) });
    };
    const tr = (this.transport = new Transport(ui, {
      onPlay: () => { this.playing = !this.playing; tr.setPlaying(this.playing); },
      onSeek: u => {
        if (this.mode === 'director' && this.director) this.T = u * this.director.total;
        else { const [a, b] = this.timeRange(); this.simT = a + u * (b - a); }
      },
    }));
    tr.addGroup(CRASH_GROUPS.modes, 'modes');
    tr.addGroup(CRASH_GROUPS.speeds, 'speeds');
    tr.addGroup(CRASH_GROUPS.view);
    document.querySelectorAll('#modes button').forEach(b => (b.onclick = () => this.setMode(b.dataset.mode)));
    document.querySelectorAll('#speeds button').forEach(b => (b.onclick = () => { this.speed = +b.dataset.speed; document.querySelectorAll('#speeds button').forEach(x => x.classList.toggle('on', x === b)); }));
    const carView = this.view.carView, lattice = this.view.lattice;
    $('b-heat').onclick = () => { const on = !carView.uniforms.uHeat.value; carView.uniforms.uHeat.value = on ? 1 : 0; $('b-heat').classList.toggle('on', on); };
    $('b-lattice').onclick = () => { lattice.group.visible = !lattice.group.visible; carView.group.visible = !lattice.group.visible; $('b-lattice').classList.toggle('on', lattice.group.visible); };
    $('b-ui').onclick = () => document.body.classList.toggle('clean');
    $('b-sound').onclick = async () => { if (this.audio.enabled) this.audio.stop(); else await this.audio.start(); $('b-sound').classList.toggle('on', this.audio.enabled); };
    let idleTimer = 0;
    const wake = () => { document.body.classList.remove('idle'); clearTimeout(idleTimer); idleTimer = setTimeout(() => document.body.classList.add('idle'), 4000); };
    addEventListener('pointermove', wake); addEventListener('pointerdown', wake); wake();
    addEventListener('keydown', e => { if (e.key === 'h' || e.key === 'H') document.body.classList.toggle('clean'); if (e.key === ' ') $('b-play').click(); });
    this.setMode(this.mode);
    if (params.has('kiosk')) {
      document.body.classList.add('kiosk');
      addEventListener('pointerdown', () => { if (!this.audio.enabled) this.audio.start().then(() => $('b-sound').classList.add('on')); }, { once: true });
    }
  }

  setMode(mode) {
    this.mode = mode;
    document.querySelectorAll('#modes button').forEach(b => b.classList.toggle('on', b.dataset.mode === mode));
    this.controls.enabled = mode === 'free';
    document.body.classList.toggle('cinema', mode === 'director');
    if (mode !== 'director' && this.lastEval) this.simT = this.lastEval.t;
    if (mode === 'free' && this.player?.ready) { const c = this.player.bodyCentre(this.simT); this.controls.target.set(c[0], c[1], c[2]); }
    this.buildMarks();
  }

  timeRange() { return [-3, this.player?.duration || 7]; }

  buildMarks() {
    if (!this.player?.ready) { this.transport.setMarks([]); return; }
    const ev = this.player.meta.events?.breaks || [];
    const toU = t => {
      if (this.mode === 'director' && this.director) return this.director.timelineU(t);
      const [a, b] = this.timeRange(); return (t - a) / (b - a);
    };
    const marks = [];
    for (const e of ev) { const u = toU(e.t); if (u !== null) marks.push({ u, cls: e.type === 'detach' ? '' : 'glass' }); }
    this.transport.setMarks(marks);
  }

  buildTicker(meta) {
    const list = (meta.events?.breaks || []).filter(e => e.type === 'detach' || e.type === 'glass' || e.type === 'crack' || e.type === 'lens');
    this.tickerEvents = list.map(e => ({ t: e.t, kind: e.type === 'detach' ? 'detach' : 'glass', text: e.type === 'detach' ? `${NAMES[e.piece] || e.piece} 脱落` : e.type === 'crack' ? '前风挡 龟裂' : `${NAMES[e.group] || e.group} 碎裂` }));
    const ol = this.ctx.ui.$('ticker'); ol.innerHTML = '';
    for (const e of this.tickerEvents) { const li = document.createElement('li'); li.className = e.kind; li.innerHTML = `${e.text}<span>${e.t.toFixed(2)}s</span>`; ol.appendChild(li); e.el = li; }
  }

  updateHUD(t) {
    if (!this.player?.ready) return;
    const meta = this.player.meta;
    for (const e of this.tickerEvents || []) e.el.classList.toggle('show', e.t <= t);
    const ev = meta.events?.breaks || [];
    const det = ev.filter(e => e.type === 'detach' && e.t <= t).length;
    const glass = ev.filter(e => e.type !== 'detach' && e.t <= t).length;
    const parts = meta.particles ? meta.particles.spawn.filter(s => s <= t).length : 0;
    const tear = meta.tearTime ? countBelow(meta.tearTime, t) : 0;
    const c = this.player.bodyCentre(t);
    const html = [['仿真时间', `${t.toFixed(3)} s`], ['车身高度', `${c[1].toFixed(2)} m`], ['脱落部件', det], ['破碎玻璃/灯罩', glass], ['碎片', parts], ['撕裂单元', tear]].map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
    if (html !== this._hud) { this.ctx.ui.$('stats').innerHTML = html; this._hud = html; }
  }
  shotCaption(ev) { return `镜头 ${this.director.shots.indexOf(ev.shot) + 1}<b>${ev.shot.name}</b>`; }

  // ------------------------------------------------------------------ frame
  /** Advance and render one frame. dt = wall seconds. */
  frame(dt) {
    const stage = this.ctx.stage, cam = stage.camera, ui = this.ctx.ui;
    if (!this.player?.ready) { stage.render(dt); return; }
    let t, shake = 0;
    if (this.mode === 'director' && this.director) {
      if (this.playing) this.T += dt * this.speed;
      // kiosk: after the full cut, show the end card and move on to the next pose
      if (this.params.has('kiosk') && this.T >= this.director.total && !this._advancing) {
        this._advancing = true;
        this.showEndCard();
        const keys = Object.keys(PRESETS), next = keys[(keys.indexOf(this.presetName) + 1) % keys.length];
        setTimeout(async () => { await this.loadPreset(next); this.T = 0; this._advancing = false; ui.$('endcard').classList.remove('show'); }, 6000);
      }
      if (this._advancing) this.T = Math.min(this.T, this.director.total - 1e-3);
      const ev = (this.lastEval = this.director.evaluate(this.T));
      t = ev.t; shake = ev.shake;
      applyPose(cam, ev);
      ui.setShot(this.shotCaption(ev));
      this.transport.setProgress((this.T % this.director.total) / this.director.total);
    } else {
      const [a, b] = this.timeRange();
      const maxT = this.live ? this.player.duration : b;
      if (this.playing) this.simT += dt * this.speed;
      if (this.simT > maxT) this.simT = this.live ? maxT : a;
      t = this.simT;
      const c = this.player.bodyCentre(t);
      if (this.mode === 'follow') {
        const tgt = new THREE.Vector3(c[0], c[1] * 0.6 + 0.4, c[2]);
        this._follow = this._follow || cam.position.clone();
        const desired = tgt.clone().add(new THREE.Vector3(-6.5, 3.2, 7.5));
        this._follow.lerp(desired, 1 - Math.exp(-dt * 3));
        cam.position.copy(this._follow); cam.fov = 45; cam.updateProjectionMatrix(); cam.lookAt(tgt);
      } else {
        this.controls.target.lerp(new THREE.Vector3(c[0], c[1], c[2]), 1 - Math.exp(-dt * 2));
        this.controls.update();
      }
      ui.setShot(this.live ? '实时计算<b>边算边看</b>' : '');
      this.transport.setProgress((t - a) / (b - a));
    }
    this.renderAt(t, dt, shake);
    const rate = this._prevT !== undefined && dt > 0 ? (t - this._prevT) / dt : this.speed;
    this._prevT = t;
    this.audio.update(t, rate, this.player.meta, this.player.bodyCentre(t)[1]);
  }

  renderAt(t, dt, shake = 0) {
    const stage = this.ctx.stage;
    this.view.apply(t);
    stage.finish.uniforms.uShake.value.set((Math.random() - 0.5) * shake * 0.01, (Math.random() - 0.5) * shake * 0.01);
    if (this.world.group.userData.wind) this.world.group.userData.wind.value += dt;
    this.transport.setLabel(`${t.toFixed(2)} s`);
    this.updateHUD(t);
    stage.render(dt);
  }

  /** Offline film frame at director time T (see cinema/film.js). Used by tools/film/*.mjs. */
  renderFilm(T, { samples = 16, shutter = 0.5 / 30, aperture = 0.035 } = {}) {
    const d = this.director, wind = this.world.group.userData.wind;
    const ev0 = d.evaluate(T);
    this.ctx.ui.setShot(this.shotCaption(ev0));
    this.transport.setLabel(`${ev0.t.toFixed(2)} s`);
    this.updateHUD(ev0.t);
    renderFilmFrame(this.ctx.stage, T, x => d.evaluate(x), t => { this.view.apply(t); if (wind) wind.value = T; }, { samples, shutter, aperture });
  }

  showEndCard() {
    const meta = this.player.meta, ev = meta.events?.breaks || [];
    const det = ev.filter(e => e.type === 'detach').map(e => NAMES[e.piece] || e.piece);
    const glass = ev.filter(e => e.type === 'glass' || e.type === 'lens' || e.type === 'crack').length;
    const label = PRESETS[this.presetName]?.label || meta.label || '';
    const el = this.ctx.ui.$('endcard');
    el.innerHTML = `<h3>${label}</h3><p>${meta.scenario?.speed ?? ''} km/h · 坠落 ${DECK.top} m</p>
      <div class="nums"><div><b>${det.length}</b><span>部件脱落</span></div><div><b>${glass}</b><span>玻璃/灯罩碎裂</span></div><div><b>${meta.particles?.count ?? 0}</b><span>碎片</span></div><div><b>${(meta.tearTime || []).filter(t => t < 1e8).length}</b><span>金属撕裂单元</span></div></div>
      <p class="parts">${det.join(' · ')}</p>`;
    el.classList.add('show');
  }
}
