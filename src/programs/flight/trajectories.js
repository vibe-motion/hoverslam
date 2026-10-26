// Canvas overlay for recorded actual and planned flight paths. It is driven only by draw(t), so seeking and
// paused playback produce the same picture as forward playback at that simulation time.
import { Vector3 } from 'three';
import { streetLevel } from '../../flight/world.js';

const MINT = '#93D3B8', RED = '#D94A58', GOLD = '#F2C879', BLUE = '#78C9ED';
const PLAN_BLUE = '#397DE0', LILAC = '#A79BBB';
const PRE_ROLL = -2.5, MAX_ACTUAL_SAMPLES = 4096, MAX_CANDIDATES = 8;

export class FlightTrajectories {
  constructor({ root, camera }) {
    this.camera = camera;
    this.canvas = document.createElement('canvas');
    this.canvas.className = 'flight-trajectories';
    Object.assign(this.canvas.style, { position: 'fixed', inset: '0', width: '100vw', height: '100vh',
      display: 'block', pointerEvents: 'none', zIndex: '1' });
    this.g = this.canvas.getContext('2d');
    root.appendChild(this.canvas);
    this.options = { trail: true, planning: false, exploration: false };
    this.hasPlanning = false;
    this.actual = [];
    this.touchdown = null;
    this.traces = [];
    this._world = new Vector3();
    this._project = new Vector3();
    this._screenOut = [0, 0];
    this._p = [0, 0, 0];
    this._lastDpr = 0;
  }

  setLog(log, fallbackPad = null) {
    this.log = log;
    this.pad = normalizePad(log?.flight?.pad) || normalizePad(fallbackPad);
    this.actual = [];
    this.touchdown = null;
    this.traces = [];
    if (!log?.t?.length || !log?.p) { this.hasPlanning = false; return; }

    // Sample the full pre-roll and log timeline once. A stride bounds memory and drawing cost for dense logs.
    const T = log.t, first = T[0], end = T[T.length - 1];
    const count = T.length + 1, stride = Math.max(1, Math.ceil(count / MAX_ACTUAL_SAMPLES));
    this._addActual(PRE_ROLL);
    for (let t = PRE_ROLL + 0.05; t < first; t += 0.05) this._addActual(t);
    for (let i = 0; i < T.length; i += stride) if (T[i] >= PRE_ROLL) this._addActual(T[i]);
    if (this.actual[this.actual.length - 1]?.t !== end) this._addActual(end);

    const flight = log.flight, events = flight?.events?.length ? flight.events : (log.log?.events || []);
    let touchdown = null;
    for (const event of events) {
      if (event?.type === 'touchdown' && Number.isFinite(event.t)) { touchdown = event; break; }
    }
    const metrics = flight?.metrics || log.log?.metrics;
    const mt = metrics?.touchdown_t;
    if (!touchdown && Number.isFinite(mt)) touchdown = { t: mt };
    if (touchdown) {
      const p = this._eventPosition(touchdown, metrics);
      if (p) this.touchdown = { t: touchdown.t, p };
    }

    const flightTrace = flight?.planningTrace;
    const trace = Array.isArray(flightTrace) ? (flightTrace.length ? flightTrace : log.log?.planningTrace) : (flightTrace || log.log?.planningTrace);
    this.traces = Array.isArray(trace) ? trace.slice() : (trace ? [trace] : []);
    this.traces.sort((a, b) => a.t - b.t);
    this.hasPlanning = this.traces.some(e => Number.isFinite(e?.t));
  }

  _addActual(t) {
    if (this.actual.length && this.actual[this.actual.length - 1].t === t) return;
    this.actual.push({ t, p: this.log.p(t, [0, 0, 0]) });
  }

  _eventPosition(event, metrics) {
    const p = event.p || event.position || metrics?.touchdown_p || metrics?.touchdown_position;
    if (Array.isArray(p) && p.length >= 3 && p.slice(0, 3).every(Number.isFinite)) return p.slice(0, 3);
    if (Number.isFinite(event.t)) return this.log.p(event.t, [0, 0, 0]);
    return null;
  }

  setOptions({ trail, planning, exploration } = {}) {
    if (trail !== undefined) this.options.trail = !!trail;
    if (planning !== undefined) this.options.planning = !!planning;
    if (exploration !== undefined) this.options.exploration = !!exploration;
  }

  _resize() {
    const dpr = window.devicePixelRatio || 1, w = Math.max(1, Math.round(innerWidth * dpr)), h = Math.max(1, Math.round(innerHeight * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h || this._lastDpr !== dpr) {
      this.canvas.width = w; this.canvas.height = h;
      this.canvas.style.width = `${innerWidth}px`; this.canvas.style.height = `${innerHeight}px`;
      this._lastDpr = dpr;
    }
    this.g.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  _screen(p) {
    const x = Array.isArray(p) ? p[0] : p.x, y = Array.isArray(p) ? p[1] : p.y, z = Array.isArray(p) ? p[2] : p.z;
    this._project.set(x, y, z).project(this.camera);
    const v = this._project;
    if (!Number.isFinite(v.x + v.y + v.z) || v.z < -1 || v.z > 1) return null;
    this._screenOut[0] = ((v.x + 1) * innerWidth) / 2;
    this._screenOut[1] = ((1 - v.y) * innerHeight) / 2;
    return this._screenOut;
  }

  _path(points, t, color, width, alpha = 1) {
    if (!Array.isArray(points) || points.length < 2) return;
    const g = this.g;
    g.save(); g.globalAlpha = alpha; g.strokeStyle = color; g.lineWidth = width;
    g.lineCap = 'round'; g.lineJoin = 'round'; g.beginPath();
    let pen = false;
    for (let i = 0; i < points.length; i++) {
      const p = points[i];
      if (!Array.isArray(p) || p.length < 3 || !Number.isFinite(p[0]) || !Number.isFinite(p[1]) || !Number.isFinite(p[2])) { pen = false; continue; }
      const s = this._screen(p);
      if (!s) { pen = false; continue; }
      if (!pen) { g.moveTo(s[0], s[1]); pen = true; }
      else g.lineTo(s[0], s[1]);
    }
    g.stroke(); g.restore();
  }

  _actual(t) {
    const g = this.g, points = this.actual;
    if (!points.length || t < PRE_ROLL) return;
    g.beginPath(); g.strokeStyle = MINT; g.lineWidth = 3; g.lineCap = 'round'; g.lineJoin = 'round';
    let pen = false;
    for (let i = 0; i < points.length && points[i].t <= t; i++) {
      const s = this._screen(points[i].p);
      if (!s) { pen = false; continue; }
      if (!pen) { g.moveTo(s[0], s[1]); pen = true; } else g.lineTo(s[0], s[1]);
    }
    if (t >= PRE_ROLL && t <= (this.log?.duration ?? Infinity)) {
      const s = this._screen(this.log.p(t, this._p));
      if (s) {
        if (!pen) g.moveTo(s[0], s[1]); else g.lineTo(s[0], s[1]);
      }
    }
    g.stroke();
  }

  _latestTrace(t) {
    let lo = 0, hi = this.traces.length;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (this.traces[mid].t <= t) lo = mid + 1; else hi = mid; }
    return lo ? this.traces[lo - 1] : null;
  }

  _remaining(points, times, t) {
    if (!times || times.length !== points?.length || !points.length || t <= times[0]) return points;
    if (t >= times[times.length - 1]) return [];
    let k = 0;
    while (k + 1 < times.length && times[k + 1] <= t) k++;
    const f = (t - times[k]) / (times[k + 1] - times[k]);
    return [points[k].map((v, j) => v + f * (points[k + 1][j] - v)), ...points.slice(k + 1)];
  }

  _touchdown() {
    if (!this.touchdown) return;
    const [x, , z] = this.touchdown.p;
    const s = this._screen(this._world.set(x, streetLevel(x, z) + 0.04, z));
    if (!s) return;
    const g = this.g, r = 7;
    g.save(); g.strokeStyle = RED; g.lineWidth = 2.5; g.beginPath();
    g.moveTo(s[0] - r, s[1] - r); g.lineTo(s[0] + r, s[1] + r);
    g.moveTo(s[0] + r, s[1] - r); g.lineTo(s[0] - r, s[1] + r); g.stroke(); g.restore();
  }

  _pad() {
    if (!this.pad) return;
    const { x, z, radius } = this.pad, g = this.g;
    g.save(); g.strokeStyle = GOLD; g.globalAlpha = 0.9; g.lineWidth = 2; g.beginPath();
    let pen = false;
    for (let i = 0; i <= 64; i++) {
      const a = i * Math.PI * 2 / 64, px = x + radius * Math.cos(a), pz = z + radius * Math.sin(a);
      const s = this._screen(this._world.set(px, streetLevel(px, pz) + 0.055, pz));
      if (!s) { pen = false; continue; }
      if (!pen) { g.moveTo(s[0], s[1]); pen = true; } else g.lineTo(s[0], s[1]);
    }
    g.stroke(); g.restore();
  }

  _plans(t) {
    if (!this.options.planning || !this.hasPlanning) return;
    const e = this._latestTrace(t);
    if (!e || e.mode === 'ground') return;
    if (this.options.exploration && Array.isArray(e.candidates)) {
      const n = Math.min(MAX_CANDIDATES, e.candidates.length);
      for (let i = 0; i < n; i++) this._path(e.candidates[i], t, LILAC, 1.5, 0.16);
    }
    this._path(this._remaining(e.upper, e.upperTimes, t), t, GOLD, 2);
    this._path(this._remaining(e.lower, e.lowerTimes, t), t, BLUE, 2.5);
    const upper = e.upper;
    const end = Array.isArray(upper) && upper.length ? upper[upper.length - 1] : null;
    if (!Array.isArray(end) || end.length < 3 || !Number.isFinite(end[0]) || !Number.isFinite(end[1]) || !Number.isFinite(end[2])) return;
    const s = this._screen(this._world.set(end[0], streetLevel(end[0], end[2]) + 0.04, end[2]));
    if (!s) return;
    const g = this.g, r = 8;
    g.save(); g.strokeStyle = PLAN_BLUE; g.globalAlpha = 1; g.lineWidth = 2.5; g.lineCap = 'round';
    g.beginPath(); g.moveTo(s[0] - r, s[1]); g.lineTo(s[0] + r, s[1]);
    g.moveTo(s[0], s[1] - r); g.lineTo(s[0], s[1] + r); g.stroke(); g.restore();
  }

  draw(t) {
    this._resize();
    const g = this.g;
    g.clearRect(0, 0, innerWidth, innerHeight);
    this.camera.updateWorldMatrix?.(true, false);
    this.camera.updateProjectionMatrix?.();
    this._pad();
    if (this.options.trail) {
      this._actual(t);
      if (this.touchdown && t >= this.touchdown.t) this._touchdown();
    }
    this._plans(t);
  }

  dispose() { this.canvas.remove(); this.log = null; this.actual = []; this.traces = []; this.touchdown = null; this.pad = null; }
}

function normalizePad(pad) {
  if (!pad || !Number.isFinite(pad.radius) || pad.radius <= 0) return null;
  const c = pad.center;
  const x = Array.isArray(c) ? c[0] : c?.x;
  const z = Array.isArray(c) ? (c.length > 2 ? c[2] : c[1]) : c?.z;
  return Number.isFinite(x) && Number.isFinite(z) ? { x, z, radius: pad.radius } : null;
}
