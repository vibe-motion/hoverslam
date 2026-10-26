// Flight telemetry drawn into a 2D canvas: shown as a DOM overlay in realtime and composited into offline film
// frames (the film tools capture only the WebGL canvas), so both look the same. Pure function of the sampled
// state at sim time t: 10 thruster bars grouped bottom / rear / side in spec order, tank pressure / temperature /
// mass, feed pressure, altitude, vertical speed, tilt, brake pressure and steer angle (spec/drive.json), and the
// touchdown verdict against the task criteria (stop tasks: stopping time / distance, kerb).
import { PROPULSION, DRIVE } from '../../core/spec.js';
import { GROUPS } from '../../flight/contract.js';
import { flightResult } from './result.js';
export { verdictLabel } from './result.js';

const FONT = '-apple-system, "PingFang SC", "Helvetica Neue", "Microsoft YaHei", sans-serif';
const MINT = '#93d3b8', HOT = '#ff8a3d', RED = '#ff5a4e', FG = 'rgba(238,242,240,0.95)', DIM = 'rgba(238,242,240,0.58)';
const GROUP_LABEL = { bottom: '底部', rear: '尾部', side: '侧面' };

export class FlightHUD {
  constructor() {
    this.canvas = document.createElement('canvas');
    this.canvas.className = 'flight-hud';
    this.g = this.canvas.getContext('2d');
    this.ids = PROPULSION.nozzles.map(z => z.id);
  }
  resize(w, h) { if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; } }

  /** Interactive cards share one CSS flow; the full-frame canvas remains the offline film source. */
  mountLive(dock) {
    this.live = {};
    for (const [name, width, label] of [['thrust', 311, '喷口遥测'], ['drive', 178, '制动与转向遥测'], ['status', 282, '飞行状态遥测']]) {
      const canvas = document.createElement('canvas');
      canvas.className = 'instrument-card'; canvas.style.width = `${width}px`;
      canvas.setAttribute('role', 'img'); canvas.setAttribute('aria-label', label);
      dock.appendChild(canvas); this.live[name] = canvas;
    }
    this.live.verdict = document.createElement('section');
    this.live.verdict.className = 'experiment-result';
    this.live.verdict.setAttribute('aria-label', '实验结果');
    this.live.verdict.innerHTML = '<h2></h2><div class="result-metrics"></div><p class="result-note"></p>';
    dock.parentElement.parentElement.appendChild(this.live.verdict);
  }

  drawLive(s) {
    // A fixed logical sheet separates card geometry from viewport size and device pixel ratio.
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.resize(Math.round(1920 * dpr), Math.round(1080 * dpr));
    this.draw({ ...s, title: null, verdict: null, letterbox: false });
    const rows = s.roll != null ? 10 : 9;
    for (const [name, rect] of Object.entries({ thrust: [22, 840, 311, 222], drive: [347, 840, 178, 222], status: [1618, 22, 282, rows * 25 + 54] })) {
      const dst = this.live[name], [x, y, w, h] = rect;
      dst.hidden = name === 'drive' && !s.drive;
      const W = Math.round(w * dpr), H = Math.round(h * dpr);
      if (dst.width !== W || dst.height !== H) { dst.width = W; dst.height = H; }
      dst.style.height = `${h}px`;
      const g = dst.getContext('2d'); g.clearRect(0, 0, W, H);
      g.drawImage(this.canvas, x * dpr, y * dpr, w * dpr, h * dpr, 0, 0, W, H);
    }
    const v = s.verdict, box = this.live.verdict;
    box.hidden = !v;
    if (v) {
      const result = flightResult(v), key = JSON.stringify(result);
      if (key !== this.resultKey) {
        this.resultKey = key;
        box.querySelector('h2').textContent = result.title;
        box.querySelector('h2').style.color = result.color;
        const rows = result.lines.map(text => { const p = document.createElement('p'); p.textContent = text; return p; });
        box.querySelector('.result-metrics').replaceChildren(...rows);
        box.querySelector('.result-note').textContent = result.note || '';
      }
    }
  }

  /**
   * @param s {t, thrust[], valve[], nom[], tank:{m,T,p}, m0, pc, pset, alt, vy, tilt, rate, speed, phase, roll (deg or null),
   *           drive: null | {brake 0..1, steer rad}, verdict: null | {success, vs, tilt, rate, hull, crit, stop?}, title, sub, letterbox}
   */
  draw(s) {
    const c = this.canvas, g = this.g, W = c.width, H = c.height, k = H / 1080;
    g.clearRect(0, 0, W, H);
    const lb = s.letterbox ? H * 0.065 : 0;
    const px = v => v * k;
    g.save();
    g.textBaseline = 'alphabetic';
    // ---- title (top left)
    if (s.title) {
      shadow(g, k);
      g.fillStyle = FG; g.font = `300 ${px(40)}px ${FONT}`; g.fillText(s.title, px(40), lb + px(64));
      g.fillStyle = DIM; g.font = `500 ${px(14)}px ${FONT}`; g.fillText(s.sub || '', px(42), lb + px(90));
      noShadow(g);
    }
    // ---- thruster bars (bottom left)
    const bx = px(40), by = H - lb - px(96), bh = px(92), bw = px(16), gap = px(9), ggap = px(26);
    panel(g, bx - px(18), by - bh - px(52), px(18) * 2 + 10 * bw + 7 * gap + 2 * ggap, bh + px(130), k);
    g.font = `500 ${px(12)}px ${FONT}`; g.fillStyle = DIM; g.fillText('冷气推力器 · 阀门开度', bx, by - bh - px(26));
    g.font = `500 ${px(9)}px ${FONT}`;
    g.fillStyle = '#397DE0'; g.fillText('━ 最小油门', bx + px(155), by - bh - px(26));
    g.fillStyle = '#D94A58'; g.fillText('━ 最大油门', bx + px(218), by - bh - px(26));
    let x = bx;
    for (const grp of ['bottom', 'rear', 'side']) {
      const idx = GROUPS[grp];
      g.fillStyle = DIM; g.font = `500 ${px(11)}px ${FONT}`;
      g.fillText(GROUP_LABEL[grp], x, by + px(56));
      for (const i of idx) {
        const v = clamp01(s.valve[i]), f = s.nom[i] > 0 ? clamp01(s.thrust[i] / s.nom[i]) : 0;
        g.fillStyle = 'rgba(255,255,255,0.10)'; g.fillRect(x, by - bh, bw, bh);
        // valve opening (outline) and delivered thrust (fill): they part when the regulator drops out
        g.fillStyle = 'rgba(255,255,255,0.85)';
        g.fillRect(x, by - bh * f, bw, bh * f);
        g.fillStyle = 'rgba(255,255,255,0.85)'; g.fillRect(x, by - bh * v - px(1.5), bw, px(3));
        // Command bounds are distinct from the measured valve opening and delivered thrust.
        // Draw the in-width limit ticks last so they remain visible above the fill and opening marker.
        const limitAt = (values, fallback) => Array.isArray(values) ? (values[i] ?? fallback) : (values ?? fallback);
        const lo = clamp01(limitAt(s.commandLimits?.min, 0)), hi = clamp01(limitAt(s.commandLimits?.max, 1));
        g.fillStyle = '#397DE0'; g.fillRect(x, by - bh * lo - px(1), bw, px(2));
        g.fillStyle = '#D94A58'; g.fillRect(x, by - bh * hi - px(1), bw, px(2));
        g.save(); g.translate(x + bw / 2 + px(3.5), by + px(6)); g.rotate(Math.PI / 2);
        g.fillStyle = f > 0.02 ? FG : DIM; g.font = `500 ${px(9)}px ${FONT}`; g.fillText(this.ids[i].replace('_', ''), 0, 0); g.restore();
        x += bw + gap;
      }
      x += ggap - gap;
    }
    const total = s.thrust.reduce((a, b) => a + b, 0);
    g.fillStyle = FG; g.font = `500 ${px(13)}px ${FONT}`;
    g.fillText(`总推力 ${(total / 1000).toFixed(1)} kN`, bx, by + px(76));
    // ---- brakes / steering (next to the thrusters): brake pressure bar, steer needle on a +-max_deg arc
    if (s.drive) {
      const dx = bx - px(18) + px(18) * 2 + 10 * bw + 7 * gap + 2 * ggap + px(14), dw = px(178);
      panel(g, dx, by - bh - px(52), dw, bh + px(130), k);
      const x0 = dx + px(18), b = clamp01(s.drive.brake), on = b > 0.03;
      g.font = `500 ${px(12)}px ${FONT}`; g.fillStyle = DIM; g.fillText('制动 · 转向', x0, by - bh - px(26));
      g.fillStyle = 'rgba(255,255,255,0.10)'; g.fillRect(x0, by - bh, bw * 1.6, bh);
      g.fillStyle = on ? RED : 'rgba(255,90,78,0.3)'; g.fillRect(x0, by - bh * b, bw * 1.6, bh * b);
      g.fillStyle = on ? FG : DIM; g.font = `500 ${px(11)}px ${FONT}`; g.fillText('制动', x0, by + px(22)); g.fillText(`${(b * 100).toFixed(0)}%`, x0, by + px(40));
      const cx = x0 + px(96), cy = by - px(20), r = px(52), mx = DRIVE.steer.max_deg * Math.PI / 180, d = s.drive.steer, a = Math.max(-1, Math.min(1, d / mx));
      g.strokeStyle = 'rgba(255,255,255,0.18)'; g.lineWidth = px(3); g.beginPath(); g.arc(cx, cy, r, -Math.PI / 2 - 0.9, -Math.PI / 2 + 0.9); g.stroke();
      g.strokeStyle = Math.abs(a) > 0.01 ? HOT : MINT; g.lineWidth = px(3); g.beginPath(); g.moveTo(cx, cy); g.lineTo(cx + r * Math.sin(0.9 * a), cy - r * Math.cos(0.9 * a)); g.stroke();
      g.fillStyle = FG; g.beginPath(); g.arc(cx, cy, px(3.5), 0, 2 * Math.PI); g.fill();
      g.textAlign = 'center'; g.fillStyle = DIM; g.font = `500 ${px(11)}px ${FONT}`; g.fillText('转向', cx, by + px(22));
      g.fillStyle = FG; g.font = `500 ${px(13)}px ${FONT}`; const dd = d * 180 / Math.PI; g.fillText(`${Math.abs(dd) < 0.05 ? '' : dd > 0 ? '右 ' : '左 '}${Math.abs(dd).toFixed(1)}°`, cx, by + px(40)); g.textAlign = 'left';
    }
    // ---- telemetry (top right)
    const rows = [
      ['高度', `${s.alt.toFixed(2)} m`],
      ['垂直速度', `${s.vy >= 0 ? '+' : '−'}${Math.abs(s.vy).toFixed(2)} m/s`],
      ['水平速度', `${(s.speed * 3.6).toFixed(0)} km/h`],
      ['倾角', `${s.tilt.toFixed(1)}°`],
      ...(s.roll != null ? [['横滚（累计）', `${s.roll.toFixed(0)}°`]] : []),
      ['角速度', `${s.rate.toFixed(2)} rad/s`],
      ['气瓶压力', `${(s.tank.p / 1e6).toFixed(1)} MPa`],
      ['气体温度', `${s.tank.T.toFixed(0)} K`],
      ['供气压力', s.pc > 0 ? `${(s.pc / 1e6).toFixed(1)} MPa` : '—'],
      ['剩余气体', `${s.tank.m.toFixed(1)} kg`],
    ];
    const tw = px(250), tx = W - tw - px(36), ty = lb + px(36), rh = px(25);
    panel(g, tx - px(16), ty - px(14), tw + px(32), rows.length * rh + px(54), k);
    g.font = `500 ${px(12)}px ${FONT}`; g.fillStyle = DIM; g.fillText(`遥测 · ${phaseLabel(s.phase)}`, tx, ty + px(8));
    rows.forEach(([a, b], r) => {
      const y = ty + px(38) + r * rh;
      g.fillStyle = DIM; g.font = `400 ${px(14)}px ${FONT}`; g.fillText(a, tx, y);
      g.fillStyle = FG; g.font = `500 ${px(15)}px ${FONT}`; g.textAlign = 'right'; g.fillText(b, tx + tw, y); g.textAlign = 'left';
    });
    // gas gauge
    const gy = ty + px(38) + rows.length * rh - px(6);
    g.fillStyle = 'rgba(255,255,255,0.12)'; g.fillRect(tx, gy, tw, px(4));
    const fr = clamp01(s.tank.m / s.m0);
    g.fillStyle = fr > 0.35 ? MINT : HOT; g.fillRect(tx, gy, tw * fr, px(4));
    // ---- verdict (centre) after touchdown
    if (s.verdict) {
      const result = flightResult(s.verdict), cx = W / 2, cy = lb + px(150);
      shadow(g, k);
      g.textAlign = 'center';
      g.fillStyle = result.color; g.font = `300 ${px(56)}px ${FONT}`; g.fillText(result.title, cx, cy);
      g.fillStyle = FG; g.font = `400 ${px(16)}px ${FONT}`;
      result.lines.forEach((line, i) => g.fillText(line, cx, cy + px(34 + i * 24)));
      if (result.note) { g.fillStyle = DIM; g.font = `400 ${px(13)}px ${FONT}`; g.fillText(result.note, cx, cy + px(34 + result.lines.length * 24)); }
      g.textAlign = 'left';
      noShadow(g);
    }
    g.restore();
  }
}

const clamp01 = v => (v < 0 ? 0 : v > 1 ? 1 : v);
const phaseLabel = p => ({ deck: '桥面', air: '空中', ground: '着地' })[p] || p || '';
function panel(g, x, y, w, h, k) {
  g.save(); g.fillStyle = 'rgba(14,18,20,0.46)'; g.strokeStyle = 'rgba(255,255,255,0.12)'; g.lineWidth = Math.max(1, k);
  const r = 14 * k; g.beginPath(); g.roundRect(x, y, w, h, r); g.fill(); g.stroke(); g.restore();
}
function shadow(g, k) { g.shadowColor = 'rgba(0,0,0,0.55)'; g.shadowBlur = 18 * k; g.shadowOffsetY = 2 * k; }
function noShadow(g) { g.shadowColor = 'transparent'; g.shadowBlur = 0; g.shadowOffsetY = 0; }
