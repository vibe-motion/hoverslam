// DOM of the flight viewer (mounted into #hud) + its own styles (scoped to body.program-flight).
export const FLIGHT_CSS = `
body.program-flight .controls { right: 22px; bottom: 22px; flex-wrap: wrap; }
body.program-flight .controls .timeline { min-width: 120px; }
body.program-flight #fpanel { left: 22px; right: auto; top: 150px; width: min(236px, calc(100vw - 44px)); max-height: max(40px, calc(100dvh - var(--playback-space, 90px) - var(--instrument-height, 280px) - 174px)); overflow: auto; }
body.program-flight.cinema #fpanel { top: calc(6.5vh + 150px); max-height: max(40px, calc(93.5dvh - var(--playback-space, 90px) - var(--instrument-height, 280px) - 174px)); }
body.program-flight #instrument-dock { position: fixed; left: 22px; right: 22px; bottom: var(--playback-space, 90px); overflow: auto; padding: 2px 0 6px; max-height: max(48px, min(42dvh, calc(100dvh - var(--playback-space, 90px) - 180px))); z-index: 2; scrollbar-width: thin; }
body.program-flight #instrument-flow { display: flex; align-items: flex-end; gap: 14px; width: max-content; min-width: 100%; }
body.program-flight #instrument-dock { scrollbar-color: rgba(238,242,240,.35) transparent; }
body.program-flight #fpanel { transition: opacity .6s; scrollbar-color: rgba(238,242,240,.35) transparent; scrollbar-width: thin; }
body.program-flight.compact-panels #instrument-dock { max-height: max(48px, min(55dvh, calc(100dvh - var(--playback-space, 90px) - 86px))); }
body.program-flight.compact-panels #fpanel { position: static; flex: 0 0 236px; width: 236px; height: 279px; max-height: 279px; margin: 0; padding: 16px; }
body.program-flight #status-dock { position: fixed; top: 22px; right: 22px; width: min(282px, calc(100vw - 44px)); max-height: max(48px, calc(100dvh - var(--playback-space, 90px) - var(--instrument-height, 280px) - 36px)); overflow: auto; z-index: 2; scrollbar-width: thin; scrollbar-color: rgba(238,242,240,.35) transparent; }
body.program-flight.cinema #status-dock { top: calc(6.5vh + 22px); max-height: max(48px, calc(93.5dvh - var(--playback-space, 90px) - var(--instrument-height, 280px) - 36px)); }
body.program-flight #status-dock[hidden] { display: none; }
body.program-flight.clean #status-dock { visibility: hidden; }
@media (max-width: 650px) {
  body.program-flight #title { max-width: max(54px, calc(100vw - 340px)); overflow: hidden; }
  body.program-flight #title h1 { font-size: 28px; }
  body.program-flight #title h1 span, body.program-flight #title p { display: none; }
}
body.program-flight #instrument-dock[hidden] { display: none; }
body.program-flight.clean #instrument-dock { visibility: hidden; }
body.program-flight .instrument-card { flex: 0 0 auto; display: block; }
body.program-flight .instrument-card[hidden], body.program-flight .experiment-result[hidden] { display: none; }
body.program-flight .experiment-result { position: fixed; top: clamp(100px, 13.89vh, 200px); left: 280px; right: 326px; text-align: center; color: var(--fg); text-shadow: 0 2px 18px rgba(0,0,0,.55); z-index: 2; max-height: max(48px, calc(100dvh - var(--playback-space,90px) - var(--instrument-height,230px) - clamp(100px,13.89vh,200px) - 16px)); overflow: auto; scrollbar-width: thin; }
body.program-flight .experiment-result h2 { margin: 0 0 18px; font-size: clamp(32px,5.2vh,64px); font-weight: 300; line-height: 1.15; }
body.program-flight .experiment-result p { margin: 8px 0; font-size: 16px; line-height: 1.6; }
body.program-flight .experiment-result .result-note { font-size: 13px; color: var(--dim); }
body.program-flight .experiment-result p:empty { display: none; }
body.program-flight.clean .experiment-result { visibility: hidden; }
@media (max-width: 900px) { body.program-flight .experiment-result { left: 22px; top: 86px; } }
@media (max-width: 650px) {
  body.program-flight #status-dock { max-height: min(30dvh,304px); }
  body.program-flight .experiment-result { right: 22px; top: calc(min(30dvh,304px) + 36px); max-height: max(48px,calc(70dvh - var(--playback-space,90px) - var(--instrument-height,230px) - 52px)); }
}
body.program-flight .flight-hud { position: fixed; inset: 0; width: 100%; height: 100%; pointer-events: none; z-index: 1; }
body.program-flight.clean .flight-hud { opacity: 0; }
body.program-flight #b-ui-restore { display: none; position: fixed; right: 22px; bottom: 22px; z-index: 3; align-items: center; gap: 7px; background: var(--glass); }
body.program-flight.clean #b-ui-restore { display: inline-flex; }
body.program-flight #b-ui svg { vertical-align: middle; }
body.program-flight.clean .flight-trajectories, body.program-flight.clean .visual-controls { visibility: hidden; }
body.program-flight .visual-controls { flex: 0 0 280px; margin-left: auto; padding: 16px 18px; border: 1px solid var(--line); border-radius: 18px; background: var(--glass); font-size: 12px; }
body.program-flight .visual-controls[hidden] { display: none; }
body.program-flight .visual-controls h2 { margin: 0 0 14px; font-size: 14px; font-weight: 400; color: var(--dim); }
body.program-flight .visual-switches { display: flex; gap: 6px; }
body.program-flight #trajectory-legend { display: grid; grid-template-columns: 1fr 1fr; gap: 9px 12px; margin-top: 14px; font-size: 11px; pointer-events: none; }
body.program-flight #trajectory-legend[hidden] { display: none; }
body.program-flight .controls button:disabled { opacity: .4; cursor: default; }
body.program-flight .visual-controls button:disabled { opacity: .4; cursor: default; }
body.program-flight .runs { display: grid; grid-template-columns: 1fr; gap: 6px; margin-bottom: 10px; }
body.program-flight .runs button { text-align: left; padding: 8px 10px; font-size: 13px; }
body.program-flight .runs button small { display: block; font-size: 10px; opacity: .62; margin-top: 2px; }
body.program-flight .runs button.ok::after { content: ' ●'; color: var(--mint); }
body.program-flight .runs button.fail::after { content: ' ●'; color: var(--hot); }
body.program-flight .runs button.on.ok::after, body.program-flight .runs button.on.fail::after { color: #0d1714; }
body.program-flight #fnote { font-size: 11px; color: var(--dim); line-height: 1.5; margin: 6px 0 0; }
body.program-flight #fnote code { font-size: 10px; color: var(--fg); word-break: break-all; }
body.program-flight #t-marks span.ign { background: var(--mint); }
body.program-flight #t-marks span.td { background: #fff; width: 3px; }
body.program-flight .shot { left: auto; right: 36px; text-align: right; bottom: 104px; }
body.program-flight.cinema .shot { bottom: calc(6.5vh + 96px); }
@media (max-height: 520px) {
  body.program-flight #title { top: 12px; }
  body.program-flight #title h1 { font-size: 28px; }
  body.program-flight #title p { font-size: 11px; }
  body.program-flight #title .sub { display: none; }
  body.program-flight #fpanel { top: 86px; max-height: max(0px, calc(100dvh - var(--playback-space, 90px) - var(--instrument-height, 280px) - 98px)); padding: 8px 12px; }
}
`;

export const FLIGHT_UI = `
<header id="title">
  <h1 aria-label="QQ飛車 · Jet-Car Mini">QQ飛車<span>Jet-Car Mini</span></h1>
  <p>冷气推力 · 飞出高架 · 着陆</p>
  <p class="sub" id="techline">10 喷口冷气推力套件 · XPBD 软体 · 闭环控制</p>
</header>
<aside id="fpanel" class="panel">
  <h2>飞行记录</h2>
  <div id="runs" class="runs"></div>
  <p id="fnote"></p>
</aside>
<button id="b-ui-restore" title="显示界面 (H)" aria-label="显示界面"><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7S2 12 2 12Z"/><circle cx="12" cy="12" r="3"/></svg></button>`;

export const FLIGHT_GROUPS = {
  trajectories: '<button id="b-trail" aria-pressed="true" title="从起点到当前时刻的完整轨迹">轨迹</button><button id="b-plan" aria-pressed="false">规划</button><button id="b-explore" aria-pressed="false">探索</button>',
  modes: '<button data-mode="director" class="on">导演</button><button data-mode="free">自由</button><button data-mode="follow">跟随</button>',
  speeds: '<button data-speed="0.1">0.1×</button><button data-speed="0.25">0.25×</button><button data-speed="0.5">0.5×</button><button data-speed="1" class="on">1×</button>',
  view: '<button id="b-plume" class="on">喷流</button><button id="b-tele" class="on">遥测</button><button id="b-ui" title="隐藏界面 (H)" aria-label="隐藏界面"><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="m3 3 18 18M10.6 5.1 12 5c6.5 0 10 7 10 7a19 19 0 0 1-3.1 4.1M6.2 6.2A21 21 0 0 0 2 12s3.5 7 10 7a12 12 0 0 0 5.8-1.8M9.9 9.9a3 3 0 0 0 4.2 4.2"/></svg></button>',
};
