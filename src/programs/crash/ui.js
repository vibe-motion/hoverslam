// DOM of the crash viewer (mounted into #hud).
export const CRASH_UI = `
<header id="title">
  <h1 aria-label="QQ飛車 · Jet-Car Mini">QQ飛車<span>Jet-Car Mini</span></h1>
  <p>高架坠车 · 软体破坏仿真</p>
  <p class="sub" id="techline">XPBD 节点-梁晶格 · 塑性屈服 · 断裂 · GPU 四面体嵌入</p>
</header>
<aside id="panel" class="panel">
  <h2>坠落姿态</h2>
  <div id="presets" class="presets"></div>
  <details id="custom">
    <summary>自定义参数</summary>
    <label>车速 <output id="o-speed">70</output> km/h<input id="i-speed" type="range" min="40" max="110" value="70" /></label>
    <label>驶出角度 <output id="o-angle">15</output>°<input id="i-angle" type="range" min="6" max="30" value="15" /></label>
    <label>俯仰角速度 <output id="o-pitch">0.3</output><input id="i-pitch" type="range" min="-2.5" max="2.5" step="0.05" value="0.3" /></label>
    <label>侧翻角速度 <output id="o-roll">-1.2</output><input id="i-roll" type="range" min="-3.5" max="3.5" step="0.05" value="-1.2" /></label>
    <label>偏航角速度 <output id="o-yaw">-0.2</output><input id="i-yaw" type="range" min="-2" max="2" step="0.05" value="-0.2" /></label>
    <label>结构强度 <output id="o-strength">1.00</output><input id="i-strength" type="range" min="0.5" max="1.6" step="0.05" value="1" /></label>
    <div class="row"><button id="b-random">随机姿态</button><button id="b-run" class="primary">开始计算</button></div>
  </details>
  <div id="progress" class="progress" hidden><div class="bar"><i></i></div><span>计算中</span></div>
  <h2>数据</h2>
  <dl id="stats" class="stats"></dl>
  <ol id="ticker" class="ticker"></ol>
</aside>
<div id="endcard" class="endcard"></div>`;

export const CRASH_GROUPS = {
  modes: '<button data-mode="director" class="on">导演</button><button data-mode="free">自由</button><button data-mode="follow">跟随</button>',
  speeds: '<button data-speed="0.05">0.05×</button><button data-speed="0.2">0.2×</button><button data-speed="0.5">0.5×</button><button data-speed="1" class="on">1×</button>',
  view: '<button id="b-heat">应变热图</button><button id="b-lattice">晶格</button><button id="b-sound" title="声音">声音</button><button id="b-ui" title="隐藏界面 (H)">⤢</button>',
};
