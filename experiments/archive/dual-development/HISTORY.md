# 双层非线性飞行控制

`dual` 是新增的 JS 控制器，使用随项目携带的 OSQP WASM。现有 `mpc`、刚体孪生、软体仿真和轨迹格式保持兼容。普通用户仍只需要原有 Node/npm 环境；不需要安装 acados、Python 求解器或 Emscripten。

```sh
npm run flight:run -- task=escape-soft-landing controller=dual preset=flat seed=0 out=/tmp/dual-flat.json
node tools/sim/flight.mjs controller=dual preset=flat seed=0 dur=4.5 out=dual-flat outDir=/tmp/dual-soft
node tools/flight/bench.mjs experiments/configs/dual-landing.json out=/tmp/dual-example
```

刚体输出使用原 `coldgas-traj@1`，软体输出使用原 `.sim`。已有查看器继续读取这些文件。浏览器求解器本身可直接加载，但现有 flight 页面仍以轨迹/录制播放为主，本次没有将页面改为在线控制界面。

## 组织

| 模块 | 职责 |
| --- | --- |
| `src/flight/guidance/model.js` | 连续空中预测：六自由度、气体质量、实际阀门动态、喷口力和力矩；提供局部状态扰动与阶段雅可比 |
| `src/flight/guidance/optimizer.js` | 固定阶段时间的稀疏多重射击 SQP，非线性 rollout 线搜索，只接受目标下降的控制更新 |
| `src/flight/guidance/index.js` | 上层剩余飞行规划、时间候选选择，下层短预测窗口跟踪，按时间采样上层状态和阀门前馈 |
| `src/flight/guidance/defaults.json` | 两层规模、频率、权重和信赖域 |
| `src/flight/qp/` | OSQP C 桥接、ESM 封装和预编译 WASM |
| `vendor/osqp/`、`tools/solver/` | 固定版本源码、许可证和维护者复建入口 |

加载使用 `await loadController('dual')`；初始化后 `act(state)` 保持同步，适配现有两种仿真步进。其他控制器不加载 WASM。直接同步 `createController('dual')` 需此前完成异步加载。

上层首次使用 PD 的模拟轨迹作初值，随后复用已有计划。实际控制动作由两层优化结果产生；不是在 PD 输出上叠加一个名为 MPC 的修正。QP 求解失败会显式报告，不能当作有效优化结果。

默认上层为 16 段、最多 3 次 SQP，每 0.8 秒重规划；下层为 6 段、0.6 秒预测、1 次 SQP，每 5 个控制步重规划。上层保留首次选定的绝对着陆时刻，避免每次重新估计剩余时间而不断推迟着陆。终端状态通过加权代价逼近，尚不是硬等式约束；`statistics.lastUpperDiagnostics.terminalError` 可查看实际预测偏差。

## 数学与物理边界

状态为位置、速度、四元数、机体系角速度、剩余气体质量和十个阀门开度，共 24 维；优化局部增量用三维姿态扰动，共 23 维。控制变量为十个喷口的指令开度。预测使用原推进/质量参数，移除指令死区以获得闭阀附近可用的梯度；真实被控对象保留原死区。

每次 SQP 将阶段动力学线性化成等式约束，加入输入范围、状态信赖域、气体和阀门界，构成凸 QP。OSQP 只处理该 QP；非凸动力学的局部搜索在 JS 层。控制更新后重新积分完整非线性模型，用实际目标值决定是否接受。

空中两层预测面向着陆，不包含接触和车身变形。触地后切换到仅刹停控制器，不再求解飞行或地面轨迹优化；轮地接触与软体变形仍由真实被控对象计算。上层时间目前采用有限个候选，不能称为连续自由终端时间优化。可选回收案例已增加推进组离散启停（见下文），但仍没有障碍规避、翻转阶段约束或全局最优保证，因此不能将本实现表述为完整 Falcon 9 制导系统。扩展这些问题需要明确增加相应决策变量和约束。

悬停起飞与外部脚本驱动任务仍会明确拒绝。定点着陆任务会生成水平减速参考；已验证的 flat / seed 0 案例见下文，不代表任意目标都可达。

默认开启 `ground.enabled`，控制职责分为：

- `src/flight/landing-modes.js`：着陆任务的上层模式管理。首次触地选择地面模式，回弹不会自动重启飞行；后续任务可通过 `controller.setMode('flight' | 'ground')` 明确切换。
- `src/flight/guidance/index.js`：仅在飞行模式运行两层规划。切换时清空上下层轨迹、缓存指令、着陆时刻、PD 初值生成器和当前诊断；累计求解次数保留用于运行统计。地面模式下 `predict()` 返回空计划。
- `src/flight/controllers/ground-stop.js`：初版行驶控制器只负责刹停，输出 100% 车轮制动、零转向和零 RCS。达到停车速度后继续保持制动；沿用原制动压力动态和 ABS，不改变物理模型。

飞行模式下检测到离地边沿时，仅执行一次制动、转向命令清零；之后保持该命令，不重置执行器物理状态。地面模式下的短暂回弹不触发飞行模式复位。`statistics` 提供 `mode`、`modeTransitions`、`departureResets` 和 `flightActive`。`ground.enabled:false` 保留 10 通道空中接口，并拒绝刹停任务。

未来地面油门约定为 0～100% 对应轮端油门，100～200% 在满油门基础上将同向 RCS 从 0 增至 80%；制动始终为 0～100%，无 RCS 补偿。该助推与电机本轮均未实现，不影响飞行模式的推力分配。

## 运行核验

采用少量实际运行检查，不新增测试框架。Node 和无头 Chrome 已运行同一预编译 QP；项目 Vite 构建通过。Chrome 中完整 `flat` 刚体闭环成功，耗时约 0.84 秒；浏览器与 Node 的结果只有浮点量级差异。

以下为加入地面控制前的首版结果。2026-10-27，本机、`escape-soft-landing`、seed=0，原默认 `mpc` 与新默认 `dual` 的结果如下。`dual` 通过原 worker 批处理入口运行；两者预测模型和目标并不相同，这组数据表示具体样例的计算开销与结果，不能作为相同优化问题的通用加速比。

| 预设 | 控制器 | 整段耗时 / s | 着陆成功 | 触地垂速 / m/s | 触地倾角 / ° | 耗气 / kg |
| --- | --- | ---: | --- | ---: | ---: | ---: |
| flat | mpc | 27.77 | 是 | 1.135 | 1.532 | 42.55 |
| flat | dual | 0.61 | 是 | 0.879 | 0.299 | 43.34 |
| nose | mpc | 23.98 | 是 | 1.211 | 0.323 | 50.04 |
| nose | dual | 0.64 | 是 | 0.806 | 0.286 | 51.76 |

两例说明首版空中着陆闭环可用，且计算开销没有增加；尚未覆盖其他预设、随机初值及复杂机动。新控制器耗气略高，并不代表所有性能指标都优于原控制器。

软体 `flat` 原入口运行 4.5 秒仿真，耗时 142.01 秒，其中控制规划累计 0.56 秒（上层 3 次、下层 24 次）。3.03 秒触地，垂速 0.989 m/s、倾角 0.24°、角速度 0.008 rad/s；通过现有触地及随后 1 秒保持判据。车辆随后继续滑行，在 4.39 秒发生路缘/车身碰撞，记录中 `success=true` 与 `hull_strike_any=true` 同时成立，不能把任务判据通过理解为全程无碰撞。

原 MPC 的软体对照仅运行 3.5 秒仿真，耗时 140.63 秒、规划累计 23.56 秒；因保持窗口尚未结束，结果是 `hold_incomplete`。两段时长不同，不直接比较整段耗时或成功率。软体总耗时主要来自既有高频结构仿真，本次没有修改其步进与精度取舍。

本轮输出保存在 `/tmp/hoverslam-dual-final/`（两例刚体轨迹和汇总）、`/tmp/hoverslam-dual-soft-final/`（软体录制和指标）。这些是本机临时结果，长期复现以本页命令及 `experiments/configs/dual-landing.json` 为准。

## 落地刹停复现

```sh
node tools/flight/bench.mjs experiments/configs/dual-land-stop.json out=/tmp/dual-land-stop
node tools/sim/flight.mjs controller=dual task=escape-land-stop preset=flat seed=0 dur=6 out=dual-brake-flat outDir=public/sims
# 打开 ?program=flight&run=dual-brake-flat&mode=follow
```

以下为此前地面 CEM 制动与转向版本的历史结果。seed=0 的刚体 `escape-land-stop` 样例中，flat 和 nose 均完成着陆与刹停，全程无路缘事件和车身碰撞；触地后分别用 1.804 秒、2.087 秒达到停车速度。两例均实际输出了非零转向，触地锁定后喷口指令全零。这是两个具体样例的运行结果，不是全工况验收。

播放器提供“回到起点｜播放/暂停｜循环”控件，默认播放结束后停止；回到起点时暂停，循环需手动开启。页面继续播放计算完成的录制，在线仿真界面仍暂停实施。

软体 `flat`、seed=0 实际运行 7 秒仿真：3.03 秒触地，5.20 秒达到停车判据（触地后 2.17 秒，距离 19.12 米），直到 7 秒结束未发生路缘事件或车身碰撞。计算耗时 308.92 秒，空中规划约 0.73 秒、地面规划约 1.51 秒；保留原 8000 Hz 软体求解。该录制在总统计修正前启动，元数据顶层 `plan_s` 仅含空中部分，地面耗时须读取 `controllerStats.ground.planMs`；当前实现已合并总统计。结果位于 `/tmp/hoverslam-dual-stop-soft/`，页面副本位于忽略跟踪的 `public/sims/flight-dual-stop-flat.*`。

### 模式管理与仅刹停版本

本版删除了 dual 内的地面 CEM 调用和相应默认调参项。seed=0、`escape-land-stop` 的 flat/nose 刚体样例均完成刹停，无车身碰撞和路缘事件，触地后分别 1.804/2.087 秒达到停车判据；整段计算约 0.87/0.90 秒。结果保存在 `/tmp/hoverslam-brake-only-final/`。该结果只覆盖两个样例。

软体 flat、seed=0 的 6 秒录制在 3.03 秒触地、5.18 秒刹停（触地后 2.15 秒、18.89 米），全程无路缘或车身碰撞，计算耗时约 230 秒。触地后的 296 条日志均为全刹车、零转向和零 RCS 指令。输出位于 `/tmp/hoverslam-brake-only-soft/`。录制启动时尚未补齐“首次调用已在空中”的边沿初始化，故元数据 `departureResets=0`；该分支已修正并单独验证一次清零，输出原本就是零，未改变此次录制的控制动作。刚体运行探针另行确认地面不再求解、旧预测清空、回弹不启飞行、显式重入飞行重新规划。未新增测试框架。

## 轨迹与规划可视化

右下角的矩形遥测面板提供“轨迹 / 规划 / 探索”，随“遥测”按钮统一显隐。轨迹默认开启，显示从片头到当前时刻的全部已行驶路径，不显示未来实际轨迹；规划和探索默认关闭。旧录制没有规划快照时会提示“此录制未保存规划”，不会在播放端补算。

- 实际路径：薄荷绿 `#93D3B8`，3 px。
- 上层剩余路径：琥珀金 `#F2C879`，2 px；下层短期预测：冰蓝 `#78C9ED`，2.5 px。
- 探索：本次上层求解各时间候选的最终轨迹，灰紫 `#A79BBB`，透明度 16%，最多 8 条；不是随机采样云，也不代表所有 SQP 迭代。
- 实际触地点：红色 `#D94A58` 的 ×；预测落点：蓝色 `#397DE0` 的 ＋。两者为固定屏幕尺寸、完全不透明、无描边的符号，位置投影到对应道路表面。

线条以车体质心位置绘制。采用 Canvas 投影叠加，保持屏幕像素宽度，作为诊断显示可穿过场景遮挡，不新增依赖。暂停冻结仿真时间，但旋转相机仍会更新投影；拖动进度可双向恢复对应时刻的路径和落点。进入地面模式后清除飞行规划与预测落点，实际路径继续到停车。

生成含真实规划快照的软体录制：

```sh
node tools/sim/flight.mjs 'controller=dual:{"capturePlans":true}' task=escape-land-stop preset=flat seed=0 dur=6 out=dual-plans-flat outDir=public/sims
# ?program=flight&run=dual-plans-flat&mode=follow&paused
```

`capturePlans` 默认关闭，仅开启时采集路径。`planningTrace` 存在软体录制 `meta.flight` 或刚体轨迹根对象内，每次重规划保存 `{t, mode, upper, upperTimes, lower, lowerTimes, candidates}`；位置为世界坐标，时间为绝对仿真时间。候选路径在下层更新时保留，直到下一次上层求解替换；地面模式记录空路径。播放仅查找当前时间之前的最新快照，并裁去上层/下层路径中已过去的部分，不启动在线求解。

带规划记录的软体 flat 样例保存了 25 次快照（0.76 秒首次空中规划，3.04 秒地面清空），6 秒仿真完整完成，触地后 2.15 秒刹停，无车身碰撞及路缘事件。构建通过；刚体样例开启记录前后的状态与指令逐项相同。

交互面板使用统一 CSS 像素布局：状态遥测保持右上，按底部信息区的实际高度限制可视高度，内容超出时内部滚动；喷口、制动转向与规划面板位于播放栏上方的可滚动信息区，播放栏换行和导演模式的留白会动态计入底部间距。窄于 900 px 或不高于 520 px 时，飞行记录也移入该区域；放大视口后自动恢复左侧位置。仪表用固定逻辑尺寸绘制并按设备像素比采样，不再随窗口高度放大而挤压其他面板。离线影片仍使用原全画幅 HUD 排版。

## 定点降落（precision-pad / flat）

`precision-pad` v2 在原软着陆要求上同时检查首次触地的水平落点距离 ≤ 1.5 m、水平速度模长 ≤ 0.5 m/s。目标中心沿用 `[-22, 0, 16]`；接地后的刹停不替代触地指标，硬件和初始条件采样规则均未修改。

上层 SQP 对水平位置和速度分别使用 32 边内接多边形约束（圆半径最多保守约 0.482%），每组有一个非负恢复松弛量。线搜索使用非线性重积分得到的真实圆约束违反量构造 merit；一旦实际预测可行，不接受破坏可行性的步长。时长候选优先选择预测可行解，再比较代价。下层窗口到达上层选中轨迹的终点时，缩短窗口到同一时刻并施加相同终端约束；较早的跟踪窗口不强求提前停车。原有不带定点终端限制的任务保持原求解行为。

`diagnostics.terminalConstraints` 分别报告位置/速度模长、约束残差、违反量与可行状态；`qpSolved` 仅表示数值求解状态。受约束求解中，后续 QP 失败会返回已接受的非线性轨迹并明确记录 `stop=qp_failure`、`qpSolved=false`，不会把该轨迹自动标为可行。所有候选不可行时会执行当前 merit 最优的恢复轨迹，最终任务成功仍以真实触地指标判定。规划快照保留上层终端残差，供录制后检查。

复现参数见 `experiments/configs/dual-precision-pad.json`。该案例固定 flat、seed 0，任务原有随机化得到速度 58.0328 km/h、出桥角 14.1098°；软体使用刚体输出中的同一 launch，不将默认 flat 的 64 km/h 混作相同初始条件。8 次上层 / 2 次下层迭代、0.4 s 上层更新，以及较大的终端速度权重仅属于该案例，不修改其他案例默认值。

```sh
pad_controller=$(node --input-type=module -e 'import fs from "node:fs"; console.log(JSON.stringify(JSON.parse(fs.readFileSync("experiments/configs/dual-precision-pad.json","utf8")).controllers[0]))')
node tools/flight/run.mjs task=precision-pad preset=flat seed=0 controller="$pad_controller" out=public/sims/dual-pad-rigid.json
node tools/sim/flight.mjs task=precision-pad preset=flat launch=public/sims/dual-pad-rigid.json controller="$pad_controller" out=dual-pad-flat dur=6 hz=100
```

浏览器入口 `?program=flight&run=dual-pad-flat&mode=follow&paused`。金色圆环表示目标区域，红色 × 表示实测触地点，蓝色 ＋ 表示动态预测触地点。结果卡显示位置误差、水平速度及各自阈值；成功状态来自对应仿真记录的实测结果。页面仍是闭环计算结果的录制回放，不开启在线仿真。

本次固定案例结果：刚体落点误差 0.09748 m、触地水平速度 0.03221 m/s、垂速 0.31334 m/s；软体对应为 0.09 m、0.224 m/s、1.104 m/s，倾角 7.86°，无车身碰撞、路缘碰撞，`success=true`。软体在 3.403 s 触地，6 s / 601 帧录制用时 207.626 s（含物理、求解与编码，其中控制规划统计为 7.64 s），不作实时性能承诺。完整指标保存在 `experiments/reports/dual-precision-pad/metrics.json`；这仅验证 flat / seed 0，不扩展为跨工况鲁棒性结论。

验证边界：构建通过；求解器的有界可行恢复、不可达残差、零次迭代与无约束输出保持检查通过。既有 `npm test` 在 JS `pad-mppi` 对历史 golden 的约 1.09e-7 指令差异处未通过；同进程移除新增水平速度判据后输出逐项相同，因此没有调整 golden 或放宽容差。该项不计作本次通过项。

另行执行的既有 Python 全套检查在中止时进度为 44%（98 项已收集），已出现一个 controller 和两个 parity 失败；controller 定位为调用 Node 子进程时 `OSError: [Errno 7] Argument list too long`。未将这次未完成的全套检查记为通过，也未扩大本任务去修改既有测试基础设施。


## 实验结果展示模板

实验结果保持原有顶部居中大字、无边框背景的样式：成功为薄荷绿，失败为红色，指标为白色，说明为灰色。结果不属于底部仪表卡片；右上遥测与右下规划面板的位置不变。页面在可用宽度内换行，高度不足时结果区域可滚动，并跟随“遥测／隐藏界面”开关。

`src/programs/flight/result.js` 提供统一内容模板 `resultTemplate({ title, success, metrics, details, note })`：`metrics` 与 `details` 是文字行数组，空行自动省略。现有飞行案例通过 `flightResult(verdict)` 填入触地指标、定点指标、刹停信息和用气说明；交互结果与离线影片读取同一份模板内容。新增实验只填字段，不另建结果卡片或改变字号、色彩、位置。

```js
resultTemplate({
  title: '定点降落', success: true,
  metrics: ['位置误差 0.09 m / 1.5 m ✓', '触地水平速度 0.224 m/s / 0.5 m/s ✓'],
  details: ['四轮着地 · 无车身碰撞'],
  note: '软体 · flat / seed 0',
});
```


喷口开度条以蓝色横线表示最小允许油门，红色横线表示最大允许油门。无额外约束的现有录制默认分别为 0 和 1，因此蓝线在底部、红线在顶部。白色标记仍是实际阀门开度，填充仍是相对额定推力。后续案例可在软体 `meta.flight.commandLimits` 或刚体轨迹根级 `commandLimits` 保存 `{min, max}`；字段支持统一标量或按喷口顺序排列的数组。刻度只反映录制所声明的指令边界，不会截断回放值，也不代表求解器已实施非凸限制。

## 非凸回收（40% 历史案例 / flat）

配置为 `experiments/configs/dual-recovery.json`，在定点降落任务上开启
`recovery: {enabled: true, minThrottle: 0.4}`，原硬件、气体参数、阀门动态及触地判据均不修改。

底部四个喷口组成共同点火组：滑行阶段十路指令全零；燃烧阶段四路底部指令分别限制在
[0.4, 1]，保留差动姿态控制，其余六路 RCS 可在 [0, 1] 内修正。因此不允许单独开启 RCS
绕过推进组最小推力限制。此约束只属于空中回收配置，触地后仍切到全刹车、零 RCS。

现有模型底部满推力约 34.906 kN，满气总重约 11.258 kN；40% 对应约 13.962 kN，
稳态最小推重比约 1.24。审计根据录制中的气量、温度、气压逐帧重算可用推力。
这里比较底部推力合力大小与重量；倾斜时世界竖直分量会更小。阀门仍有 15 ms 开启时间常数，
由零点火约 25 ms 后才超过重量，关闭时也会衰减。因此蓝色 40% 刻度是开启指令下界，
实际白色阀门标记和推力填充在过渡时允许经过下界以下，不能宣称瞬时实际推力跳变。

`guidance/recovery.js` 在每个预测窗口搜索有限个单燃烧区间时序，上层最多 9 个、下层最多 5 个。
每个时序固定阶段开关，交给原 SQP / OSQP 优化连续开度；输入绝对上下界同时用于初始非线性
rollout、QP 和线搜索。连续松弛解只提供搜索初值，不能直接成为执行指令。选解优先满足终端
位置/速度约束，再比较带违反量惩罚的代价。下层第一段覆盖实际执行保持时间，最终触地段可截短。

这是有限时序搜索与局部非线性优化，不保证混合整数全局最优。每个窗口只搜索一段连续燃烧，
但滚动闭环可以多次关机和重启。可配置 `switchPenalty` 对每次开关状态变化收费，计入上一条
实际执行指令到首段、以及预测段之间的切换；上层时长候选也计入该成本。未假设窗口结束时
必然关机，重规划不会清除上一条实际指令的开关状态。终端可行性仍优先，惩罚不会替代它。
本版没有硬性限制总点火次数或最小开关驻留时间。
案例借鉴“最小开启推力大于重量”的回收难点，不声称复现 Falcon 9 的真实发动机或飞行程序。

复现（仍为离线闭环计算后播放）：

```sh
recovery_controller=$(node --input-type=module -e 'import fs from "node:fs"; console.log(JSON.stringify(JSON.parse(fs.readFileSync("experiments/configs/dual-recovery.json","utf8")).controllers[0]))')
node tools/flight/run.mjs task=precision-pad preset=flat seed=0 controller="$recovery_controller" out=public/sims/dual-recovery-rigid.json
node tools/sim/flight.mjs task=precision-pad preset=flat launch=public/sims/dual-recovery-rigid.json controller="$recovery_controller" out=dual-recovery-flat dur=6 hz=100
node tools/flight/recovery-audit.mjs input=public/sims/dual-recovery-rigid.json
node tools/flight/recovery-audit.mjs input=public/sims/flight-dual-recovery-flat.sim
```

入口 `?program=flight&run=dual-recovery-flat&mode=follow&paused`。沿用完整实际轨迹、动态规划/
预测、实际与预测落点和定点目标环；底部喷口蓝色刻度为 40%，红色为 100%。探索线仍显示
上层各时长候选最终选中的轨迹，不额外显示每个离散开关候选。录制携带 `commandLimits`
和规划快照；播放器不运行求解器，在线仿真保持暂停实施。

首个直接沿用定点参数的非凸版本虽然刚体通过，但软体触地误差 2.08 m、水平速度 1.995 m/s、
倾角 33.79°，并发生车身碰撞，不能用于通过结论。随后仅调整控制参数：下层更新 100→40 ms，
网格 6×100→8×80 ms，途中姿态/角速率权重 30/5→90/15。无开关惩罚的修订版刚体误差
0.394 m、水平速度 0.387 m/s；软体误差 1.46 m、水平速度 0.038 m/s、垂速 0.644 m/s、
倾角 3.18°，无车身及路缘碰撞，满足原触地判据并随后刹停。位置裕量较小，不作鲁棒性结论。

该软体录制 6 s / 601 帧耗时 317.609 s，控制规划约 67.46 s；部分计算与其他试验并行，
不作为隔离性能基准。相比此前普通定点案例，非凸候选搜索明确增加了求解开销。完整指标与
失败对照保存在 `experiments/reports/dual-recovery/metrics.json`。未新增依赖或改变物理步长。


开关惩罚对照配置为 `experiments/configs/dual-recovery-switch.json`，每次切换成本为 2。
刚体仍通过（误差 0.300 m、水平速度 0.458 m/s），但包含初次点火的实际切换由 10 次增至
11 次，不能宣称该权重已降低闭环切换频率。769 条记录中的候选切换计数及对应惩罚逐项核验一致。
软体对照在最后记录到 3.25 s 后进程退出 143，未生成完整录制，终止原因未确认，不计为通过。
当前可视化展示已通过的 `dual-recovery.json`（switchPenalty=0）；惩罚实现及独立对照配置保留，
不将未完成结果覆盖已验收录制。审计的切换次数包括首次点火，统计窗口止于首次触地，不包含强制地面关机。


## 35% 回收与重启间隔惩罚

当前 `dual-recovery.json` 与 `dual-recovery-switch.json` 已切换为 35% 最小开启油门、
`switchPenalty: 2`。上述 40% 历史实验对应提交 `015a9d4` 的配置与计次惩罚实现，
其已生成录制保持原样，不把旧录制的 40% 刻度改成 35%。

上下层每个候选的新增成本为
`switchPenalty × Σ 1 / (本次开启时间 − 最近一次关闭时间)`，时间单位为秒。
只对 off→on 收费，关机不收费；首次点火没有历史关机时间，成本为零。
实际关闭时刻在输出指令发生 on→off 时更新，跨上下层重规划保存；预测中的关机时刻仅用于
该候选，不修改实际历史。重置任务或切换飞行模式会清除历史。候选评估使用实际阶段时长，
不按阶段数近似间隔，也不在预测窗口末尾人为添加一次关机。

正间隔直接取倒数，不添加任意平滑常数；零间隔重启成本为无穷大（权重为零时成本仍为零）。
原有正步长和控制周期保证可执行重启有正间隔。终端可行性仍优先于成本；该惩罚不等价于
硬性最小关闭时长，也不保证实际切换次数必然减少。

现有模型底部四喷口在初始气压下的 35% 合推力约 12.217 kN，初始重量约 11.258 kN，
稳态推重比约 1.085。仍保留阀门滞后，瞬时点火推力不保证大于重量。
新录制使用 `dual-recovery35-flat`，以避免与旧 40% 录制混淆。

当前版本复现命令：

```sh
recovery_controller=$(node --input-type=module -e 'import fs from "node:fs"; console.log(JSON.stringify(JSON.parse(fs.readFileSync("experiments/configs/dual-recovery.json","utf8")).controllers[0]))')
node tools/flight/run.mjs task=precision-pad preset=flat seed=0 controller="$recovery_controller" out=public/sims/dual-recovery35-rigid.json
node tools/sim/flight.mjs task=precision-pad preset=flat launch=public/sims/dual-recovery35-rigid.json controller="$recovery_controller" out=dual-recovery35-flat dur=6 hz=100
node tools/flight/recovery-audit.mjs input=public/sims/flight-dual-recovery35-flat.sim
# ?program=flight&run=dual-recovery35-flat&mode=follow&paused
```


35% 固定 flat / seed 0 实测：刚体落点误差 0.380 m、水平速度 0.352 m/s；
软体误差 0.27 m、水平速度 0.054 m/s、垂速 1.336 m/s、倾角 4.92°，原任务判据通过，
无车身或路缘碰撞，触地后 1.011 s 停车。软体 6 s / 601 帧计算 319.3 s，规划 67.04 s。
指令违规为零，沿途最小可用稳态推重比 1.0852；空中 4 次点火、3 次关机，三次重启关闭间隔
分别为 0.08、0.32、0.12 s。相对历史 40% 案例切换减少，但最低油门也同时改变，不能作单因素归因。

构建、首次点火/预测关机/跨窗口间隔边界核验，以及刚体记录中的 741 条候选倒数惩罚核验通过。
未扩大既有全套测试范围。指标见 `experiments/reports/dual-recovery35/metrics.json`。
