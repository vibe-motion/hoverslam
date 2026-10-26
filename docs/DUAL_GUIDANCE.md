# 双层非凸飞行与正式演示

正式二开 demo 仅保留“落地刹停.非凸”和“定点降落.非凸”。仓库跟踪源代码、配置、复现脚本和紧凑指标；
**所有软体录制、轨迹数据及其本地归档均不纳入 Git**。旧二开案例移至
[`experiments/archive/dual-development`](../experiments/archive/dual-development/README.md)，不再显示在默认演示列表。官方原有案例和算法保留；对应录制生成后仍可在列表中选择。

## 运行

需要 Node.js ≥ 22.12（推荐 24）与 npm；无需 Python 或重新编译求解器。车模、静置状态及 WASM 均已跟踪。

```sh
npm ci
npm run demos:generate                         # 两个案例，耗时数分钟/例
npm run demos:generate -- only=nonconvex-pad     # 仅定点降落
npm run demos:generate -- only=nonconvex-stop    # 仅落地刹停
npm run demos:view                            # 自动打开正式演示，默认暂停
```

播放入口：

- `?program=flight&run=nonconvex-stop&mode=follow&paused` — 落地刹停.非凸。
- `?program=flight&run=nonconvex-pad&mode=follow&paused` — 定点降落.非凸。

复现脚本先运行刚体取得任务随机化后的 flat / seed 0 launch，再以同一 launch 驱动软体闭环，
将录制写入忽略跟踪的 `public/sims/flight-nonconvex-*.sim`，刚体中间文件保留在打印出的临时目录。
两个案例均录制 6 s，100 Hz 记录，物理步长不变。浏览器只播放计算结果，
**在线仿真仍暂停实施**。默认停止于结尾；支持重置、暂停、循环和跟随视角旋转缩放。

## 共同非凸配置

两例配置分别为 `experiments/configs/nonconvex-stop.json`、`nonconvex-pad.json`。
底部四喷口共同点火：滑行时十路指令全零；开启时四路底部指令分别处于 [0.35, 1]，其余六路 RCS
可连续调节姿态。原硬件与阀门动态保持不变。初始底部最低稳态总推力约 12.217 kN，重量约
11.258 kN，推重比约 1.085。此处比较底部推力大小与重量，非倾斜后的竖直投影；阀门点火过渡
期间实际推力可以低于重量，35% 是指令边界。

每次重启成本为 `2 / (本次开启时间 − 最近一次关闭时间)`，时间单位秒。关机不收费；首次点火
无历史关闭时刻，不收费。实际关机时间跨重规划保存；预测关机只更新候选内部历史。终端可行性
优先，惩罚不是硬性最小停机时间。记录中包含候选评分和重启间隔，审计入口为：

```sh
node tools/flight/recovery-audit.mjs input=public/sims/flight-nonconvex-pad.sim
node tools/flight/recovery-audit.mjs input=public/sims/flight-nonconvex-stop.sim
```

## 两个任务的区别

定点降落继承 `precision-pad`：首次触地水平误差 ≤ 1.5 m、水平速度 ≤ 0.5 m/s，目标中心
[-22, 0, 16]。落地刹停使用继承 `escape-land-stop` 的 `level-land-stop`，不设落点靶心；以 `terminal_horizontal_speed`
指定接地水平速度参考，触地后由轮端制动完成停车。参考速度是规划代价目标，不是已满足的硬约束。
两例非凸推进配置相同；刹停例另外加强姿态/角速度权重，接地及保持倾角 ≤ 5°、接地角速度 ≤ 0.5 rad/s。
水平速度不设硬上限，4 m/s 仅作参考，任务与姿态权重不同于定点例。

首次触地由 `landing-modes.js` 切到地面模式，清空上下层飞行轨迹和指令，回弹不自动重启飞行。
初版地面控制器仅输出全刹车、零转向、零 RCS；不含油门助推。离地边沿仅一次复位制动与转向。

## 模块与边界

- `guidance/model.js`：原参数六自由度预测、气体质量和阀门动态；不预测软体变形与轮地接触。
- `guidance/optimizer.js`：稀疏多重射击 SQP，OSQP WASM 求解局部凸子问题，非线性 rollout 线搜索。
- `guidance/recovery.js`：有限单燃烧区间候选搜索；输入上下界同时用于初值、QP 和线搜索。
- `guidance/index.js`：上层剩余飞行规划及时间候选、下层短时跟踪，模式切换和执行历史。

随项目携带的 OSQP WASM 不需要运行时安装 acados、Python 求解器或 Emscripten。
每个预测窗口搜索有限个开关时序，滚动执行可多次重启，不保证混合整数全局最优或单次点火。
连续松弛解仅作初值，不能直接越过非凸限制执行。定点终端采用弹性多边形约束与实际圆残差；
QP 求解成功不等同于真实触地可行，最终以记录指标验收。

## 可视化与证据

实际轨迹为薄荷绿，上层规划为金色，下层预测为蓝色，探索为淡紫色时长候选最终轨迹；
实际落点红色 ×、预测落点蓝色 ＋，均不透明无描边。蓝/红刻度分别为录制声明的最小/最大油门，
白色保持实际阀门/推力显示。实验结果使用原大字模板。规划与预测按录制时间动态更新，暂停可绕车查看。

正式案例指标位于 `experiments/reports/demos/`，仅证明固定 flat / seed 0，不代表跨工况鲁棒性。
历史失败、性能记录和既有全套测试未通过项保存在归档 HISTORY.md；不改 golden 或放宽验收阈值。
