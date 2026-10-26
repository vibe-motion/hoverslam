# 干净工作副本复现核验（2026-10-27）

以 `git archive da2db3c` 导出仅含已跟踪文件的临时工作副本，再应用本次 package/生成提示/官方案例入口整理。
没有复制现有 node_modules、public/sims、Python 环境、未提交的 Worker 适配或在线仿真草稿。
环境为 Linux、Node 24.18.0、npm 11.16.0；声明最低 Node 22.12，未对每个 Node 版本逐一运行。

- `npm ci`：从 npm registry 安装锁定的 17 个包，通过；未新增依赖、未下载浏览器。
- 已跟踪的车模、静置状态、生成规格及 OSQP WASM 均可直接使用，无需 settle、spec 或重新编译求解器。
- `npm run demos:generate -- only=nonconvex-stop` 与 `-- only=nonconvex-pad`：分别完整运行刚体和 6 s 软体闭环，退出码均为 0。
- 两例 `.sim` 内所有 `metrics` 与本目录既有验收报告逐项相同，均 `success=true`。
- `npm run build`：通过。
- `npm run demos:view -- --port 5188`：仅本机监听，查看器加载生成的录制。核验时设置 BROWSER=none，避免自动打开外部浏览器，再在应用内检查。

两条 only 命令为节省等待时间同时运行；默认不带 only 时脚本仍按顺序生成两个案例。
既有全套 parity/controller 测试未重跑，不将其历史失败记作通过。
软体录制及临时中间文件始终不跟踪；本报告不包含录制数据。
