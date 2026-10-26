# 双层 MPC 二开试验归档

此处保存正式 demo 筛选前的配置、指标和设计演进，不作为当前推荐运行入口。
正式配置见 `experiments/configs/nonconvex-{pad,stop}.json`；运行 `npm run demos:generate`。

- `configs/`：早期着陆、刹停、定点、40%/35% 回收与惩罚试验配置。
- `reports/`：当时的实测指标，包含失败尝试与未完成核验，不追写为当前结果。
- `HISTORY.md`：迁移前完整双层控制说明，保留历史路径/命令及其版本语境；当前复现说明见 `docs/DUAL_GUIDANCE.md`。

对应实现版本：初版非凸回收 `015a9d4`；35% 与倒数重启惩罚 `e24e0c9`。
本地旧录制归档在 `experiments/runs/archive/dual-development/`，与当前录制一样全部 Git-ignored。
原项目其他算法/研究配置未删除；未跟踪在线仿真草稿和用户文件未移动、未纳入提交。
