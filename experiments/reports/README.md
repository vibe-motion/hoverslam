# experiments/reports — 实验报告

One folder per study, written for the project owner (Chinese). Each `README.md`: question, method (models,
controllers, cost, optimizers, budgets), setup (configs, presets, seeds, spec hash), results (tables, SVG figures in
the same folder), conclusions and limitations. Every number must be reproducible from a config in
`experiments/configs/` and a run stamp in `experiments/runs/` (git-ignored); scripts that make the figures live
next to the study's tools (`tools/flight/<study>/`).

- `barrel-roll/` — MPC: leave the viaduct, jump, roll 360° about the long axis, land softly (twin + soft body)
- `gas-late-burn/` — MPC (gradient / sampling / hybrid): is "free fall first, burn late" the gas-optimal soft landing?
- `gas-two-costs/` — the FALL script's two experiments: MPC with a cost without / with a gas term ("lower the car slowly" vs
  "suicide burn"), gas used by each, trajectories for the film
- `touchdown-calibration/` — grey-box fit of the twin's touchdown model to designed soft-body landings (`coldgas-rigid@2`)
- `drive-brakes-steering/` — kit rear arms, brakes with ABS and steering after the landing (`coldgas-rigid@3`)
- `demos/` — 当前正式非凸落地刹停与定点降落，配置、验证范围和复现入口。
- 旧双层二开记录移至 `../archive/dual-development/reports/`，保留历史失败与版本依据。
