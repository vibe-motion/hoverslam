# PR 测试失败对照核查（2026-10-27）

结论：在同一运行环境下，上游与本 PR 均为 **95 passed / 3 failed**，失败项相同。
本轮没有发现 PR 新增的测试失败，因此没有修改实现、测试容差或基准数据。
该结论限于下述测试及工况，不等同于所有新增功能均已由现有测试覆盖。

## 对照方法

- 上游：`fe133fbeaa4846f54584872602d4ba6f90f28e6d`。
- PR：`e282a21b40f21ea80dbffda91b9c63c2ddb5bfb5`。
- 分别使用 `git archive <ref>` 导出独立源码副本，不带入当前工作区未提交文件和录制。
- 两侧使用相同 Node 24.18.0、Python 3.13.12、NumPy 2.5.4、pytest 9.1.1；共享同一已安装的 Node 依赖目录。
- 每次从对应副本执行命令，并设置 `PYTHONPATH=<该副本>/python`；已确认 `coldgas.spec.ROOT` 指向对应副本，避免 editable 安装误用当前工作区。

```sh
npm run spec:check
PYTHONPATH="$PWD/python" /path/to/shared/venv/bin/python -m pytest python/tests -q --tb=short
node tests/parity/check.mjs
```

| 检查 | 上游 | PR |
| --- | --- | --- |
| 生成规格检查 | 通过 | 通过 |
| 完整 Python 测试（含双向 parity） | 95 通过 / 3 失败，226.10 s | 95 通过 / 3 失败，227.55 s |
| JS parity 独立运行 | 仅重算 `pad-mppi` 失败，退出码 1 | 同左，完整标准输出逐字节相同 |
| 已有 JS / Python golden 之间交叉比较 | 全部通过 | 全部通过 |

没有用 `npm test` 的短路退出代替剩余测试；规格、parity 与完整 Python 测试分别运行。

## 失败归因

### 控制器测试的进程参数长度超限

两侧 `test_pd_and_mlp_match_js` 均在 `python/tests/conftest.py:16` 的 `subprocess.run` 失败：

```text
OSError: [Errno 7] Argument list too long: '.../bin/node'
```

测试将 300 个状态和多组策略序列化到同一个 `node -e` 参数，超过当前 Linux 环境的参数长度限制，
尚未进入 Node 控制器执行。上游测试及调用辅助函数在本 PR 中没有改动。这是上游测试在当前环境下已有的问题，按范围要求不修复。

### 两个方向的 pad-mppi 基准复现失败

`test_python_reproduces_js_goldens` 和 `test_js_reproduces_python_goldens` 都因 `pad-mppi` 失败。
独立调用两侧的原始 parity 比较函数，保存并比较完整轨迹和误差报告，结果为：

- 上游与 PR 的 JS 重算结果完全相同，仅 `spec_hash` 不同；两侧 Python 重算结果亦如此。
- 上游与 PR 的完整误差报告完全相同，均为 261 行，事件相同，无缺失字段。
- 非接触段 `u_cmd` 最大绝对及相对偏差均为 `1.0871759032263228e-7`，超过原有 `1e-9` 相对容差。
- 当前环境下 JS 与 Python 的重算轨迹彼此完全相同，仅 `producer` 标识不同。
- 既有两套 golden 彼此一致，但与当前环境重算结果存在上述偏差；基准、parity 工具和 MPPI 算法在本 PR 中均未改动。

因此这是上游已可复现的“当前重算结果与历史 golden 不一致”，不是本 PR 导致的 JS/Python 孪生分歧。
本次没有进一步判定历史 golden 生成环境或此前变更为何导致差异，也没有重新生成 golden 或放宽容差。

## 处置边界

本轮只补充核查记录。保留上述上游失败，PR 仍不宣称全套测试通过。
两个新增非凸 demo 的干净复现证据见 [REPRODUCTION.md](REPRODUCTION.md)。
在线闭环及 Worker 草稿未参与本轮测试或提交，软体录制不纳入 Git。
