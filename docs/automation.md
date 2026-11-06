# 自动化：能自动到哪一步

## 1. 参数闭环（已实现，`tools/auto_improve.py`）

```bash
python3 tools/auto_improve.py --iterations 8 --candidates 3 --max-minutes 90 --dry-run
python3 tools/auto_improve.py --iterations 8 --candidates 3 --max-minutes 90 --commit
```

一轮 = 采样一组参数 → 跑两条真实视频 + 仿真 → 打分 → **只有门禁全过且分数提升才接受**。
接受时写回 `config/default.yaml`，`--commit` 还会自动 `git commit`。

- 搜索空间：`q_yaw`、`q_r`、`q_xyz`、`r_yaw`、`singer_tau`、`max_fire_facing`、`switch_hysteresis`（有界，非全空间）
- 目标函数：`0.45·平均命中率 + 0.45·平均射频 + 0.10·仿真均值`——射频与命中率同权，防止"少开火刷命中率"
- 门禁：任一视频命中率 −0.8 pp / 射频 −0.3 hits/s / 仿真 −3 pp 即拒绝
- 日志：`tools/auto_improve_log.jsonl`，每行一个候选，可回放

实测一轮（1 候选）约 2 分钟：基线 67.6/68.6、10.6/10.0、仿真 78.3；候选 67.2/69.3 被"分数未提升"拒绝。
一次 8×3 的夜间跑约 40–60 分钟，适合挂计划任务。

## 2. OpenCode 插件（`tools/opencode/rm-guard/`）

把回归门禁暴露成工具 `rm_gate`，任何改动后调用一次即可，不用记命令。

安装（项目级）：

```bash
mkdir -p .opencode/plugins
cp -r tools/opencode/rm-guard .opencode/plugins/rm-guard
```

或全局（`~/.config/opencode/opencode.json`）：

```jsonc
{ "plugins": ["C:/Users/Lgnatius/Desktop/RM/autoaim-takeover/tools/opencode/rm-guard"] }
```

环境变量 `RM_REPO` 指定仓库根目录。**注意：本机尚未加载验证过**（需要重启/重载 OpenCode 才会生效），
所以它是"可选增强"，不加载也不影响 `tools/regression.py` 本身。

## 3. 边界：什么能自动，什么不能

| 层 | 能自动吗 | 原因 |
| --- | --- | --- |
| 参数搜索 | ✅ 完全自动 | 有目标函数、有门禁、可复现 |
| 回归判断 | ✅ 完全自动 | `regression.py` + CI |
| 结构改动（换模型、加前馈） | ⚠️ 半自动 | 需要代理写代码，门禁负责"不许退化" |
| 发现新方向 | ❌ 不能 | 需要人/大模型判断，脚本只能验证 |

所以"不断自动提升"的正确形态是：**人来定方向 → 代理写改动 → 门禁裁决 → 只有净收益才进入 main**。
插件和脚本负责中间那段的自动化与守门，不能替代方向判断。
