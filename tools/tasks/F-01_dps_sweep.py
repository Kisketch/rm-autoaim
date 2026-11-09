#!/usr/bin/env python3
"""F-01: re-decide three knobs under one canonical objective.

Until now different sweeps used different objectives (hit rate, hits/s,
"0.5*mean_hit + 2*sum_hps"), which is why B-04 said the fire window should be
30 deg and E-02 said 36 deg. This script fixes the objective once:

    DPS = hits per second x 10 HP

and re-sweeps the three knobs that were decided under different metrics:
fire window, side angle, association gate.

Output: docs/dps_sweep.md (+ config/default.yaml if a knob wins clearly)
"""

from __future__ import annotations

import re
from pathlib import Path

from _common import DAMAGE_PER_HIT, REPO, eval_videos

SWEEPS = {
    "max_fire_facing": [27, 30, 33, 36, 39],
    "side_angle": [15, 20, 25, 30],
    "max_match_distance": [0.25, 0.35, 0.45],
}
YAML_KEY = {"max_fire_facing": "max_fire_facing_deg", "side_angle": "side_angle_deg"}


def yaml_value(key: str) -> float:
    text = (REPO / "config/default.yaml").read_text(encoding="utf-8")
    yk = YAML_KEY.get(key, key)
    m = re.search(rf"^\s*{re.escape(yk)}:\s*([-\d.eE+]+)", text, re.MULTILINE)
    return float(m.group(1)) if m else 0.0


def write_value(key: str, value: float) -> None:
    p = REPO / "config/default.yaml"
    text = p.read_text(encoding="utf-8")
    yk = YAML_KEY.get(key, key)
    text = re.sub(rf"^(\s*{re.escape(yk)}:\s*)[-\d.eE+]+.*$",
                  lambda m: f"{m.group(1)}{value:g}", text, count=1, flags=re.MULTILINE)
    p.write_text(text, encoding="utf-8")


def main() -> int:
    lines = [
        "# 统一 DPS 口径后的三旋钮复核（F-01）",
        "",
        "**口径**：`DPS = hits/s × 单发伤害`，17 mm 弹丸取 **10 HP**。",
        "hits/s 已经把命中率与射频合在一起，所以排名等价于“每秒打掉多少血”；",
        "命中率只作为平手时的次要指标（它衡量弹药效率，不直接等于伤害）。",
        "",
        "**为什么要重跑**：B-04 用命中率×发数判 30° 最优，E-02 用另一个加权分判 36° 最优。",
        "两个结论矛盾，根源不是数据而是口径。此脚本只认 DPS。",
        "",
    ]

    adopted = []
    for knob, values in SWEEPS.items():
        cur = yaml_value(knob)
        rows = []
        for v in values:
            m = eval_videos(["--set", f"{knob}={v}"], f"dps_{knob}_{v}")
            mean_hit = (m["3"][0] + m["4"][0]) / 2
            mean_hps = (m["3"][1] + m["4"][1]) / 2
            rows.append((v, m["3"], m["4"], mean_hit, mean_hps, mean_hps * DAMAGE_PER_HIT))
            print(f"{knob}={v}: v3 {m['3'][0]:.1f}%/{m['3'][1]:.2f}  "
                  f"v4 {m['4'][0]:.1f}%/{m['4'][1]:.2f}  DPS {mean_hps * DAMAGE_PER_HIT:.1f}")

        best = max(rows, key=lambda r: (r[5], r[3]))
        base = min(rows, key=lambda r: abs(r[0] - cur))
        gain = 100.0 * (best[5] - base[5]) / max(1e-9, base[5])
        no_hit_regression = (best[1][0] >= base[1][0] - 1.0) and (best[2][0] >= base[2][0] - 1.0)
        adopt = gain >= 0.5 and no_hit_regression and best[0] != cur

        lines += [
            f"## `{knob}`（当前 {cur:g}）",
            "",
            "| 值 | 3.avi 命中率 / hits/s | 4.avi 命中率 / hits/s | 平均 DPS |",
            "| --- | --- | --- | --- |",
        ]
        for v, v3, v4, mh, mhps, d in rows:
            mark = "**" if v == best[0] else ""
            lines.append(f"| {mark}{v:g}{mark} | {v3[0]:.1f}% / {v3[1]:.2f} | "
                         f"{v4[0]:.1f}% / {v4[1]:.2f} | {mark}{d:.1f}{mark} |")
        lines += [
            "",
            f"- 最优 `{best[0]:g}`，相对当前 `{cur:g}` 的 DPS 变化 **{gain:+.1f}%**；"
            f"命中率是否回退：{'否' if no_hit_regression else '是'}。",
            f"- 处置：{'**采纳**，写回配置。' if adopt else '保持原值。'}",
            "",
        ]
        if adopt:
            write_value(knob, best[0])
            adopted.append(f"{knob}: {cur:g} → {best[0]:g}")

    lines += [
        "## 结论",
        "",
        f"- 采纳的改动：{('; '.join(adopted) if adopted else '无（三个旋钮在原值下 DPS 最优）')}。",
        "- 口径统一后，以后所有扫描都只看 DPS；命中率只用于解释“为什么 DPS 变了”。",
        "- 注意：DPS 只看伤害，不看弹药成本。比赛里弹量有限时，应改用"
        "“期望伤害 / 期望消耗”，那是另一个指标，需要弹量数据。",
        "",
        "*评测是虚拟射击；DPS = 命中数/时长 × 10。*",
    ]

    out = Path("docs/dps_sweep.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nadopted: {adopted}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
