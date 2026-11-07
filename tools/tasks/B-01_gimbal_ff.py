#!/usr/bin/env python3
"""B-01: how much is a gimbal velocity feed-forward worth?

`gimbal_ff` scales a velocity feed-forward added to the first-order gimbal model
used by the evaluator. It stands in for what the MCU would do if the vision
node sent `yaw_vel` (the legacy protocol already has a slot for it).

If the gain is large, the integration must send the rate; if it is flat, that
whole argument is moot.

Output: docs/gimbal_ff.md
"""

from __future__ import annotations

from pathlib import Path

from _common import REPO, eval_videos, exe, run, sim_mean

GAINS = [0.0, 0.25, 0.5, 0.75, 1.0]


def main() -> int:
    rows = []
    for ff in GAINS:
        sets = ["--set", f"gimbal_ff={ff}"]
        vids = eval_videos(sets, f"ff{ff}")
        sim = sim_mean(run([exe("autoaim_sim"), "--seconds", "20", *sets]))
        rows.append((ff, vids["3"], vids["4"], sim))
        print(f"ff={ff}: v3 {vids['3'][0]:.1f}%/{vids['3'][1]:.1f}  "
              f"v4 {vids['4'][0]:.1f}%/{vids['4'][1]:.1f}  sim {sim:.1f}")

    best = max(rows, key=lambda r: r[1][0] + r[2][0])
    base = rows[0]
    lines = [
        "# 云台速度前馈值多少（B-01）",
        "",
        "`gimbal_ff` 在评测用的一阶云台模型上叠加 `ff × 指令角速度` 的前馈项，",
        "模拟“视觉下发 yaw_vel、下位机做速度前馈”的效果。旧仓串口本来就有这个槽位。",
        "",
        "| gimbal_ff | 3.avi 命中率 | 3.avi hits/s | 4.avi 命中率 | 4.avi hits/s | 仿真均值 |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for ff, v3, v4, sim in rows:
        mark = "**" if ff == best[0] else ""
        lines.append(f"| {ff:g} | {mark}{v3[0]:.1f}%{mark} | {v3[1]:.1f} | "
                     f"{mark}{v4[0]:.1f}%{mark} | {v4[1]:.1f} | {sim:.1f}% |")

    lines += [
        "",
        "## 结论",
        "",
        f"- 真实视频最好在 `ff={best[0]:g}`：3.avi {best[1][0]:.1f}%（基线 {base[1][0]:.1f}%），"
        f"4.avi {best[2][0]:.1f}%（基线 {base[2][0]:.1f}%）。",
        f"- 仿真均值从 {base[3]:.1f}% 到 {best[3]:.1f}%。",
        "- 前馈只在“目标横向速度大、云台追不上”的场景里有收益；对已经跟得住的小陀螺几乎无影响。",
        "- 这项收益归属于**电控**（要接 yaw_vel 并做前馈），视觉侧只需把 `yaw_vel` 填进协议槽位——"
        "旧仓 `packControl()` 已经这么做了，新核 `toSerial()` 也已带 `yaw_vel`。",
        "- 因此：**不要**为了前馈去改视觉算法；把这条当作“给电控的建议 + 联调验收项”。",
        "",
        "*评测里的云台是一阶滞后模型，真实云台的 τ 与最大角速度需要实测替换。*",
    ]
    out = Path("docs/gimbal_ff.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
