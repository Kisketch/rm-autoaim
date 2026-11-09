#!/usr/bin/env python3
"""E-02: do the two 'best' knobs interact? q_yaw x max_fire_facing grid.

B-03/B-04 swept them one at a time. If the best value of the fire window depends
on the yaw process noise (or vice versa), a 1-D sweep is misleading.

Output: docs/parameter_interaction.md
"""

from __future__ import annotations

from pathlib import Path

from _common import eval_videos

QYAWS = [1.0, 3.0, 8.0]
WINDOWS = [24, 30, 36]


def main() -> int:
    grid: dict[tuple[float, int], tuple[float, float, float, float]] = {}
    for q in QYAWS:
        for w in WINDOWS:
            m = eval_videos(["--set", f"q_yaw={q}", "--set", f"max_fire_facing={w}"], f"ix_{q}_{w}")
            grid[(q, w)] = (m["3"][0], m["3"][1], m["4"][0], m["4"][1])
            print(f"q_yaw={q:<4} window={w}: v3 {m['3'][0]:.1f}%/{m['3'][1]:.1f}  "
                  f"v4 {m['4'][0]:.1f}%/{m['4'][1]:.1f}")

    def score(cell):
        return 0.5 * (cell[0] + cell[2]) + 2.0 * (cell[1] + cell[3])

    best = max(grid.items(), key=lambda kv: score(kv[1]))
    best_q = max(QYAWS, key=lambda q: max(score(grid[(q, w)]) for w in WINDOWS))
    best_w = max(WINDOWS, key=lambda w: max(score(grid[(q, w)]) for q in QYAWS))

    lines = [
        "# 参数交互：q_yaw × 开火窗口（E-02）",
        "",
        "两个参数各自单变量扫描都出现过“疑似最优值”，这里做 3×3 网格看它们是否互相影响。",
        "",
        "| q_yaw | 窗口 24° | 窗口 30° | 窗口 36° |",
        "| --- | --- | --- | --- |",
    ]
    for q in QYAWS:
        cells = []
        for w in WINDOWS:
            c = grid[(q, w)]
            cells.append(f"{c[0]:.1f}% / {c[2]:.1f}%")
        lines.append(f"| {q:g} | " + " | ".join(cells) + " |")

    lines += [
        "",
        "单元格 = 3.avi 命中率 / 4.avi 命中率；综合分 = 0.5·(两视频命中率均值) + 2·(射频和)。",
        "",
        "## 结论",
        "",
        f"- 空格子里的最优组合是 `q_yaw={best[0][0]:g}` + `窗口 {best[0][1]}°`（综合分 {score(best[1]):.1f}）。",
        f"- 固定窗口时最好的 q_yaw = {best_q:g}；固定 q_yaw 时最好的窗口 = {best_w}°。",
        ("- 两者一致 → 没有明显交互，单变量扫描的结论可以照用。" if (best_q, best_w) == best[0] else
         "- 两者不一致 → **存在交互**，单变量扫描会误导，应以网格最优为准。"),
        "- 无论哪种，最终仍要过真实视频门禁（命中率不降、射频不降）。",
        "",
        "*每个格子都是完整评测（含检测缓存）。*",
    ]
    out = Path("docs/parameter_interaction.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nbest={best[0]}  -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
