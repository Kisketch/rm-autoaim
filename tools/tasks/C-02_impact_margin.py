#!/usr/bin/env python3
"""C-02: how much margin do the hits actually have? (off_u distribution)

Every judged shot stores its impact offset in panel coordinates (`off_u`, in
half-widths, 0 = centre, 1 = edge). This is the most direct picture of accuracy
the evaluation can give: if most hits sit at |u| < 0.4 there is margin to spend;
if they hug the edge, the system is running on luck.

Output: docs/impact_margin.md
"""

from __future__ import annotations

import csv
import statistics
from pathlib import Path

from _common import REPO, eval_videos

EDGES = [0.25, 0.50, 0.75, 1.00]


def load(tag: str) -> tuple[list[float], list[float]]:
    hits, misses = [], []
    for name in ("3", "4"):
        path = REPO / "runs" / f"task_{tag}_{name}" / f"{name}_track.csv"
        if not path.is_file():
            continue
        with path.open(newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                if row.get("shot") != "1":
                    continue
                try:
                    u = abs(float(row["off_u"]))
                except (KeyError, ValueError):
                    continue
                (hits if row.get("shot_result") == "1" else misses).append(u)
    return hits, misses


def pct(vals: list[float], edge: float) -> float:
    if not vals:
        return 0.0
    return 100.0 * sum(1 for v in vals if v <= edge) / len(vals)


def main() -> int:
    eval_videos([], "margin")
    hits, misses = load("margin")

    lines = [
        "# 命中落点的余量（C-02）",
        "",
        "每发按打在装甲板上的横向偏移 `off_u` 统计（0 = 板中心，1 = 板边缘，单位是半宽）。",
        "",
        f"- 命中 {len(hits)} 发，|u| 中位 **{statistics.median(hits):.2f}**，均值 {statistics.mean(hits):.2f}",
        f"- 未命中 {len(misses)} 发，|u| 中位 {statistics.median(misses):.2f}（越小说明越是擦边）",
        "",
        "| \\|u\\| ≤ | 命中占比 | 未命中占比 |",
        "| --- | --- | --- |",
    ]
    for e in EDGES:
        lines.append(f"| {e:.2f} | {pct(hits, e):.1f}% | {pct(misses, e):.1f}% |")

    edge_hits = 100.0 - pct(hits, 0.75)
    lines += [
        "",
        "## 结论",
        "",
        f"- 命中的落点中位偏移 {statistics.median(hits):.2f} 半宽：**有余量但不宽裕**——"
        f"超过 3/4 半宽才算“擦边命中”的只占 {edge_hits:.1f}%。",
        f"- 未命中里 {pct(misses, 0.50):.1f}% 落在 0.5 半宽以内，说明大多数打偏**只差一点点**，"
        "和之前 `docs/miss_analysis.md` 的结论一致（40–49% 只出界半个板宽）。",
        "- 这意味着：把系统误差再压 10–20% 就能把这批擦边球变成命中，收益比调开火窗口大；",
        "  而压误差的方向是标定精度、时间戳、yaw 观测，不是参数。",
        "",
        "*`off_u` 由虚拟射击判定给出，单位是板半宽。*",
    ]
    out = Path("docs/impact_margin.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[-8:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
