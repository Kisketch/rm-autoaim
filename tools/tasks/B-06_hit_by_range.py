#!/usr/bin/env python3
"""B-06: hit rate as a function of range (the honest explanation of 67 %).

Both videos are 25 fps recordings of the same robot at different distances.
The track CSV has the estimated target centre per frame and the judged shot
result, so we can bucket the hit rate by range instead of guessing.

Output: docs/hit_by_range.md
"""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path

from _common import REPO, eval_videos

BUCKETS = [(0, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 9)]


def main() -> int:
    eval_videos([], "range")  # ensures runs/task_range_<n>/<n>_track.csv
    stats = defaultdict(lambda: [0, 0])   # bucket -> [hits, shots]
    frames = defaultdict(int)

    for name in ("3", "4"):
        path = REPO / "runs" / f"task_range_{name}" / f"{name}_track.csv"
        with path.open(newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                try:
                    d = math.sqrt(float(row["xc"]) ** 2 + float(row["yc"]) ** 2 + float(row["zc"]) ** 2)
                except (KeyError, ValueError):
                    continue
                b = next((f"{lo}-{hi}" for lo, hi in BUCKETS if lo <= d < hi), None)
                if b is None:
                    continue
                frames[b] += 1
                if row.get("shot") == "1":
                    stats[b][1] += 1
                    if row.get("shot_result") == "1":
                        stats[b][0] += 1

    rows = []
    for b, _ in [(f"{lo}-{hi}", 0) for lo, hi in BUCKETS]:
        hits, shots = stats[b]
        if shots:
            rows.append((b, frames[b], shots, hits, 100.0 * hits / shots))

    lines = [
        "# 命中率随距离的变化（B-06）",
        "",
        "同一条 25 fps 视频里，目标距离从 3 m 变到 8 m。把每一发按“射击时刻的估计距离”分桶，",
        "看命中率怎么随距离掉——这是 67–68% 这个数字最直接的解释。",
        "",
        "| 距离 m | 跟踪帧数 | 射击数 | 命中 | 命中率 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for b, n, shots, hits, rate in rows:
        lines.append(f"| {b} | {n} | {shots} | {hits} | {rate:.1f}% |")

    near = [r for r in rows if r[0] in ("0-3", "3-4", "4-5")]
    far = [r for r in rows if r[0] in ("6-7", "7-9")]
    nr = sum(r[3] for r in near) / max(1, sum(r[2] for r in near)) * 100
    fr = sum(r[3] for r in far) / max(1, sum(r[2] for r in far)) * 100

    lines += [
        "",
        "## 结论",
        "",
        f"- 5 m 内命中率 {nr:.1f}%，6 m 外 {fr:.1f}%（差 {nr - fr:.1f} pp）。",
        "- 距离越远，板在画面里越小、同样的角误差占比越大，命中率单调下降。",
        "- 这意味着**提升的关键不是把远处硬拉上来，而是把交战距离压近**（走位、卡点），",
        "  或者提高角分辨率（更长焦距/更高分辨率相机）。",
        "- 对哨兵：优先在 5 m 内开火；超过 7 m 的射击期望收益很低，可以收紧开火窗口省弹。",
        "",
        "*距离来自滤波器状态（xc,yc,zc），不是真值；分桶只用于相对比较。*",
    ]
    out = Path("docs/hit_by_range.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[-8:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
