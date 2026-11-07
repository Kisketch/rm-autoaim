#!/usr/bin/env python3
"""C-01: fire window x distance, two-dimensional.

B-04 showed 30 deg maximises DPS overall; B-06 showed 7-9 m is the weakest band.
The obvious question is whether the window should depend on distance: tight when
far (fewer, better shots), wide when close.

This runs the three windows and buckets the judged shots by range.

Output: docs/window_by_range.md
"""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path

from _common import REPO, eval_videos

WINDOWS = [22, 26, 30]
BUCKETS = [(4, 6), (6, 7), (7, 9)]


def bucket(d: float) -> str | None:
    for lo, hi in BUCKETS:
        if lo <= d < hi:
            return f"{lo}-{hi}"
    return None


def collect(tag: str) -> dict[str, list[int]]:
    stats: dict[str, list[int]] = defaultdict(lambda: [0, 0])
    for name in ("3", "4"):
        path = REPO / "runs" / f"task_{tag}_{name}" / f"{name}_track.csv"
        if not path.is_file():
            continue
        with path.open(newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                if row.get("shot") != "1":
                    continue
                try:
                    d = math.sqrt(float(row["xc"]) ** 2 + float(row["yc"]) ** 2 + float(row["zc"]) ** 2)
                except (KeyError, ValueError):
                    continue
                b = bucket(d)
                if b is None:
                    continue
                stats[b][1] += 1
                if row.get("shot_result") == "1":
                    stats[b][0] += 1
    return stats


def main() -> int:
    tables = {}
    for w in WINDOWS:
        eval_videos(["--set", f"max_fire_facing={w}"], f"dw{w}")
        tables[w] = collect(f"dw{w}")

    lines = [
        "# 开火窗口 × 距离（C-01）",
        "",
        "三个开火窗口各自跑一遍两条视频，把射击按距离分桶，看“远距离要不要收紧窗口”。",
        "",
        "| 距离 m | 窗口 22° | 窗口 26° | 窗口 30° |",
        "| --- | --- | --- | --- |",
    ]
    for lo, hi in BUCKETS:
        b = f"{lo}-{hi}"
        cells = []
        for w in WINDOWS:
            hits, shots = tables[w].get(b, [0, 0])
            cells.append(f"{100.0 * hits / shots:.1f}% ({shots} 发)" if shots else "—")
        lines.append(f"| {b} | " + " | ".join(cells) + " |")

    lines += [
        "",
        "## 结论",
        "",
        "- 同一距离段内，窗口从 30° 收到 22° 命中率会升、发数会掉；**收益是否值得要看 DPS，不是命中率**。",
        "- 若某个距离段在窄窗口下命中率提升明显、发数掉得少，就可以做“按距离切换窗口”；",
        "  否则应保持单一窗口，避免多一套状态机。",
        "- 具体阈值由上面的表直接读出：比较每列的 (命中率 × 发数)，取大的那一列。",
        "",
        "*数据来自虚拟射击；距离是滤波器估计值。*",
    ]
    out = Path("docs/window_by_range.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[-9:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
