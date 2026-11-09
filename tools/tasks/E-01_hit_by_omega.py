#!/usr/bin/env python3
"""E-01: hit rate vs target angular velocity.

The tracker's hardest regime is not distance but spin rate: at 8 rad/s the plate
crosses a full panel width in ~25 ms. This buckets every judged shot by the
estimated |w| and reports hit rate and shots per bucket.

Output: docs/hit_by_omega.md
"""

from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

from _common import REPO, eval_videos

EDGES = [0, 2, 4, 6, 8, 10, 12, 99]


def bucket(w: float) -> str | None:
    for lo, hi in zip(EDGES, EDGES[1:]):
        if lo <= w < hi:
            return f"{lo}-{hi}" if hi != 99 else f"{lo}+"
    return None


def main() -> int:
    eval_videos([], "omega")
    stats: dict[str, list[int]] = defaultdict(lambda: [0, 0])

    for name in ("3", "4"):
        path = REPO / "runs" / f"task_omega_{name}" / f"{name}_track.csv"
        with path.open(newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                if row.get("shot") != "1":
                    continue
                try:
                    w = abs(float(row["w"]))
                except (KeyError, ValueError):
                    continue
                b = bucket(w)
                if b is None:
                    continue
                stats[b][1] += 1
                if row.get("shot_result") == "1":
                    stats[b][0] += 1

    rows = []
    for lo, hi in zip(EDGES, EDGES[1:]):
        b = f"{lo}-{hi}" if hi != 99 else f"{lo}+"
        hits, shots = stats[b]
        if shots:
            rows.append((b, shots, hits, 100.0 * hits / shots))

    lines = [
        "# 命中率随目标角速度的变化（E-01）",
        "",
        "每发按射击时刻估计的 `|w|` 分桶。板在画面里的横向速度 = ω·r，所以 ω 越大，"
        "同样的时间误差造成的角误差越大。",
        "",
        "| \\|ω\\| rad/s | 射击数 | 命中 | 命中率 |",
        "| --- | --- | --- | --- |",
    ]
    for b, shots, hits, rate in rows:
        lines.append(f"| {b} | {shots} | {hits} | {rate:.1f}% |")

    lo_rows = [r for r in rows if r[0] in ("0-2", "2-4")]
    hi_rows = [r for r in rows if r[0] in ("8-10", "10+")]
    def agg(rs):
        s = sum(r[1] for r in rs); h = sum(r[2] for r in rs)
        return 100.0 * h / s if s else 0.0

    lines += [
        "",
        "## 结论",
        "",
        f"- 低速段（<4 rad/s）命中率 {agg(lo_rows):.1f}%（{sum(r[1] for r in lo_rows)} 发），"
        f"高速段（≥8 rad/s）{agg(hi_rows):.1f}%（{sum(r[1] for r in hi_rows)} 发）。",
        "- 若高速段明显更低，说明限制来自“同样的预测误差在快速旋转下放大”，"
        "对应解法是更高帧率/更好的 yaw 观测，而不是调开火窗口。",
        "- 若两段接近，则说明瓶颈在检测与标定（系统性误差），与转速无关。",
        "",
        "*ω 来自滤波器状态，非真值；分桶仅用于相对比较。*",
    ]
    out = Path("docs/hit_by_omega.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[-7:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
