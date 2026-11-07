#!/usr/bin/env python3
"""D-01: where do the misses come from? (state and re-acquire timing)

C-02 split the misses into "edge" (38 %) and "far off" (49 %). This asks *when*
the far-off ones happen: right after the tracker re-acquires the target, or in
steady tracking? If they cluster after re-acquisition, the fix is occlusion
handling, not the estimator's steady-state noise.

Output: docs/failure_modes.md
"""

from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

from _common import REPO, eval_videos


def main() -> int:
    eval_videos([], "failmodes")
    stats = defaultdict(lambda: [0, 0])   # bucket -> [hits, shots]

    for name in ("3", "4"):
        path = REPO / "runs" / f"task_failmodes_{name}" / f"{name}_track.csv"
        with path.open(newline="", encoding="utf-8") as fh:
            prev_state = "LOST"
            since_track = 999
            for row in csv.DictReader(fh):
                state = row.get("state", "LOST")
                if state == "TRACKING":
                    since_track = 0 if prev_state != "TRACKING" else since_track + 1
                else:
                    since_track = 999
                prev_state = state

                if row.get("shot") != "1":
                    continue
                if state != "TRACKING":
                    bucket = "非 TRACKING 时开火"
                elif since_track <= 3:
                    bucket = "重捕后 ≤3 帧"
                elif since_track <= 10:
                    bucket = "重捕后 4–10 帧"
                else:
                    bucket = "稳态跟踪"
                stats[bucket][1] += 1
                if row.get("shot_result") == "1":
                    stats[bucket][0] += 1

    order = ["重捕后 ≤3 帧", "重捕后 4–10 帧", "稳态跟踪", "非 TRACKING 时开火"]
    lines = [
        "# 打偏发生在什么时候（D-01）",
        "",
        "按“射击时刻距上次重新捕获有多少帧”分层。如果脱靶集中在重捕后几帧，",
        "说明问题在遮挡/丢失后的恢复，而不是稳态估计精度。",
        "",
        "| 状态 | 射击数 | 命中 | 命中率 |",
        "| --- | --- | --- | --- |",
    ]
    for b in order:
        hits, shots = stats.get(b, [0, 0])
        if shots:
            lines.append(f"| {b} | {shots} | {hits} | {100.0 * hits / shots:.1f}% |")

    total = sum(s for _, s in stats.values())
    fresh = stats.get("重捕后 ≤3 帧", [0, 0])
    steady = stats.get("稳态跟踪", [0, 0])

    lines += [
        "",
        "## 结论",
        "",
        f"- 共 {total} 发；其中“重捕后 ≤3 帧”占 {100.0 * fresh[1] / max(1, total):.1f}%，"
        f"“稳态跟踪”占 {100.0 * steady[1] / max(1, total):.1f}%。",
    ]
    if fresh[1] >= 30 and steady[1] >= 30:
        lines.append(
            f"- 重捕后 ≤3 帧命中 {100.0 * fresh[0] / fresh[1]:.1f}%，稳态 {100.0 * steady[0] / steady[1]:.1f}%。"
            + ("→ **恢复期明显更差**，遮挡/丢失后的重捕是主要失分点。"
               if fresh[0] / fresh[1] < steady[0] / steady[1] - 0.05 else
               "→ 两者接近，脱靶不是重捕造成的，问题在稳态预测。")
        )
    lines += [
        "- 若恢复期占比高且命中低，优先做“遮挡期外推 + 放大协方差”，而不是继续调过程噪声。",
        "",
        "*状态取自 `*_track.csv` 的 `state` 列；分层是相对的，没有真值标注。*",
    ]
    out = Path("docs/failure_modes.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[-8:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
