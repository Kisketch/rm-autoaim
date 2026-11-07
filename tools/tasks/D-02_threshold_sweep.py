#!/usr/bin/env python3
"""D-02: is the detection threshold limiting the far range?

A-04 showed dimming kills detection. The threshold 70 was tuned on these clips,
but never swept against *range*: if far plates are dimmer, a lower threshold may
recover them at the cost of more false light bars.

Sweeps 50..90 and reports hit rate plus the fraction of frames with at least one
armor ("frames with armor"), which is the detection-recall proxy.

Output: docs/threshold_vs_range.md
"""

from __future__ import annotations

import re
from pathlib import Path

from _common import REPO, eval_videos, run, exe, video_dir

VALUES = [50, 60, 70, 80, 90]


def armor_ratio(text: str) -> float:
    m = re.search(r"frames with armor ([\d.]+)", text)
    return float(m.group(1)) * 100 if m else 0.0


def main() -> int:
    vdir = video_dir()
    rows = []
    for t in VALUES:
        sets = ["--set", f"channel_diff_threshold={t}"]
        vids = eval_videos(sets, f"thr{t}")
        ratios = []
        for name in ("3", "4"):
            text = run([exe("autoaim_video"), vdir / f"{name}.avi", "--out",
                        REPO / "runs" / f"thr{t}_{name}", *sets])
            ratios.append(armor_ratio(text))
        rows.append((t, vids["3"], vids["4"], ratios))
        print(f"thr={t}: v3 {vids['3'][0]:.1f}%  v4 {vids['4'][0]:.1f}%  armor {ratios[0]:.1f}/{ratios[1]:.1f}%")

    base = next(r for r in rows if r[0] == 70)
    lines = [
        "# 通道差阈值 × 距离（D-02）",
        "",
        "阈值越低越能检出暗/远的灯条，但也越容易把琥珀色装饰灯算进来。",
        "`frames with armor` 是“至少检出一块装甲板的帧占比”，作为召回代理指标。",
        "",
        "| 阈值 | 3.avi 命中率 | 4.avi 命中率 | 3.avi 有目标帧 | 4.avi 有目标帧 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for t, v3, v4, (r3, r4) in rows:
        mark = "**" if t == 70 else ""
        lines.append(f"| {mark}{t}{mark} | {v3[0]:.1f}% | {v4[0]:.1f}% | {r3:.1f}% | {r4:.1f}% |")

    best_hit = max(rows, key=lambda r: r[1][0] + r[2][0])
    best_recall = max(rows, key=lambda r: r[3][0] + r[3][1])
    lines += [
        "",
        "## 结论",
        "",
        f"- 命中率最高在阈值 {best_hit[0]}（3.avi {best_hit[1][0]:.1f}%，4.avi {best_hit[2][0]:.1f}%）；"
        f"召回最高在阈值 {best_recall[0]}（{best_recall[3][0]:.1f}% / {best_recall[3][1]:.1f}%）。",
        f"- 当前 70：{base[1][0]:.1f}% / {base[2][0]:.1f}%，召回 {base[3][0]:.1f}% / {base[3][1]:.1f}%。",
        "- 若降低阈值只提高召回、不提高命中率，说明多检出的目标大多是假的或打不准的，不该改。",
        "- 阈值与曝光耦合：换相机/换场地必须重扫，不能沿用 70。",
        "",
        "*命中率来自虚拟射击；召回代理只用“有目标帧占比”，没有人工标注。*",
    ]
    out = Path("docs/threshold_vs_range.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[-8:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
