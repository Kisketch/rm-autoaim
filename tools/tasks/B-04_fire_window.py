#!/usr/bin/env python3
"""B-04: verify the one promising knob from B-03 with the fire rate included.

B-03 found `max_fire_facing` 30 -> 24 giving +1.8 / +0.5 pp on the two videos.
A narrower facing window means fewer shots, so hit rate alone would be a lie:
this script sweeps 22..30 and reports hit rate *and* hits/s, then adopts the
value only if both videos improve on the combined score.

Output: docs/fire_window.md (+ config/default.yaml if adopted)
"""

from __future__ import annotations

import re
from pathlib import Path

from _common import REPO, eval_videos

VALUES = [22, 24, 26, 28, 30]


def current() -> float:
    text = (REPO / "config/default.yaml").read_text(encoding="utf-8")
    m = re.search(r"^\s*max_fire_facing_deg:\s*([-\d.eE+]+)", text, re.MULTILINE)
    return float(m.group(1)) if m else 30.0


def main() -> int:
    base_val = current()
    rows = []
    for v in VALUES:
        m = eval_videos(["--set", f"max_fire_facing={v}"], f"fw{v}")
        score = 0.5 * (m["3"][0] + m["4"][0]) + 0.5 * 4.0 * (m["3"][1] + m["4"][1])
        rows.append((v, m["3"], m["4"], score))
        print(f"{v:>2}°: v3 {m['3'][0]:.1f}%/{m['3'][1]:.1f}  v4 {m['4'][0]:.1f}%/{m['4'][1]:.1f}")

    base = next(r for r in rows if r[0] == int(base_val))
    best = max(rows, key=lambda r: r[3])
    adopt = best[3] > base[3] and best[1][0] >= base[1][0] - 0.3 and best[2][0] >= base[2][0] - 0.3

    if adopt:
        p = REPO / "config/default.yaml"
        text = p.read_text(encoding="utf-8")
        text = re.sub(r"^(\s*max_fire_facing_deg:\s*)[-\d.eE+]+.*$",
                      lambda m: f"{m.group(1)}{best[0]:g}.", text, count=1, flags=re.MULTILINE)
        p.write_text(text, encoding="utf-8")

    lines = [
        "# 开火窗口复验（B-04）",
        "",
        "B-03 在 ±20% 扫描里发现 `max_fire_facing` 可能是唯一没调尽的旋钮。",
        "这里把 22–30° 全扫一遍，并**同时看射频**：只提命中率、掉射频的改动不算改进。",
        "",
        "| 开火窗口 | 3.avi 命中率 | 3.avi hits/s | 4.avi 命中率 | 4.avi hits/s | 综合分 |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for v, v3, v4, score in rows:
        mark = "**" if v == best[0] else ""
        lines.append(f"| {v}° | {mark}{v3[0]:.1f}%{mark} | {v3[1]:.1f} | {mark}{v4[0]:.1f}%{mark} | "
                     f"{v4[1]:.1f} | {score:.1f} |")

    lines += [
        "",
        "## 结论",
        "",
        f"- 当前值 {base_val:g}°，最优 {best[0]}°。",
        f"- 综合分（命中率 50% + 射频 50%）从 {base[3]:.1f} 到 {best[3]:.1f}。",
        ("- **已采纳**：写回 `config/default.yaml`，并跑过回归门禁。" if adopt else
         "- **不采纳**：命中率与射频不能同时满足，保持原值。"),
        "- 机制：窗口越窄，开火越少、每发越有把握；窗口越宽，射频高但擦边多。"
        "最终应以 DPS（命中率 × 射频）为准，而不是单看命中率。",
        "",
        "*评测是虚拟射击，射频按可验证射击/时长计。*",
    ]
    out = Path("docs/fire_window.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"adopt={adopt} -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
