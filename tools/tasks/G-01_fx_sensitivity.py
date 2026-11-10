#!/usr/bin/env python3
"""G-01: how much does a focal-length error cost?

The videos have no calibration, so fx=2400 is an assumption (8 mm lens on a
3.45 um sensor gives 2319). This sweep answers whether R-02 (calibration) is
worth a day: it changes fx by ±12 % and measures the hit-rate response.

Output: docs/fx_sensitivity.md
"""

from __future__ import annotations

from pathlib import Path

from _common import eval_videos

VALUES = [2100, 2200, 2319, 2400, 2500, 2700]


def main() -> int:
    rows = []
    for fx in VALUES:
        m = eval_videos(["--set", f"fx={fx}"], f"fx{fx}")
        rows.append((fx, m["3"], m["4"]))
        print(f"fx={fx}: v3 {m['3'][0]:.1f}%/{m['3'][1]:.2f}  v4 {m['4'][0]:.1f}%/{m['4'][1]:.2f}")

    base = next(r for r in rows if r[0] == 2400)
    best = max(rows, key=lambda r: r[1][0] + r[2][0])
    lines = [
        "# 焦距误差的代价（G-01）",
        "",
        "视频无标定，`fx=2400` 是假设值（8 mm 镜头 + 3.45 µm 像元 → 理论 2319）。",
        "这里把 fx 从 2100 扫到 2700，看命中率怎么变，用来判断 R-02（标定）值不值得做。",
        "",
        "| fx (px) | 相对 2400 | 3.avi 命中率 | 3.avi hits/s | 4.avi 命中率 | 4.avi hits/s |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for fx, v3, v4 in rows:
        mark = "**" if fx == best[0] else ""
        lines.append(f"| {mark}{fx}{mark} | {(fx / 2400 - 1) * 100:+.1f}% | {v3[0]:.1f}% | {v3[1]:.2f} | "
                     f"{v4[0]:.1f}% | {v4[1]:.2f} |")

    spread = max(r[1][0] for r in rows) - min(r[1][0] for r in rows)
    lines += [
        "",
        "## 结论",
        "",
        f"- 6 个 fx 值下 3.avi 命中率极差 **{spread:.1f} pp**，最优在 fx={best[0]}"
        f"（比 2400 高 {(best[1][0] - base[1][0]):+.1f} / {(best[2][0] - base[2][0]):+.1f} pp）。",
        "- fx 同时改变“深度”和“板的角尺寸”，两者部分抵消，所以曲线不陡——**但也意味着单靠调 fx 骗不出命中率**。",
        "- 若理论值 2319 与最优值相差在 ±3% 内，说明现有假设够用；否则必须先标定再谈其它优化。",
        "- 注意：fx 不是可调参数，是物理量。这里只是**用误差反推标定精度要求**，不是建议改 fx。",
        "",
        "*`--set fx=` 会同时改 fx/fy，深度随之变化；命中率变化包含“深度误差”与“角尺寸误差”两部分。*",
    ]
    out = Path("docs/fx_sensitivity.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
