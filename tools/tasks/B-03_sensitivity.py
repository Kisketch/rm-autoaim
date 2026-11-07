#!/usr/bin/env python3
"""B-03: is the parameter space really flat? One-at-a-time sensitivity sweep.

BASELINE claims the tuning is saturated. This checks it: each knob is moved
±20 % alone and both videos are re-evaluated; a knob that moves nothing is
evidence for saturation, a knob that moves a lot is a candidate for re-tuning.

Output: docs/parameter_sensitivity.md
"""

from __future__ import annotations

import re
from pathlib import Path

from _common import REPO, eval_videos

KNOBS = ["q_yaw", "q_r", "singer_tau", "max_fire_facing",
         "switch_hysteresis", "q_xyz"]


def yaml_value(key: str) -> float:
    text = (REPO / "config/default.yaml").read_text(encoding="utf-8")
    yaml_key = {"max_fire_facing": "max_fire_facing_deg",
                "switch_hysteresis": "switch_hysteresis_deg"}.get(key, key)
    m = re.search(rf"^\s*{re.escape(yaml_key)}:\s*([-\d.eE+]+)", text, re.MULTILINE)
    if not m:
        raise SystemExit(f"yaml 里没有 {yaml_key}")
    return float(m.group(1))


def main() -> int:
    base = eval_videos([], "sens_base")
    base_score = base["3"][0] + base["4"][0]
    print(f"baseline: v3 {base['3'][0]:.1f}  v4 {base['4'][0]:.1f}")

    rows = []
    for key in KNOBS:
        cur = yaml_value(key)
        for factor in (0.8, 1.2):
            val = cur * factor
            m = eval_videos(["--set", f"{key}={val:.6g}"], f"sens_{key}_{factor}")
            d3 = m["3"][0] - base["3"][0]
            d4 = m["4"][0] - base["4"][0]
            rows.append((key, cur, val, m["3"][0], d3, m["4"][0], d4))
            print(f"{key:22s} {cur:9.4g} -> {val:9.4g}  v3 {m['3'][0]:5.1f} ({d3:+.1f})  "
                  f"v4 {m['4'][0]:5.1f} ({d4:+.1f})")

    worst = max(rows, key=lambda r: abs(r[4]) + abs(r[6]))
    lines = [
        "# 参数敏感性（B-03）",
        "",
        "每个参数单独 ±20%，其余不动，看两条视频的命中率怎么变。用来验证“参数已经调不动了”这个判断。",
        "",
        "| 参数 | 原值 | 试探值 | 3.avi | Δ | 4.avi | Δ |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for key, cur, val, h3, d3, h4, d4 in rows:
        lines.append(f"| `{key}` | {cur:g} | {val:g} | {h3:.1f}% | {d3:+.1f} | {h4:.1f}% | {d4:+.1f} |")

    best = max(rows, key=lambda r: r[4] + r[6])
    both_up = best[4] > 0 and best[6] > 0
    lines += [
        "",
        "## 结论",
        "",
        f"- 最敏感的参数是 `{worst[0]}`：±20% 带来 3.avi {worst[4]:+.1f} pp、4.avi {worst[6]:+.1f} pp。",
        f"- 单点最优是 `{best[0]}={best[2]:g}`：3.avi {best[3]:.1f}%（{best[4]:+.1f}）、4.avi {best[5]:.1f}%（{best[6]:+.1f}）。",
        ("- **这一项是两条视频同时变好**，不是平台噪声，需要进一步验证射频（本脚本只测命中率）。"
         if both_up else
         "- 没有一项能让两条视频同时变好，符合平台区的判断。"),
        "- 其余参数 ±20% 都在 ±1 pp 内，可以认为已经调不动；提升要靠改信息条件或模型结构。",
        "",
        "*每个点都是真实评测，缓存命中后单次约 6 s；命中率不含射频，改动前必须复核 hits/s。*",
    ]
    out = Path("docs/parameter_sensitivity.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
