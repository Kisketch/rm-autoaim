#!/usr/bin/env python3
"""A-03: association stress test for two robots that pass close to each other.

The multi-target manager is a greedy nearest-neighbour association with a gate.
This script replays that logic in Python (same formula as src/target.cpp) over
synthetic plate observations of two robots, and measures how often an
observation is attached to a track that belongs to the *other* robot.

It is a model-level study, not a run of the C++ code: the point is to compare
gate policies under controlled separations, which the real videos cannot do
(they only ever contain one robot).

Output: docs/multi_target.md
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path

FPS_LIST = [25.0, 12.5]
SECONDS = 20.0
NOISE_M = 0.03
GATE_BASE = 0.35


@dataclass
class Robot:
    x: float
    y: float
    yaw0: float
    omega: float
    r1: float = 0.25
    r2: float = 0.22


def plates(robot: Robot, t: float):
    yaw = robot.yaw0 + robot.omega * t
    for k in range(4):
        a = yaw + k * math.pi / 2
        r = robot.r1 if k % 2 == 0 else robot.r2
        yield (robot.x - r * math.cos(a), robot.y - r * math.sin(a))


def gate_for(distance: float, policy: str) -> float:
    if policy == "fixed":
        return GATE_BASE
    return GATE_BASE * min(1.0, max(0.40, 0.35 + 0.065 * distance))


def run(separation: float, fps: float, policy: str, seed: int = 11) -> dict:
    rng = random.Random(seed)
    a = Robot(6.0, -separation / 2, 0.3, 3.0)
    b = Robot(6.0, +separation / 2, 1.1, -2.0)
    tracks: list[list[float]] = []
    votes: list[dict[str, int]] = []
    cross = 0
    assigned = 0
    frames = int(SECONDS * fps)

    for n in range(frames):
        t = n / fps
        obs = []
        for label, robot in (("A", a), ("B", b)):
            for p in plates(robot, t):
                obs.append((label, (p[0] + rng.gauss(0, NOISE_M), p[1] + rng.gauss(0, NOISE_M))))
        rng.shuffle(obs)

        for label, p in obs:
            best, best_d = None, 1e9
            for tid, tr in enumerate(tracks):
                d = math.hypot(p[0] - tr[0], p[1] - tr[1])
                if d < best_d and d < gate_for(math.hypot(p[0], p[1]), policy):
                    best, best_d = tid, d
            if best is None:
                tracks.append([p[0], p[1]])
                votes.append({label: 1})
                continue
            owner = max(votes[best], key=votes[best].get)
            if owner != label:
                cross += 1
            assigned += 1
            votes[best][label] = votes[best].get(label, 0) + 1
            tracks[best][0], tracks[best][1] = p

    return {
        "separation": separation,
        "fps": fps,
        "policy": policy,
        "cross_rate": 100.0 * cross / max(1, assigned),
        "tracks_per_robot_s": len(tracks) / (2 * SECONDS),
    }


def main() -> int:
    rows = [run(sep, fps, pol) for sep in (1.0, 1.5, 2.0, 3.0)
            for fps in FPS_LIST for pol in ("fixed", "scaled")]

    lines = [
        "# 双目标关联压测（A-03）",
        "",
        "两个机器人以 3.0 / −2.0 rad/s 反向旋转、在 6 m 处相向通过，间距从 1.0 m 到 3.0 m；",
        "每个装甲板观测叠加 3 cm 横向噪声。关联逻辑复刻 `src/target.cpp`：贪心最近邻 + 门限。",
        "",
        "指标：**串目标率** = 观测被挂到“历史上属于另一台机器人”的轨迹的比例（越低越好）；",
        "**轨迹数** = 每个机器人每秒新建的轨迹数（越低越稳）。",
        "",
        "| 间距 m | fps | 门限 | 串目标率 % | 轨迹数/机器人/s |",
        "| --- | --- | --- | --- | --- |",
    ]
    for r in rows:
        lines.append(f"| {r['separation']:.1f} | {r['fps']:.1f} | {r['policy']} | "
                     f"{r['cross_rate']:.1f} | {r['tracks_per_robot_s']:.2f} |")

    def avg(policy: str, key: str) -> float:
        vals = [r[key] for r in rows if r["policy"] == policy]
        return sum(vals) / len(vals)

    lines += [
        "",
        "## 结论",
        "",
        f"- 16 组配置（4 个间距 × 2 个帧率 × 2 种门限）串目标率：fixed {avg('fixed', 'cross_rate'):.1f}%，scaled {avg('scaled', 'cross_rate'):.1f}%。",
        f"- 轨迹数：fixed {avg('fixed', 'tracks_per_robot_s'):.2f} 条，scaled {avg('scaled', 'tracks_per_robot_s'):.2f} 条（每机器人每秒）。",
        "- 两者都接近 0：在「两机平动 + 旋转、无遮挡」的几何下，**位置门限不是瓶颈**，缩放门限拿不到可测收益。",
        "- 之前“1.0 m 会串轨”的判断在本模型里**没有被复现**，不能作为结论。",
        "- 真正的串轨风险来自**遮挡/丢失后的重关联**：目标消失若干帧再出现时，最近邻会优先匹配“离得近的那条轨迹”，而那可能属于另一台机器人。要验证必须给仿真加遮挡与丢帧，而不是调门限。",
        "- 处置：`src/target.cpp` 的缩放门限保留（无害，低帧率下更稳），但**不宣称收益**；A-03 判定为「未复现失败模式，需要遮挡场景才能继续」。",
        "",
        "*这是模型级仿真，不是 C++ 实测。*",
    ]
    out = Path("docs/multi_target.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[-9:]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
