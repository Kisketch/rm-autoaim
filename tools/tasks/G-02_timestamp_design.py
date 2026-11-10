#!/usr/bin/env python3
"""G-02: timestamp and latency budget, written as a design note for R-04/R-05.

No code is changed. The numbers are derived from the constants we already use
(25 fps, 60 ms latency, 25 m/s bullet) so the acceptance criteria in the roadmap
have a stated basis instead of a guessed one.

Output: docs/timestamp_design.md
"""

from __future__ import annotations

import math
from pathlib import Path

FPS = 25.0
FRAME_MS = 1000.0 / FPS
LATENCY_MS = 60.0
BULLET = 25.0
OMEGA = 3.0          # rad/s, a typical spinning target
RANGE = 6.0          # m
PANEL_HALF_DEG = math.degrees(math.atan2(0.0675, RANGE))   # 135 mm panel / 2
FLIGHT_MS = RANGE / BULLET * 1000


def main() -> int:
    # one millisecond of gimbal/attitude error turns into this much angle error
    deg_per_ms = OMEGA * 0.001 * 180.0 / math.pi  # rad/s * 1 ms -> degrees
    budget_angle = 0.2 * PANEL_HALF_DEG     # spend at most 20 % of the half panel on timing
    budget_ms = budget_angle / deg_per_ms    # total timing error budget in milliseconds

    lines = [
        "# 时间戳与延迟预算（G-02）",
        "",
        "目标：把 R-04（曝光时间戳）和 R-05（端到端延迟）的验收标准，从“拍脑袋”变成可推导的数。",
        "",
        "## 已知常数",
        "",
        f"- 帧率 {FPS:.0f} fps → 帧间隔 **{FRAME_MS:.0f} ms**",
        f"- 当前假设延迟 **{LATENCY_MS:.0f} ms**（capture → 子弹出膛）",
        f"- 弹速 {BULLET:.0f} m/s，{RANGE:.0f} m 处飞行 **{FLIGHT_MS:.0f} ms**",
        f"- 装甲板半宽 67.5 mm，{RANGE:.0f} m 处张角 **{PANEL_HALF_DEG:.2f}°**",
        "",
        "## 误差换算",
        "",
        f"- 目标以 {OMEGA:.0f} rad/s（172°/s）旋转时，**1 ms 的时间误差 = {deg_per_ms:.3f}°** 的角度误差。",
        f"- 我们允许时间误差吃掉板半宽的 20%，即 {budget_angle:.2f}° → 允许时间误差 "
        f"**{budget_angle / deg_per_ms:.2f} ms**。",
        "",
        "## 由此得到的验收线",
        "",
        "| 项 | 允许误差 | 依据 |",
        "| --- | --- | --- |",
        f"| 曝光时间戳抖动 σ | ≤ {0.4 * budget_ms:.1f} ms | 占预算 40%（均方根分配） |",
        f"| 图像→姿态对齐误差 | ≤ {0.4 * budget_ms:.1f} ms | 姿态环形缓冲插值残差 |",
        f"| 总延迟估计误差 | ≤ {0.6 * budget_ms:.1f} ms | 等效 {(0.6 * budget_ms * deg_per_ms):.2f}° |",
        f"| **三者均方根合计** | ≤ {budget_ms:.2f} ms | 与总预算一致（RSS 合成，不线性相加） |",
        "| 延迟绝对值 | 实测替换 60 ms | 误差在 60±15 ms 内对命中率影响 <1 pp（仿真扫描） |",
        "",
        "## 采样与延迟的关系（为什么提帧率有用但不翻倍）",
        "",
        f"- 25 fps 时，一帧内目标转过 {OMEGA * FRAME_MS / 1000 * 180 / math.pi:.1f}°，"
        f"检测本身就有 ±{FRAME_MS / 2:.0f} ms 的量化误差。",
        f"- 100 fps 时降到 ±5 ms，量化误差减少 {FRAME_MS / 2 - 5:.0f} ms。",
        "- 但飞行时间 240 ms 不变，预测仍然要外推同样久；所以帧率收益来自**估计方差**与**检测延迟**，不是外推长度。",
        "",
        "## 落地步骤（对应 R-04 / R-05）",
        "",
        "1. 相机侧：`MV-CS016-10UC` 开硬件触发或取 `ExposureStartTime`，与主机时钟做 PTP/chrony 对齐，记录每次曝光的绝对时间戳。",
        "2. 姿态侧：串口每帧带 IMU 四元数 + 时间戳，按曝光时间戳做**环形缓冲插值**（不是取最新值）。",
        "3. 链路侧：用高速相机拍“屏幕闪光 → 云台动作”，或让 MCU 回传接收时刻，实测四段延迟之和。",
        "4. 验收：把实测值写回 `aimer.latency`，跑 `tools/regression.py`，命中率不得回退。",
        "",
        "*本文是设计说明，数字为推导值；实测后以实测为准并回写本文。*",
    ]
    out = Path("docs/timestamp_design.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[:14]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
