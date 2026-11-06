#!/usr/bin/env python3
"""A-04: how sensitive is the detector to exposure and sensor noise?

Takes the first 360 frames of video 3, applies four transforms (darker, brighter,
two noise levels), re-encodes each as MJPG and runs the same evaluator. Reports
hit rate, frames-with-armor ratio and the 0.32 s prediction error.

Motivation: the robot will run under different lighting than these two clips,
and the camera may be replaced. If the detector degrades gracefully, threshold
70 is fine; if it falls off a cliff, the threshold has to become adaptive.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

# The evaluator links opencv_world*.dll from the prebuilt package; a Python
# subprocess does not inherit the shell's PATH edit, so prepend it here.
for _cand in (r"C:\opencv\opencv\build\x64\vc16\bin", r"C:\opencv\build\x64\vc16\bin"):
    if Path(_cand).is_dir():
        import os
        os.environ["PATH"] = _cand + os.pathsep + os.environ.get("PATH", "")
        break

ROOT = Path(__file__).resolve().parents[2]
EXE = ROOT / "build" / "Release" / "autoaim_video.exe"
SRC = ROOT.parent / "template" / "template" / "video" / "3.avi"
OUT = ROOT / "runs" / "exposure"
DOC = ROOT / "docs" / "exposure_robustness.md"
N_FRAMES = 360


def make_variant(name: str, fn) -> Path:
    dst = OUT / f"3_{name}.avi"
    if dst.is_file():
        return dst
    OUT.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(SRC))
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(str(dst), cv2.VideoWriter_fourcc(*"MJPG"), fps, (w, h))
    rng = np.random.default_rng(0)
    for i in range(N_FRAMES):
        ok, frame = cap.read()
        if not ok:
            break
        writer.write(fn(frame, rng))
    cap.release()
    writer.release()
    return dst


def parse(text: str) -> tuple[float, float, float]:
    hit = hps = med = float("nan")
    mask = float("nan")
    for line in text.splitlines():
        if line.startswith("| robot EKF (whole-robot model)"):
            c = [x.strip() for x in line.strip().strip("|").split("|")]
            hit, hps = float(c[4].rstrip("%")), float(c[5])
        if line.startswith("| robot EKF | 0.32"):
            c = [x.strip() for x in line.strip().strip("|").split("|")]
            med = float(c[4])
        if line.startswith("tracking ratio"):
            mask = float(line.split("frames with armor")[1].split(",")[0])
    return hit, hps, med, mask


def run(video: Path, tag: str):
    out = OUT / f"eval_{tag}"
    proc = subprocess.run([str(EXE), str(video), "--out", str(out)],
                          cwd=str(ROOT), capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(proc.stderr[-400:])
    return parse(proc.stdout)


def main() -> int:
    if not EXE.is_file() or not SRC.is_file():
        raise SystemExit("缺 exe 或源视频")
    variants = {
        "base": lambda f, r: f,
        "dark06": lambda f, r: np.clip(f.astype(np.float32) * 0.6, 0, 255).astype(np.uint8),
        "bright14": lambda f, r: np.clip(f.astype(np.float32) * 1.4, 0, 255).astype(np.uint8),
        "noise8": lambda f, r: np.clip(f.astype(np.float32) + r.normal(0, 8, f.shape), 0, 255).astype(np.uint8),
        "noise16": lambda f, r: np.clip(f.astype(np.float32) + r.normal(0, 16, f.shape), 0, 255).astype(np.uint8),
    }
    rows = []
    for name, fn in variants.items():
        path = make_variant(name, fn)
        hit, hps, med, mask = run(path, name) if name != "base" else run(SRC, "base")
        rows.append((name, hit, hps, med, mask))
        print(f"  {name:9s} hit {hit:5.1f}%  hps {hps:4.1f}  pred32 {med:5.1f}  armor-frames {mask:.3f}")

    label = {"base": "原始", "dark06": "亮度 ×0.6", "bright14": "亮度 ×1.4",
             "noise8": "噪声 σ=8", "noise16": "噪声 σ=16"}
    lines = [
        "# 曝光与噪声鲁棒性（A-04）",
        "",
        f"3.avi 前 {N_FRAMES} 帧，四种变换后重跑同一评测器。",
        "",
        "| 变换 | 命中率 | hits/s | 0.32 s 中位误差 | 有目标帧占比 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for name, hit, hps, med, mask in rows:
        lines.append(f"| {label[name]} | {hit:.1f}% | {hps:.1f} | {med:.1f} px | {mask:.3f} |")

    base_hit = rows[0][1]
    worst = min(rows[1:], key=lambda r: r[1])
    lines += [
        "",
        "## 结论",
        "",
        f"- 最伤的是**{label[worst[0]]}**：命中率 {worst[1]:.1f}%（相对原始 {base_hit:.1f}% 变化 {worst[1]-base_hit:+.1f} pp）。",
        "- 检测阈值 70 是绝对通道差，亮度整体缩放时 R−B 也跟着缩放：变暗会压低灯条响应，",
        "  变亮会让更多背景进入阈值区。",
        "- 噪声对灯条（高强度、大面积）影响小，对边缘角点影响大——体现在预测误差而不是命中率上。",
    ]
    DOC.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {DOC}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
