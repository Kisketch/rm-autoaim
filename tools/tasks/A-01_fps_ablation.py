#!/usr/bin/env python3
"""A-01: frame-rate ablation.

Re-encodes the two 25 fps test videos at 12.5 and 6.25 fps (drop every other /
every fourth frame, keep playback speed), runs the same evaluator, and writes
docs/fps_ablation.md with hit rate, fire rate and the 0.32 s prediction error.

This is the other end of the "what would 100 fps buy us" curve: if halving the
frame rate costs X pp, then the camera upgrade is worth roughly the same order
of magnitude - not a doubling.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXE = ROOT / "build" / "Release" / "autoaim_video.exe"
VIDEOS = ROOT.parent / "template" / "template" / "video"
OUT = ROOT / "runs" / "fps_ablation"
DOC = ROOT / "docs" / "fps_ablation.md"

FFMPEG_CANDIDATES = [
    r"C:\Users\Lgnatius\AppData\Local\Programs\yt-dlp\ffmpeg.exe",
    "ffmpeg",
]


def ffmpeg() -> str:
    for c in FFMPEG_CANDIDATES:
        if c == "ffmpeg":
            if shutil.which("ffmpeg"):
                return "ffmpeg"
        elif Path(c).is_file():
            return c
    raise SystemExit("找不到 ffmpeg")


def decimate(src: Path, dst: Path, step: int, fps: float) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    vf = f"select='not(mod(n\\,{step}))',setpts={1/step:.6f}*PTS"
    cmd = [ffmpeg(), "-y", "-i", str(src), "-vf", vf, "-r", str(fps),
           "-c:v", "mjpeg", "-q:v", "3", "-an", str(dst)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"ffmpeg 失败: {proc.stderr[-400:]}")


def parse(text: str) -> tuple[float, float, float]:
    hit = hps = 0.0
    med = float("nan")
    for line in text.splitlines():
        if line.startswith("| robot EKF (whole-robot model)"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            hit, hps = float(cells[4].rstrip("%")), float(cells[5])
        if line.startswith("| robot EKF | 0.32"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            med = float(cells[4])
    return hit, hps, med


def evaluate(video: Path, tag: str) -> tuple[float, float, float]:
    out = OUT / tag
    env = None
    proc = subprocess.run([str(EXE), str(video), "--out", str(out)],
                          cwd=str(ROOT), capture_output=True, text=True, env=env)
    if proc.returncode != 0:
        raise SystemExit(f"评测失败 {video}: {proc.stderr[-500:]}")
    return parse(proc.stdout)


def main() -> int:
    if not EXE.is_file():
        raise SystemExit(f"找不到 {EXE}")
    rows = []
    for stem in ("3", "4"):
        src = VIDEOS / f"{stem}.avi"
        if not src.is_file():
            raise SystemExit(f"找不到 {src}")
        rows.append((stem, 25.0, evaluate(src, f"{stem}_fps25")))
        for fps, step in ((12.5, 2), (6.25, 4)):
            dst = OUT / f"{stem}_fps{fps:g}.avi"
            if not dst.is_file():
                decimate(src, dst, step, fps)
            rows.append((stem, fps, evaluate(dst, f"{stem}_fps{fps:g}")))
            print(f"  {stem}.avi @ {fps:g} fps -> {rows[-1][2]}")

    lines = [
        "# 帧率消融（A-01）",
        "",
        "把 25 fps 的两条测试视频降采样到 12.5 / 6.25 fps（丢弃隔帧，播放速度不变），",
        "用**同一个**评测器与同一份参数重跑。这是「升到 100 fps 能拿多少」曲线的另一端。",
        "",
        "| 视频 | fps | 命中率 | hits/s | 0.32 s 中位预测误差 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for stem, fps, (hit, hps, med) in rows:
        lines.append(f"| {stem}.avi | {fps:g} | {hit:.1f}% | {hps:.1f} | {med:.1f} px |")

    # 结论：每翻倍一档的代价
    lines += ["", "## 结论", ""]
    for stem in ("3", "4"):
        sub = [(f, h) for s, f, (h, _, _) in rows if s == stem]
        sub.sort()
        d1 = sub[1][1] - sub[0][1]
        d2 = sub[2][1] - sub[1][1]
        lines.append(f"- {stem}.avi：25→12.5 fps **{d1:+.1f} pp**，12.5→6.25 fps **{d2:+.1f} pp**")
    lines += [
        "",
        "读法：帧率减半的损失，与帧率翻倍的收益大致对称（误差 ∝ 1/√N，但真正的大头是",
        "0.29 s 弹丸飞行时间的外推，帧率改变不了它）。若降采样损失很小，说明当前瓶颈不在采样率，",
        "而在运动模型与检测精度——这与仿真 `--fps` 实验的结论一致（25→100 fps 均值仅 +2 pp）。",
    ]
    DOC.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {DOC}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
