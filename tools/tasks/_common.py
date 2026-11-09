"""Shared helpers for the auto-improve task scripts."""

from __future__ import annotations

import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def exe(name: str) -> Path:
    for rel in ("build/Release", "build"):
        p = REPO / rel / f"{name}.exe"
        if p.is_file():
            return p
    raise SystemExit(f"找不到 {name}，先构建")


def video_dir() -> Path:
    for cand in (REPO / "video", REPO.parent / "template/template/video",
                 REPO.parent.parent / "template/template/video",
                 Path("/home/lgnatius/video")):
        if (cand / "3.avi").is_file():
            return cand
    raise SystemExit("找不到 3.avi / 4.avi")


def run(cmd: list[str], timeout: int = 900) -> str:
    proc = subprocess.run([str(c) for c in cmd], cwd=str(REPO), capture_output=True,
                          text=True, timeout=timeout)
    if proc.returncode != 0:
        raise SystemExit(f"{' '.join(map(str, cmd))} -> {proc.returncode}\n{proc.stderr[-800:]}")
    return proc.stdout


def video_metrics(text: str) -> tuple[float, float]:
    for line in text.splitlines():
        if line.startswith("| robot EKF (whole-robot model)"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            return float(cells[4].rstrip("%")), float(cells[5])
    raise SystemExit("输出里没有 robot EKF 行")


def sim_mean(text: str) -> float:
    vals = []
    for line in text.splitlines():
        if line.startswith("|") and "---" not in line and not line.startswith("| scenario"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if len(cells) >= 4:
                try:
                    vals.append(float(cells[3]))
                except ValueError:
                    pass
    return sum(vals) / len(vals) if vals else 0.0


def eval_videos(sets: list[str], tag: str) -> dict[str, tuple[float, float]]:
    vdir = video_dir()
    out = {}
    for name in ("3", "4"):
        text = run([exe("autoaim_video"), vdir / f"{name}.avi", "--out",
                    REPO / "runs" / f"task_{tag}_{name}", *sets])
        out[name] = video_metrics(text)
    return out


# Canonical objective used by every sweep from F-01 on:
#   DPS = hits per second x damage per hit (17 mm = 10 HP)
# hits/s already folds hit rate and fire rate together, so ranking by DPS is
# equivalent to ranking by damage output. Hit rate is kept only as a tie-break
# (it measures ammo efficiency, not damage).
DAMAGE_PER_HIT = 10


def dps(hits_per_sec: float) -> float:
    return hits_per_sec * DAMAGE_PER_HIT
