#!/usr/bin/env python3
"""Closed-loop improvement for the autoaim tracker/aimer.

One iteration is: sample a parameter set -> run the two real videos and the
simulator -> score -> keep it only if every gate passes and the score improved
-> optionally write config/default.yaml and commit.

Gates (defaults, overridable) are the same numbers as BASELINE.md section 10:
a candidate may not lose more than 0.8 pp of hit rate on either video, 0.3
hits/s of fire rate, or 3 pp of simulator mean.

Usage:
    python3 tools/auto_improve.py --iterations 8 --candidates 3 --dry-run
    python3 tools/auto_improve.py --iterations 8 --candidates 3 --commit
    python3 tools/auto_improve.py --max-minutes 60 --commit

The loop is deliberately conservative: hill-climb, no restarts, and the score
weights fire rate as much as hit rate, because hit rate alone is trivially
bought by firing less often.
"""

from __future__ import annotations

import argparse
import json
import random
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

# knob -> (low, high, step, log_scale)
SPACE: dict[str, tuple[float, float, float, bool]] = {
    "q_yaw": (1.0, 20.0, 0.5, True),
    "q_r": (5e-4, 1e-2, 5e-4, True),
    "q_xyz": (0.2, 4.0, 0.2, True),
    "r_yaw": (0.003, 0.05, 0.002, True),
    "singer_tau": (0.15, 0.60, 0.03, False),
    "max_fire_facing": (20.0, 45.0, 2.0, False),
    "switch_hysteresis": (0.0, 6.0, 1.0, False),
}

YAML_KEYS = {
    "switch_hysteresis": "switch_hysteresis_deg",
    "max_fire_facing": "max_fire_facing_deg",
}


@dataclass
class Metrics:
    hit: dict[str, float]
    hps: dict[str, float]
    sim: float
    score: float


def run(cmd: list[str], cwd: Path, timeout: int = 600) -> str:
    proc = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True, timeout=timeout)
    if proc.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)} -> {proc.returncode}\n{proc.stderr[-600:]}")
    return proc.stdout


def parse_video(text: str) -> tuple[float, float]:
    for line in text.splitlines():
        if line.startswith("| robot EKF (whole-robot model)"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            return float(cells[4].rstrip("%")), float(cells[5])
    raise RuntimeError("no robot-EKF row")


def parse_sim(text: str) -> float:
    hits = []
    for line in text.splitlines():
        if line.startswith("|") and not line.startswith("| scenario") and "---" not in line:
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if len(cells) >= 4:
                try:
                    hits.append(float(cells[3]))
                except ValueError:
                    pass
    return statistics.mean(hits) if hits else 0.0


def evaluate(repo: Path, exe: Path, sim_exe: Path, videos: dict[str, Path],
             params: dict[str, float], tag: str) -> Metrics:
    sets: list[str] = []
    for k, v in params.items():
        sets += ["--set", f"{k}={v:.6g}"]
    hit, hps = {}, {}
    for name, video in videos.items():
        out = repo / "runs" / f"auto_{tag}_{name}"
        text = run([str(exe), str(video), "--out", str(out), *sets], repo)
        hit[name], hps[name] = parse_video(text)
    sim_text = run([str(sim_exe), "--seconds", "10", "--seed", "0", *sets], repo)
    sim = parse_sim(sim_text)
    score = 0.45 * statistics.mean(hit.values()) + 0.45 * statistics.mean(hps.values()) + 0.10 * sim
    return Metrics(hit, hps, sim, score)


def load_yaml(path: Path) -> dict[str, float]:
    text = path.read_text(encoding="utf-8", errors="replace")
    out: dict[str, float] = {}
    for key in SPACE:
        yaml_key = YAML_KEYS.get(key, key)
        m = re.search(rf"^(\s*){re.escape(yaml_key)}:\s*([-\d.eE+]+)", text, re.MULTILINE)
        if m:
            out[key] = float(m.group(2))
    return out


def write_yaml(path: Path, params: dict[str, float]) -> None:
    text = path.read_text(encoding="utf-8")
    for key, value in params.items():
        yaml_key = YAML_KEYS.get(key, key)
        pattern = re.compile(rf"^(\s*{re.escape(yaml_key)}:\s*)[-\d.eE+]+.*$", re.MULTILINE)
        if pattern.search(text):
            text = pattern.sub(lambda m, v=value: f"{m.group(1)}{v:.6g}", text, count=1)
        else:
            text = text.rstrip() + f"\n   {yaml_key}: {value:.6g}\n"
    path.write_text(text, encoding="utf-8")


def gate(cand: Metrics, base: Metrics, tol_hit: float, tol_hps: float, tol_sim: float) -> tuple[bool, str]:
    for name in base.hit:
        if cand.hit[name] < base.hit[name] - tol_hit:
            return False, f"{name} hit {cand.hit[name]:.1f} < {base.hit[name]:.1f}-{tol_hit}"
        if cand.hps[name] < base.hps[name] - tol_hps:
            return False, f"{name} hits/s {cand.hps[name]:.1f} < {base.hps[name]:.1f}-{tol_hps}"
    if cand.sim < base.sim - tol_sim:
        return False, f"sim {cand.sim:.1f} < {base.sim:.1f}-{tol_sim}"
    return True, ""

def main() -> int:
    ap = argparse.ArgumentParser(description="closed-loop parameter improvement")
    ap.add_argument("--repo", default=".")
    ap.add_argument("--exe", default="build/Release/autoaim_video.exe")
    ap.add_argument("--sim-exe", default="build/Release/autoaim_sim.exe")
    ap.add_argument("--video-dir", default="")
    ap.add_argument("--iterations", type=int, default=6)
    ap.add_argument("--candidates", type=int, default=3)
    ap.add_argument("--max-minutes", type=float, default=90.0)
    ap.add_argument("--tol-hit", type=float, default=0.8)
    ap.add_argument("--tol-hps", type=float, default=0.3)
    ap.add_argument("--tol-sim", type=float, default=3.0)
    ap.add_argument("--improve", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--dry-run", action="store_true", help="不写 config，只打印")
    ap.add_argument("--commit", action="store_true", help="接受后 git commit config")
    ap.add_argument("--log", default="tools/auto_improve_log.jsonl")
    args = ap.parse_args()

    repo = Path(args.repo).resolve()
    exe, sim_exe = Path(args.exe).resolve(), Path(args.sim_exe).resolve()
    if not exe.is_file():
        print(f"找不到 {exe}", file=sys.stderr)
        return 1

    vdir = Path(args.video_dir) if args.video_dir else repo / "video"
    if not vdir.is_dir():
        for cand in (repo.parent / "template/template/video", repo.parent.parent / "template/template/video"):
            if cand.is_dir():
                vdir = cand
                break
    videos = {"3": vdir / "3.avi", "4": vdir / "4.avi"}
    for v in videos.values():
        if not v.is_file():
            print(f"找不到视频 {v}", file=sys.stderr)
            return 1

    cfg_path = repo / "config" / "default.yaml"
    current = load_yaml(cfg_path)
    missing = [k for k in SPACE if k not in current]
    if missing:
        print(f"配置缺少 {missing}", file=sys.stderr)
        return 1

    rng = random.Random(args.seed)
    deadline = time.time() + args.max_minutes * 60
    log_path = repo / args.log

    print("== 基线 ==")
    base = evaluate(repo, exe, sim_exe, videos, current, "base")
    print(f"  hit {base.hit}  hps {base.hps}  sim {base.sim:.1f}  score {base.score:.3f}")

    accepted = 0
    for it in range(args.iterations):
        if time.time() > deadline:
            print("到达时间上限，停止")
            break
        best: tuple[Metrics, dict[str, float]] | None = None
        for c in range(args.candidates):
            cand = dict(current)
            for key, (lo, hi, step, log_scale) in SPACE.items():
                if rng.random() > 0.45:
                    continue
                delta = step * rng.uniform(-1.5, 1.5)
                if log_scale:
                    cand[key] = float(min(hi, max(lo, cand[key] * (1.0 + delta))))
                else:
                    cand[key] = float(min(hi, max(lo, cand[key] + delta)))
            try:
                m = evaluate(repo, exe, sim_exe, videos, cand, f"i{it}c{c}")
            except Exception as exc:  # noqa: BLE001
                print(f"  cand {c} 失败: {exc}")
                continue
            ok, why = gate(m, base, args.tol_hit, args.tol_hps, args.tol_sim)
            print(f"  cand {c}: score {m.score:.3f} hit {m.hit} hps {m.hps} sim {m.sim:.1f} "
                  f"{'OK' if ok else 'GATE ' + why}")
            with log_path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps({"iter": it, "cand": c, "params": cand, "score": m.score,
                                     "hit": m.hit, "hps": m.hps, "sim": m.sim,
                                     "gate": ok, "why": why}) + "\n")
            if ok and (best is None or m.score > best[0].score):
                best = (m, cand)

        if best and best[0].score > base.score + args.improve:
            base, current = best
            accepted += 1
            print(f"  -> 接受，score {base.score:.3f}")
            if not args.dry_run:
                write_yaml(cfg_path, current)
                if args.commit:
                    msg = (f"auto_improve: score {base.score:.3f} "
                           f"(hit {base.hit}, sim {base.sim:.1f})")
                    run(["git", "add", "config/default.yaml"], repo)
                    run(["git", "commit", "-m", msg], repo)
        else:
            print("  -> 无改进")

    print(f"\n== 结束：接受 {accepted} 次 ==")
    print("  --set " + " --set ".join(f"{k}={v:.6g}" for k, v in current.items()))
    print(f"  日志 {log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
