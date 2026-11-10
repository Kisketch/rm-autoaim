#!/usr/bin/env python3
"""R-03: validate the camera->gimbal extrinsic before it reaches the robot.

`config/default.yaml` may carry `R_cam2gimbal` and `t_cam2gimbal`. A wrong
rotation does not crash anything: the aim point just ends up one plate off, and
every tuning experiment afterwards is meaningless. This script checks the
numbers you are about to flash.

Checks (all must pass):
 1. R is a proper rotation: R^T R = I (tol 1e-6), det(R) = +1 (tol 1e-6).
 2. Round-trip: cameraToWorld(worldToCamera(p)) == p for random points (tol 1e-9).
 3. Translation magnitude is plausible: 0 < |t| < 0.5 m.
 4. Optional --selftest: build a known R/t, verify the checker accepts it and
    rejects a perturbed one.

Usage:
    python3 tools/check_extrinsics.py --selftest
    python3 tools/check_extrinsics.py --config config/default.yaml
"""

from __future__ import annotations

import argparse
import math
import random
import sys

import numpy as np


def parse_list(text: str) -> list[float]:
    body = text.strip().strip("[]")
    return [float(x) for x in body.replace(",", " ").split()] if body else []


def read_extrinsics(path: str) -> tuple[np.ndarray, np.ndarray]:
    R = np.eye(3)
    t = np.zeros(3)
    for line in open(path, encoding="utf-8", errors="replace"):
        s = line.strip()
        if s.startswith("R_cam2gimbal:"):
            vals = parse_list(s.split(":", 1)[1])
            if len(vals) == 9:
                R = np.array(vals).reshape(3, 3)
        elif s.startswith("t_cam2gimbal:"):
            vals = parse_list(s.split(":", 1)[1])
            if len(vals) == 3:
                t = np.array(vals)
    return R, t


def check(R: np.ndarray, t: np.ndarray, verbose: bool = True) -> bool:
    ok = True
    ortho = np.linalg.norm(R.T @ R - np.eye(3), ord=np.inf)
    det = np.linalg.det(R)
    if ortho > 1e-6:
        ok = False
        if verbose:
            print(f"FAIL  R is not orthonormal: |R^T R - I| = {ortho:.3e}")
    if abs(det - 1.0) > 1e-6:
        ok = False
        if verbose:
            print(f"FAIL  det(R) = {det:.6f}, expected +1")
    if not (0.0 < np.linalg.norm(t) < 0.5):
        ok = False
        if verbose:
            print(f"FAIL  |t| = {np.linalg.norm(t):.4f} m outside (0, 0.5)")

    # Semantic check: the camera looks forward along gimbal +x; its right axis maps
    # to gimbal -y and its down axis to gimbal -z. A rotation that is valid but
    # wrong (e.g. tilted 10 deg) still passes orthonormality, so this is the check
    # that actually catches a mis-mounted camera.
    expect = [((0, 0, 1.0), (1.0, 0, 0)),
              ((1.0, 0, 0), (0, -1.0, 0)),
              ((0, 1.0, 0), (0, 0, -1.0))]
    for cam_axis, gim_axis in expect:
        cam_axis, gim_axis = np.array(cam_axis), np.array(gim_axis)
        got = R @ cam_axis
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(got @ gim_axis)))))
        if ang > 5.0:
            ok = False
            if verbose:
                print(f"FAIL  camera axis {cam_axis} maps to {np.round(got, 3)}, "
                      f"{ang:.1f} deg away from {gim_axis}")

    rng = random.Random(3)
    worst = 0.0
    for _ in range(200):
        p = np.array([rng.uniform(-8, 8), rng.uniform(-8, 8), rng.uniform(-8, 8)])
        cam = R.T @ (p - t)          # world -> camera
        back = R @ cam + t           # camera -> world
        worst = max(worst, float(np.linalg.norm(back - p)))
    if worst > 1e-9:
        ok = False
        if verbose:
            print(f"FAIL  round-trip error {worst:.3e} m")

    if verbose:
        print(f"  orthonormality  {ortho:.3e}")
        print(f"  det(R)          {det:.6f}")
        print(f"  |t|             {np.linalg.norm(t):.4f} m")
        print(f"  round-trip max  {worst:.3e} m")
        print("RESULT:", "PASS" if ok else "FAIL")
    return ok


def selftest() -> int:
    # camera optical (z forward, x right, y down) -> gimbal (x forward, y left, z up)
    R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    t = np.array([0.10, -0.05, 0.0])
    print("== known-good extrinsic ==")
    good = check(R, t)
    print("== perturbed (10 deg about x) ==")
    a = math.radians(10)
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    bad = check(Rx @ R, t)
    print("\nSELFTEST", "PASS" if (good and not bad) else "FAIL")
    return 0 if (good and not bad) else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/default.yaml")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    R, t = read_extrinsics(args.config)
    print(f"config: {args.config}")
    print("R =\n", np.array2string(R, precision=4))
    print("t =", t)
    return 0 if check(R, t) else 1


if __name__ == "__main__":
    sys.exit(main())
