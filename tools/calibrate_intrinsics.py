#!/usr/bin/env python3
"""Intrinsics calibration harness (R-02).

Two modes:

  python3 tools/calibrate_intrinsics.py --selftest
      Generates synthetic chessboard views with a known camera, runs the same
      OpenCV pipeline that will be used on real images, and checks that the
      recovered fx/fy/cx/cy match. This proves the tool works before the board
      arrives.

  python3 tools/calibrate_intrinsics.py --images calib/ --pattern 9x6 --square 0.025
      Real calibration: finds the board in every image, calibrates, prints the
      RMS reprojection error and writes config/camera_calib.yaml.

Acceptance (R-02): RMS < 0.5 px and fx within 5 % of 2319 (8 mm lens).
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path

import cv2
import numpy as np

IMAGE_SIZE = (1440, 1080)
TRUE_K = np.array([[2319.0, 0, 720.0], [0, 2319.0, 540.0], [0, 0, 1.0]])


def board_points(pattern: tuple[int, int], square: float) -> np.ndarray:
    cols, rows = pattern
    pts = np.zeros((cols * rows, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return pts


def selftest(pattern: tuple[int, int], square: float, views: int = 12) -> int:
    rng = np.random.default_rng(7)
    objp = board_points(pattern, square)
    obj_points, img_points = [], []
    for _ in range(views):
        rvec = rng.normal(0, 0.35, 3)
        tvec = np.array([rng.uniform(-0.12, 0.12), rng.uniform(-0.09, 0.09), rng.uniform(0.35, 0.8)])
        proj, _ = cv2.projectPoints(objp, rvec, tvec, TRUE_K, np.zeros(5))
        img_points.append(proj.reshape(-1, 2).astype(np.float32))
        obj_points.append(objp)

    rms, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, IMAGE_SIZE, None, None)
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    err_fx = abs(fx - TRUE_K[0, 0]) / TRUE_K[0, 0]
    err_cx = abs(cx - TRUE_K[0, 2])
    print(f"selftest: rms={rms:.4f} px  fx={fx:.1f} (err {100*err_fx:.2f}%)  "
          f"fy={fy:.1f}  cx={cx:.2f} (err {err_cx:.2f} px)  cy={cy:.2f}")
    ok = rms < 0.5 and err_fx < 0.01 and err_cx < 2.0
    print("SELFTEST " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def real(images: str, pattern: tuple[int, int], square: float, out: str) -> int:
    files = sorted(glob.glob(f"{images}/*.jpg") + glob.glob(f"{images}/*.png"))
    if not files:
        print(f"no images in {images}")
        return 1
    objp = board_points(pattern, square)
    obj_points, img_points, used = [], [], 0
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        ok, corners = cv2.findChessboardCorners(img, pattern, flags)
        if not ok:
            continue
        corners = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        obj_points.append(objp)
        img_points.append(corners)
        used += 1
    if used < 5:
        print(f"only {used}/{len(files)} usable views; need >=5")
        return 1

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, IMAGE_SIZE, None, None)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = dist.ravel()
    print(f"views={used}  rms={rms:.4f} px")
    print(f"fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
    print("dist=" + " ".join(f"{v:.6g}" for v in d))
    print(f"vs 2319 (8 mm): fx error {100*(fx-2319)/2319:+.2f}%")

    Path(out).parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        fh.write("%YAML:1.0\n---\n")
        fh.write(f"fx: {fx:.6f}\nfy: {fy:.6f}\ncx: {cx:.6f}\ncy: {cy:.6f}\n")
        fh.write("dist: [ " + ", ".join(f"{v:.9g}" for v in d) + " ]\n")
        fh.write(f"rms_px: {rms:.6f}\nviews: {used}\n")
    print(f"wrote {out}")
    print("PASS" if rms < 0.5 else "FAIL (rms >= 0.5 px)")
    return 0 if rms < 0.5 else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--images", default="calib")
    ap.add_argument("--pattern", default="9x6")
    ap.add_argument("--square", type=float, default=0.025, help="metres")
    ap.add_argument("--out", default="config/camera_calib.yaml")
    a = ap.parse_args()
    cols, rows = (int(v) for v in a.pattern.lower().split("x"))
    if a.selftest:
        return selftest((cols, rows), a.square)
    return real(a.images, (cols, rows), a.square, a.out)


if __name__ == "__main__":
    raise SystemExit(main())
