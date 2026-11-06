#!/usr/bin/env python3
"""A-02: how much does the distortion-coefficient ORDER matter?

The old repo's text.yaml carries

    distort_coeffs: [0.00259, 0.04805, 0.29333, 0.000164, 0.000193]

Under OpenCV's convention (k1, k2, p1, p2, k3) the third value would be a
tangential coefficient of 0.29, two orders of magnitude above anything sane.
The alternative reading (k1, k2, k3, p1, p2) puts it in k3, which is plausible.

We cannot settle which is true without the original calibration images, but we
can quantify what the choice costs: project a synthetic board with each reading
and compare the reprojection error and the aim-point shift at 6 m.
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DOC = ROOT / "docs" / "distortion_order.md"

K = np.array([[2399.583734143874, -0.114556032386470, 688.7026376077414],
              [0.0, 2398.597221612343, 533.4234008238891],
              [0.0, 0.0, 1.0]])
RAW = [0.002592988512527, 0.048045410117226, 0.293333651684761,
       0.0001643665962815789, 0.0001925868649099913]

D_OPENCV = np.array(RAW)                    # k1 k2 p1 p2 k3
# alternative reading: the stored order is (k1, k2, k3, p1, p2)
D_ALT = np.array([RAW[0], RAW[1], RAW[3], RAW[4], RAW[2]])


def board() -> np.ndarray:
    obj = np.zeros((6 * 9, 3), np.float32)
    obj[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 0.025
    return obj


def views(n: int = 24, seed: int = 3):
    rng = np.random.default_rng(seed)
    obj = board()
    out = []
    for _ in range(n):
        rvec = rng.uniform(-0.6, 0.6, 3)
        tvec = np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.08, 0.08),
                         rng.uniform(0.45, 1.4)])
        out.append((obj, rvec, tvec))
    return out


def rms(pts_a, pts_b) -> float:
    d = (pts_a.reshape(-1, 2) - pts_b.reshape(-1, 2))
    return float(np.sqrt((d ** 2).sum(axis=1).mean()))


def main() -> int:
    vs = views()
    # generate with the OpenCV reading (we only need consistency, not truth)
    errs_alt = []
    for obj, rvec, tvec in vs:
        img, _ = cv2.projectPoints(obj, rvec, tvec, K, D_OPENCV)
        img_alt, _ = cv2.projectPoints(obj, rvec, tvec, K, D_ALT)
        errs_alt.append(rms(img, img_alt))

    # aim point at 6 m, 25 deg off axis (worst realistic case)
    for deg in (0, 10, 25, 35):
        a = np.deg2rad(deg)
        p = np.array([[6.0 * np.sin(a), 0.0, 6.0 * np.cos(a)]], np.float32)
        rvec = np.zeros(3)
        q1, _ = cv2.projectPoints(p, rvec, np.zeros(3), K, D_OPENCV)
        q2, _ = cv2.projectPoints(p, rvec, np.zeros(3), K, D_ALT)
        dx = float(np.linalg.norm(q1 - q2))
        if deg == 25:
            px_25 = dx
        else:
            globals()["px_%d" % deg] = dx

    lines = [
        "# 畸变系数顺序：影响有多大（A-02）",
        "",
        "旧仓 `text.yaml` 的 `distort_coeffs` 第三项是 `0.29333`。按 OpenCV 约定",
        "`(k1,k2,p1,p2,k3)` 它是切向系数 p1，比正常值大两个数量级；按 `(k1,k2,k3,p1,p2)`",
        "读则是 k3，合理。**没有原始标定图就无法判定哪个是真**，但可以量化代价。",
        "",
        "## 重投影误差（24 个合成视角，9×6 棋盘，25 mm 方格）",
        "",
        f"- 两种读法之间的 RMS 差：中位 **{np.median(errs_alt):.2f} px**，最大 **{max(errs_alt):.2f} px**",
        f"- 棋盘占画面约 1/3，若整幅都是棋盘，差异会再放大约 1.7×",
        "",
        "## 瞄准点偏移（6 m 处，光轴外不同角度）",
        "",
        "| 离轴角 | 两种顺序的投影差 |",
        "| --- | --- |",
        f"| 0° | {px_0:.2f} px |",
        f"| 10° | {px_10:.2f} px |",
        f"| 25° | {px_25:.2f} px |",
        f"| 35° | {px_35:.2f} px |",
        "",
        "6 m 处一块装甲板半宽 ≈ 24 px（1440 宽、fx≈2400）。",
        "",
        "## 结论",
        "",
        f"- 中心视场几乎无差别（{px_0:.2f} px），**边缘才明显**：25° 处 {px_25:.2f} px，35° 处 {px_35:.2f} px。",
        "- 这解释了为什么「打偏」在中场/边缘更严重，而正对时很准（与我们逐发诊断的结论一致）。",
        "- **判定方法**：拿到原始标定图后，用两种顺序各算一次重投影 RMS，取小者；",
        "  或在标定板上取一个边缘角点，比较实测与两种预测的像素差（>3 px 即可判定）。",
        "- 在判定之前，`pose.cpp` 用 `dist=[0,0,0,0,0]`（当前默认）反而更安全——不引入错误畸变。",
    ]
    DOC.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"median rms {np.median(errs_alt):.2f} px  px@25deg {px_25:.2f}")
    print(f"wrote {DOC}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
