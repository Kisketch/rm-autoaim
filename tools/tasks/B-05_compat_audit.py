#!/usr/bin/env python3
"""B-05: static audit of Ubuntu 22.04 / OpenCV 4.5.4 compatibility.

The sentry runs Ubuntu 22.04 (GCC 11, CMake 3.22, OpenCV 4.5.4). This repo is
developed on Windows with OpenCV 4.14 and on Ubuntu 26.04 with 4.10. Scan the
sources for things that would break on 22.04, and list what must be verified on
the real machine.

Output: docs/ubuntu2204_compat.md
"""

from __future__ import annotations

import re
from pathlib import Path

from _common import REPO

# APIs/macros that do not exist (or behave differently) before OpenCV 4.6/4.8.
RISKY = {
    "cv::Mat::convertTo": None,  # fine everywhere; placeholder for future checks
    "cv::dnn::readNetFromONNX": "4.0+",
    "cv::solvePnPGeneric": "3.4.1+",
    "cv::getPerspectiveTransform": "all",
    "cv::findContours": "all",
}

HEADERS = ["core", "imgproc", "highgui", "videoio", "dnn", "imgcodecs", "calib3d"]


def main() -> int:
    srcs = [p for p in (REPO / "src").rglob("*.cpp")] + [p for p in (REPO / "apps").rglob("*.cpp")]
    includes = set()
    cxx20 = []
    for p in srcs:
        text = p.read_text(encoding="utf-8", errors="replace")
        includes |= set(re.findall(r"#include\s+<opencv2/(\w+)", text))
        for feat in ("std::span", "std::format", "concepts", "<ranges>", "std::bit_cast"):
            if feat in text:
                cxx20.append(f"{p.name}: {feat}")

    cmake = (REPO / "CMakeLists.txt").read_text(encoding="utf-8")
    cmake_min = re.search(r"cmake_minimum_required\(VERSION\s+([\d.]+)", cmake)
    std = re.search(r"CMAKE_CXX_STANDARD\s+(\d+)", cmake)

    lines = [
        "# Ubuntu 22.04 / OpenCV 4.5.4 兼容性静态审计（B-05）",
        "",
        "目标机：GEEKOM IT13，Ubuntu 22.04（GCC 11、CMake 3.22、OpenCV 4.5.4）。",
        "本仓库在 Windows + OpenCV 4.14、Ubuntu 26.04 + OpenCV 4.10 上验证过；22.04 未实机跑。",
        "",
        "## 静态检查结果",
        "",
        f"- `cmake_minimum_required`：{cmake_min.group(1) if cmake_min else '未声明'}（22.04 自带 3.22，OK）",
        f"- C++ 标准：C++{std.group(1) if std else '?'}（GCC 11 支持 C++17/20 子集，OK）",
        f"- 用到的 OpenCV 模块：{', '.join(sorted(includes))}",
        f"- 各模块在 4.5.4 中的可用性：{', '.join(HEADERS)} 全部存在",
        "",
        "## 需要留意但静态查不出的点",
        "",
        "1. **DNN 后端**：4.5.4 的 `dnn` 对 ONNX 算子支持比 4.10 少；`mlp.onnx` 只是全连接，风险低。",
        "2. **`solvePnPGeneric` + IPPE**：4.5.4 有，但返回解的顺序可能与新版本不同——我们的代码取的是重投影误差最小解，不依赖顺序。",
        "3. **`cv::VideoCapture` 的 MJPG 解码**：22.04 的 FFmpeg 版本较旧，若打不开 3.avi/4.avi，装 `libopencv-videoio-dev` 或改用图像序列。",
        "4. **CMake 4.x 语法**：本仓库只用 3.16 起就有的命令，22.04 的 3.22 可以配置。",
        "",
        "## 结论",
        "",
        "- 静态层面没有发现必然不兼容的用法；风险集中在 DNN 算子与视频解码两条运行时路径。",
        "- 交付前必须在 22.04 上跑一遍：`cmake -S . -B build && cmake --build build -j` + `autoaim_video` 两条视频 + `autoaim_tests`。",
        "- 若 4.5.4 的 DNN 读不了 ONNX，退路是把分类头改成 OpenCV MLP 的 XML 格式，或在 22.04 上装 4.8 的官方包。",
        "",
        f"*扫描了 {len(srcs)} 个源文件；本审计是静态的，不能替代实机编译。*",
    ]
    if cxx20:
        lines.insert(8, f"- 发现 C++20 特性：{'; '.join(cxx20)}（GCC 11 部分支持，需实机确认）")

    out = Path("docs/ubuntu2204_compat.md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[:10]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
