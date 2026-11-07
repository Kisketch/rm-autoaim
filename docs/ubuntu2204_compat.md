# Ubuntu 22.04 / OpenCV 4.5.4 兼容性静态审计（B-05）

目标机：GEEKOM IT13，Ubuntu 22.04（GCC 11、CMake 3.22、OpenCV 4.5.4）。
本仓库在 Windows + OpenCV 4.14、Ubuntu 26.04 + OpenCV 4.10 上验证过；22.04 未实机跑。

## 静态检查结果

- `cmake_minimum_required`：3.16（22.04 自带 3.22，OK）
- C++ 标准：C++17（GCC 11 支持 C++17/20 子集，OK）
- 用到的 OpenCV 模块：calib3d, core, highgui, imgproc, opencv, videoio
- 各模块在 4.5.4 中的可用性：core, imgproc, highgui, videoio, dnn, imgcodecs, calib3d 全部存在

## 需要留意但静态查不出的点

1. **DNN 后端**：4.5.4 的 `dnn` 对 ONNX 算子支持比 4.10 少；`mlp.onnx` 只是全连接，风险低。
2. **`solvePnPGeneric` + IPPE**：4.5.4 有，但返回解的顺序可能与新版本不同——我们的代码取的是重投影误差最小解，不依赖顺序。
3. **`cv::VideoCapture` 的 MJPG 解码**：22.04 的 FFmpeg 版本较旧，若打不开 3.avi/4.avi，装 `libopencv-videoio-dev` 或改用图像序列。
4. **CMake 4.x 语法**：本仓库只用 3.16 起就有的命令，22.04 的 3.22 可以配置。

## 结论

- 静态层面没有发现必然不兼容的用法；风险集中在 DNN 算子与视频解码两条运行时路径。
- 交付前必须在 22.04 上跑一遍：`cmake -S . -B build && cmake --build build -j` + `autoaim_video` 两条视频 + `autoaim_tests`。
- 若 4.5.4 的 DNN 读不了 ONNX，退路是把分类头改成 OpenCV MLP 的 XML 格式，或在 22.04 上装 4.8 的官方包。

*扫描了 11 个源文件；本审计是静态的，不能替代实机编译。*
