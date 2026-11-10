# 时间戳与姿态对齐：实现说明（G-02 落地步骤 1–2）

`docs/timestamp_design.md`（G-02）给出了预算，本文件说明代码如何落地其中的
第 1、2 步，以及还差什么。第 3 步（端到端延迟实测）和第 4 步（把实测值写回
`aimer.latency` 并跑回归）不在本次范围内，仍按 G-02 原文执行。

## 新增模块

| 文件 | 内容 |
| --- | --- |
| `include/autoaim/pose_buffer.hpp` / `src/pose_buffer.cpp` | `PoseBuffer`：带时间戳的云台姿态环形缓冲，查询时做四元数 SLERP 插值；`ClockAligner`：远端时钟（MCU/相机设备时钟）到主机时钟的偏移估计（最近样本差值的中位数，抗传输离群）；四元数工具（SLERP、欧拉角 ZYX→四元数、→旋转矩阵） |
| `include/autoaim/camera_source.hpp` / `src/camera_source.cpp` | `CameraSource` 统一采集接口：每帧输出图像 + **曝光中心时刻**（主机时钟）+ 是否硬件时间戳。实现：`VideoFileSource`（离线，合成时间戳，与现有 app 口径一致）、`DeviceSource`（UVC/V4L2 近似戳，明确标注不满足预算）、`MvsSource`（海康 MVS，`AUTOAIM_WITH_MVS=ON` 时编译） |

## 对齐链路（上车时的接法）

1. 相机侧：`MvsSource` 取帧时读取相机设备时间戳（`nTimeStampHigh/Low` ÷
   `TimestampTickFrequency`），每帧用 `ClockAligner` 与主机 `steady_clock`
   配对更新偏移，曝光中心 = 设备曝光起始 + `exposure_us/2`，换算到主机时钟。
   恒定传输延迟会折进偏移。**更正（2026-11）**：这一项对预测时域无害
   （`aimer.latency` 实测的是端到端总量），但对**姿态查询**有害——相机戳比
   姿态流晚 D，测量就被旋转 云台角速度 × D（云台扫动、换板时最大，例如
   换板 6 rad/s × 10 ms = 3.4°，5 m 处约 30 cm）。须用
   `DelayCalibrator`（`delay_calibration.hpp`，静止装甲板 + 扫云台）实测，
   写入 `MvsConfig::stamp_offset_us`。
2. 姿态侧：串口接收线程把每帧 IMU 姿态转成四元数（下位机给欧拉角时用
   `quatFromYawPitchRoll`），时间戳经同一个 `ClockAligner` 思路换算到主机
   时钟后 `PoseBuffer::push`。
3. 视觉侧：处理一帧时用 `pose_buffer.query(frame.exposure_stamp, q)` 取
   该帧曝光时刻的云台姿态，再由 `quatToMat(q)` 得到旋转矩阵进入位姿解算。
   `query` 返回 false（戳超出覆盖范围或两样本间隔超过 `max_gap`，即串口
   丢帧）时**不要**用旧姿态凑数：跳过该帧或至少禁止开火，并计数上报——
   这正是旧实现「取最新值」会悄悄引入整段串口周期误差的地方。

哨兵仓 `apps/sentry_robot.cpp` 现在的 `stamp = nowSeconds()` +
`serial->query(stamp, …)` 取当前值，应替换为上面的 2、3 步；串口协议本身
不需要改（已带 yaw/pitch/roll），只是接收侧要把每帧姿态**连同其时间戳**
存进缓冲，而不是只留最新一帧。

## 验收（对应 G-02 验收线）

- 单元测试 `testPoseBuffer`：3 rad/s 匀速旋转、200 Hz 姿态样本下，插值
  姿态与真值的夹角误差须 < 0.3 ms 等效的 0.0009 rad（匀速旋转时 SLERP
  理论上精确）；同时断言「取最新样本」的误差比插值大 10 倍以上，作为
  对照组防止以后有人把插值改回取最新值。
- `testClockAligner`：带 ±1 ms 高斯抖动和周期性 +50 ms 离群的配对样本，
  中位数偏移误差须 < 2 ms（实测应远小于此，该线只是门禁）。
- 上车验收仍按 G-02 第 4 步：实测延迟写回配置后跑 `tools/regression.py`，
  命中率不得回退。

## 未包含 / 待办

- MVS 部分在本仓库的离线环境**没有 SDK、未编译验证**，需要在装有 MVS 的
  机器上首次编译核对（API 按 MVS 4.x 写：`MV_CC_GetImageBuffer`、
  `MV_FRAME_OUT_INFO_EX` 的设备时间戳字段、`TimestampTickFrequency` 节点）。
  相机若设置成所列之外的像素格式，`grab` 会返回 false，需要按需补
  `MV_CC_ConvertPixelType` 路径。
- 硬件触发模式已留开关（`MvsConfig::hardware_trigger`，Line0），触发信号
  与曝光时刻的对应关系要按 G-02 第 1 步在实机上确认。
- PTP/chrony 主机时钟对齐（G-02 第 1 步后半）是部署配置，不在代码内。
