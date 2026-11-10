# 共用核心加固移植（2026-11-10）

哨兵仓审查分支 `optimize/core-hardening`（已合并，见该仓 `docs/review_2026-11.md`）
修复的问题中，属于两仓共用核心的部分移植到本仓：

| 修复 | 位置 | 说明 |
|---|---|---|
| `wrapAngle(±inf)` 死循环 | `types.hpp` | 改为 `std::remainder`，O(1)，非有限值原样返回（旧实现在滤波发散成 inf 时实测挂死） |
| Singer 外推不衰减速度 | `tracker.cpp::RobotTracker::predict` | 位置外推同步衰减速度，与 `transition()` 的 F 一致，避免二次外推过冲 |
| 弹道积分提速 | `ballistics.cpp::simulate` | RK4 步长 1 ms → 10 ms + 三次 Hermite 过零插值（对 10 µs 参考解高度误差 < 0.05 µm，原线性插值 1.2 µm） |
| 弹道求解提速 | `ballistics.cpp::solve` | 保留原有真空落差热启动；迭代改为首步不动点、其后割线法（同 0.5 mm 收敛判据），并加非有限输入守卫 |
| 配置合法性检查 | `config.hpp/.cpp::Config::validate` | 加载时一次性列出会变成 NaN/除零/错误几何的参数并抛异常；按本仓字段适配（无 TargetConfig；新增 17/42 弹种检查） |

**有意不移植**（经逐项核对，非遗漏）：
- 中心模式飞行时间提前量：哨兵仓的 TargetState 版火控有此缺陷；本仓火控在迭代内以
  `predict(t0 + t_fly)` 取中心，本来就是对的。
- TargetManager 一身份一轨迹 / 选靶滞回资格复检、串口 raw 模式与 fd 所有权、PoseBuffer
  有界外推、ROI 窗口合并、SentryVision 拷贝守卫：均为哨兵仓专属代码，本仓无对应物。

验证：本仓测试全过（含弹道金标）、仿真表无退步、真实视频 N0 回归与冻结基线一致
（见提交说明）。

## 第二轮（2026-11-10）

- **更正**：PoseBuffer 有界外推并非"本仓无对应物"——`PoseBuffer` 就在本仓（`MvsSource`
  时间戳链路的另一半）。已从哨兵仓同步，默认 `max_extrapolation = 0`，行为不变。
- **相机—姿态时间差**：`ClockAligner` 把相机的中位传输延迟折进偏移，原注释称"无害"。
  对预测时域确实无害，但姿态是在这个偏晚的戳上查的，云台转动时观测被旋转 ω·D
  （换板 6 rad/s × 10 ms ≈ 3.4°）。新增 `DelayCalibrator`（`delay_calibration.hpp`，
  静止装甲板 + 扫云台，散布最小的偏移即所求）与 `MvsConfig::stamp_offset_us`。
  合成数据：真值 12 ms → 估计 11.98 ms，板子散布 8.8 → 1.8 cm（`testDelayCalibration`）。
- 中心模式 `dc` 为 0 时的 0/0 保护。

详见哨兵仓 `docs/review_2026-11_round2.md`。
