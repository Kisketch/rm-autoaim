# 全量审计（2026-11-10，第三轮把关）

范围：`rm-autoaim` @ `86cc5eb` 与 `rm-sentry-integration` @ `1f6cab0` 全部源码。
方法：仓库卫生扫描、高警告构建、ASan+UBSan 全路径运行、两仓共用文件逐个
diff 分类、新模块退化探针、MVS 分支桩编译。修复均以"构建 + 全部测试 +
仿真 + 实拍回归与基线一致"为合入条件。

## 发现并已修复

1. **瞄准器悬垂引用（真实 UB）**：`RobotAimer / PointAimer / DirectAimer`
   以引用成员 `const Ballistics& ball_` 持有弹道对象。测试
   `testAimerOffsets` 传入临时对象，临时量在声明语句结束即销毁，后续
   每次 `aim()` 都是 use-after-scope —— ASan 构建下单元测试直接 ABORT
   （栈读越界，定位到 `Ballistics::bulletSpeed()`）。生产调用点目前都
   持有长生命周期对象，未触发，但该 API 形态本身就是雷。修复：三个
   瞄准器改为**按值持有** `Ballistics`（小对象；`setMeasuredBulletSpeed`
   无生产侧后改调用，不受影响）。两仓同改。
2. **PoseBuffer 不拒绝非有限样本**：串口坏帧可解码出 NaN 浮点，推入
   PoseBuffer 后 NaN 比较破坏时间排序并污染后续所有插值/外推。修复：
   `push()` 对 stamp 与四元数做 `isfinite` 检查，非有限样本直接丢弃
   （受影响帧查询失败被跳过，即设计内的降级）。两仓同改
   （`pose_buffer.cpp` 本就是逐字节一致文件），并新增回归测试
   `testPoseBufferNonFinite`（两仓）。
3. **哨兵帧号由墙钟推导**：`SentryVision` 用 `lround(stamp * fps)` 填
   `observations.frame`（第二轮审查指出 200 fps 下约 124 天 int 溢出）。
   改为进程内顺序计数器，与评测侧的帧序号语义一致。
4. **哨兵 README 失效引用**：`BASELINE.md`、`reports/` 链接指向本仓不
   存在的文件，改为 rm-autoaim 仓库地址（文件头注记此前已说明归属，
   正文链接未同步修正）。
5. **注释级分叉收敛**：`delay_calibration.hpp`、`pose_buffer.hpp` 此前
   两仓仅注释不同（各引用本仓的标定入口），已统一为中立的双仓表述，
   两文件恢复逐字节一致并纳入 identical set。

## 结构性措施（防复发）

- 新增 `docs/shared_core.md`：共用文件清单三分法（必须一致 / 有意分叉
  及原因 / 单仓独有）与同步规则。
- 新增 `tools/check_shared_core.sh` + CI 工作流 `shared-core.yml`：
  检出姊妹仓并比对 identical set，分叉即失败。
- 主仓删除被跟踪的工具会话杂文件（`.omo/run-continuation/*.json`、
  `tools/opencode/rm-guard/`），`.gitignore` 增加 `.omo/`。

## 核查通过、无需改动

- 两仓 `-Wall -Wextra` 构建零警告；`-Wshadow -Wconversion` 严格档的
  提示均为良性（float 数学中的 int→float、计数 size_t→double、
  config.cpp 局部变量与 OutpostTracker 成员 `R` 的同名遮蔽），已逐条看过。
- ASan+UBSan：修复第 1 项后，两仓全部单元测试、仿真、实拍评测
  （4.avi，68.6% 与基线一致）及哨兵 SentryVision 800 帧实拍
  （ROI 开/关观测 890=890）全部干净。
- MVS 分支首次完成编译验证：按 CMake 期望的 SDK 目录布局
  （`include/MvCameraControl.h` + `lib/64/libMvCameraControl.so`）用
  桩 SDK 构建 `AUTOAIM_WITH_MVS=ON` 全仓通过、测试全过；`open()` 有
  设备序号边界检查、构造异常安全。剩余风险只是与车队真实 SDK 版本的
  ABI 对齐，仍按试机清单在车上确认。
- 退化探针 15 项全过：DelayCalibrator（无帧/无姿态/云台不动/NaN/
  单帧均正确拒绝）、PoseBuffer（空/单样本精确戳/外推边界/NaN）、
  ClockAligner 退化配对。
- `Config::validate()` 两仓对共用字段的检查逐项一致。
- 共用数值路径回归：主仓 N0 实拍 67.5%/68.6% 与冻结基线逐项一致、
  仿真逐格一致；哨兵策略 67.6%/71.9% 与第二轮一致。本轮全部修复
  行为中性。

## 已知保留项（非缺陷，决策记录在 shared_core.md）

- 两仓共用文件的有意分叉（前哨站类型、姿态架构、弹种 API、检测与
  跟踪的主线功能差）逐项分类在 `docs/shared_core.md`；抽共用库的
  拓扑改造（submodule/FetchContent）仍留待车队决定，不在本轮范围。
- 哨兵 `tests/test_main.cpp` 为精简版，覆盖差已记录在清单中。
