# 共用核心清单（shared core manifest）

`rm-autoaim` 与 `rm-sentry-integration` 共用同一套算法核心。历史上两仓靠人工
移植同步，已经出现过多次"一仓修了、另一仓漏掉"（弹道热启动、PoseBuffer 外推、
wrapAngle 等）。本清单把共用文件的同步状态显式化，并由
`tools/check_shared_core.sh` + CI（`shared-core.yml`）自动把关。

## 必须逐字节一致（identical set）

下列文件在两仓必须完全相同。改任何一个，必须同批同步到另一仓；
CI 会检出分叉并失败。

- `include/autoaim/pose_buffer.hpp`、`src/pose_buffer.cpp`
- `include/autoaim/delay_calibration.hpp`、`src/delay_calibration.cpp`
- `include/autoaim/draw.hpp`、`src/draw.cpp`
- `include/autoaim/evaluation.hpp`
- `docs/shared_core.md`（本文件）

## 有意分叉（classified forks，不要"顺手同步"）

| 文件 | 分叉内容 | 原因 |
|---|---|---|
| `types.hpp` | 哨兵多 `ArmorType::OUTPOST` 与前哨站几何 | 前哨站只有哨兵仓需要 |
| `pose.cpp` / `pose.hpp` | 姿态架构不同：主仓固定安装倾角（离线/仿真世界），哨兵实时串口欧拉角 `setGimbalAttitude` | 两条链路的姿态来源不同 |
| `ballistics.*` | 主仓有弹种 API（`bullet_mm`、42mm、`setMeasuredBulletSpeed`）；哨兵只有 17mm 配置面 | 哨兵协议无弹速字段，P0-3 当时即决定不移植；数值路径（RK4/割线/热启动）两边一致 |
| `config.*` | 各仓专属配置节（主仓 keypoint/camera-source，哨兵 target/outpost） | 共用字段的校验规则两边一致（`validate()` 已核对） |
| `detector.*` | 主仓多自适应通道阈值与 YOLO 关键点后端接线；哨兵按编号识别前哨站 | 主线功能未移植到哨兵，哨兵检测行为以其自身回归为准 |
| `tracker.*` | 主仓多 `carry_velocity`、匹配门 `match_dt_scale`；哨兵多 TargetState 快照（`centerAt/armorAt`）、OutpostTracker | 各自的跟踪层需求不同；EKF 本体（Singer/NIS）一致 |
| `aimer.*` | 主仓有 CommandRate（B-01 速率前馈输出）；哨兵在应用层差分 | 电控接口不同；火控数值路径一致 |
| `evaluation.cpp` | 哨兵多 sentry 策略行 | 评测对象不同 |
| `tests/test_main.cpp` | 哨兵是精简版（3 个测试），主仓是完整版 | 已知覆盖差：哨兵共用核心的回归主要由 `tests/test_sentry.cpp` 与主仓同名测试承担（见全量审计文档） |

## 单仓独有（不属于共用核心）

- 仅主仓：`camera_source.*`、`keypoint_detector.*`
- 仅哨兵仓：`target.*`、`sentry/*`、`io/*`、`roi.*`、`serial_protocol.*`

## 同步规则

1. 改 identical set 中的文件 → 同一个工作批次内把相同内容提交到两仓。
2. 改共用数值路径（ballistics/tracker/aimer 的算法本体）→ 两仓都要过
   测试 + 仿真 + 实拍回归后才算完成。
3. 有意把某文件从 identical 降级为 fork，或反之 → 先改本清单并说明原因，
   两仓同批提交。
