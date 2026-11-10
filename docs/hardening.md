# 健壮性修复与缺口补齐（2026-11-10）

用退化输入探测（空帧/1×1/灰度/浮点图/退化角点/NaN 观测/弹道边界/配置缺失）逐项压测后修复与补齐的内容。两仓同步（哨兵仓对应提交见其仓库）。

## 修复的崩溃：检测器对空帧段错误

`Detector::detect` 喂入空 Mat 时 **segfault**：`cv::split` 对空图返回 0 个通道，随后读 `ch[2]` 越界。相机掉帧在实车上就是空 Mat，这会直接杀死视觉进程。灰度图（1 通道）走旧回退路径时同理。
修复：`detect` 入口统一 `normalizeFrame`——空帧/小于 2×2 返回无装甲；灰度转 BGR（其通道差恒为 0，语义不变）；其余非 8 位 1/3 通道输入拒绝而非崩溃。回归测试 `testDetectorDegenerateInputs`（两仓）。

探测中确认无恙的：PnP 对全重合/NaN/零/超大角点全部正确拒绝；弹道在 s=0/负距/500 m 返回 `ok=false` 且 Aimer 有检查；跟踪器对 2000 帧空观测与 NaN 观测不产出 NaN 指令；PoseBuffer 乱序入队有有序插入、查询间隙超限正确返回失败；配置缺失抛带路径的明确异常。

## 补齐的缺口 1：相机→云台杠杆臂外参（CameraModel）

此前新模型只有旋转、没有平移，旧代码实测的杠杆臂 t = [0.10, −0.05, 0] m 无法表示（数据清单遗留项）。现与哨兵仓同构：`CameraConfig.R_cam2gimbal`（9 值，行主序）/ `t_cam2gimbal`（3 值，米）可选配置，`cameraToWorld/worldToCamera` 完整刚体变换，PnP 解算走 `cameraToWorld`。默认不配置 = 历史行为（测试与仿真逐项不变）。`testCameraExtrinsic` 验证：往返变换恒等、用外参模型生成又解算的位姿误差 < 1 cm、而默认模型在同数据上保留 > 3 cm 的杠杆臂误差。旧值的坐标系约定需上车核对后才可启用，`config/default.yaml` 里以注释形式给出。

## 补齐的缺口 2：火控固定补偿角（Aimer）

旧代码 Transmit 里写死的 compensate（yaw +0.5°、pitch −0.5°）在新代码无处安放。现 `AimerConfig.yaw_offset_deg / pitch_offset_deg`（默认 0），在三个 Aimer 的最终指令上统一施加（速率由施加后的指令计算）。`testAimerOffsets` 验证偏移量精确到 1e-9 rad 且不影响开火判定。旧数值是旧坐标系下的，上车按新约定重测后填入，yaml 里有注释说明。

## 旧代码侧（交付包 auto-aim-legacy）

`BuffDetect` 硬编码 1080×1080 ROI 在非 1440×1080 输入上抛异常杀进程，已改为裁剪到实际帧范围（1440×1080 时行为逐像素不变）；复验：原尺寸与两种曾崩溃尺寸下打符全链均跑通。
