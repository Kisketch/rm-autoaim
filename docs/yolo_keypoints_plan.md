# YOLO 关键点检测：可行性、数据集与接入方案

> 状态：**推理接入已实现**（`src/keypoint_detector.cpp`，`backend: "onnx"` 开关 + 模型缺失自动回退传统方案，后处理有合成张量单元测试）。未做的是训练与真实模型验收——仍缺 GPU 与数据集。
> 结论先说：本机（无 GPU、VM 无 GPU）不具备训练条件；推理接入可以先做，训练必须换到 GPU 机器或云端。

## 1. 为什么需要它

传统灯条检测（`src/detector.cpp`）的失效模式在两条视频和仿真里都能看到：

| 失效模式 | 现状 | 影响 |
| --- | --- | --- |
| 只露出一条灯条（正侧对） | 无法配对，整帧漏检 | 两条视频最长连续漏检 8 帧 |
| 灯条被装甲板图案切断 | 形态学闭运算能补一部分 | 远处小目标易断 |
| 琥珀色装饰灯 | 通道差阈值能压住大部分 | 强光场景仍需调阈值 |
| 数字图案模糊 | 28×28 MLP，只验证过“3” | 换机器人号段需重训 |

关键点模型直接回归 4 个角点，天然处理遮挡与侧视，是各队 2024 之后的主流方案（同济 sp_vision_25、深大 RobotPilots、北科 Reborn 都是这条路）。

## 2. 数据集

| 数据集 | 规模 | 标注 | 许可/获取 |
| --- | --- | --- | --- |
| Roboflow `aruw/armor-plate-keypoints` | 18 557 张 | 4 关键点 | 公开页面，下载需 Roboflow 账号/API key |
| Roboflow `sam-fplas/robomaster-armor-plate` | 23 282 张 | 检测框 | 同上 |
| Ultralytics `liyihan/robomaster-armor` | 未核对 | 检测框 | 同上 |
| 自采 | 建议 3–5k 张 | 4 关键点 | 需相机 + 标定板，覆盖侧视/遮挡/强光 |

> 数字来自各数据集页面展示的公开统计（2026-11 检索），未逐张校验；下载后先做去重与类别统计。

## 3. 模型与训练

- 结构：YOLOv8n-pose / YOLO11n-pose（4 关键点 + 1 类），输入 640×640，输出 `(x, y, conf) × 4`。
- 损失：关键点 OKS + box；关键点可见性用 `v=2/0` 处理遮挡样本。
- 数据增强：HSV（灯条过曝）、随机透视、马赛克关闭（关键点易错）、运动模糊。
- **算力核算**：18.5k 张、100 epoch、640 输入，单张前反向约 25 ms（GTX 1660 级）→ 约 13 GPU·h；本机 CPU 约 1.5 s/张 → **≈ 770 CPU·h，不可行**。
  → 结论：训练必须用 GPU（实验室机器 / Colab / AutoDL），本机只做推理接入与评测。

## 4. 接入设计（不改动调用方）

```
FrameDetections
   ├── detector.cpp        （传统灯条，现状，默认）
   └── keypoint_detector.cpp（ONNX，新增）
```

- `DetectorConfig` 增加 `backend: "traditional" | "onnx"`、`onnx_path`、`conf_threshold`、`nms_threshold`。
- 新类实现与 `Detector::detect` 相同的签名，返回 `std::vector<ArmorDetection>`：
  - 4 关键点按 **左下→左上→右上→右下** 排序后填 `corners`；
  - 由关键点构造 `light`（左右两条边的两端），保持 `pose.cpp` 的输入不变；
  - 类别号映射到 `number`；置信度直接透传。
- 推理：OpenCV DNN（`readNetFromONNX` + `blobFromImage`）即可，CPU 上 640 输入约 8–15 ms；要进 5 ms 预算需 TensorRT/OpenVINO（阶段三后半段）。
- 回退：`backend=traditional` 时行为与现在完全一致，CI 不依赖 ONNX 文件。

## 5. 验收标准

| 项 | 判据 |
| --- | --- |
| 召回 | 侧对/遮挡帧的检出率 ≥ 传统方案的 1.3 倍（在 100 帧人工标注集上） |
| 精度 | 关键点重投影误差中位 ≤ 3 px（1440×1080） |
| 延迟 | CPU（OpenCV DNN）≤ 15 ms/帧；TensorRT ≤ 3 ms/帧 |
| 回归 | 3.avi / 4.avi 命中率不低于 `BASELINE.md` §10（66.6% / 65.4%） |
| 依赖 | 不引入 OpenCV 之外的运行时依赖（训练侧依赖不计入） |

## 6. 下一步（可直接执行）

1. 找有 GPU 的机器，申请 Roboflow API key，下载 ARUW 18 557 张 + 自采 1k 张。
2. 训练 YOLO11n-pose（100 epoch），导出 ONNX（opset 12，动态 batch=1）。
3. 按第 4 节接入，跑 `autoaim_video` 两条视频 + `autoaim_sim`，对比第 5 节。
4. 通过后再考虑 TensorRT / OpenVINO 与 INT8 量化。
