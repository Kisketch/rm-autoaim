# 参数清单（E-03）

把 `config.hpp` 的字段、`config/default.yaml` 的取值、`setParam()` 的可调名三者对齐。
**能否 `--set`** 决定了它能否进自动扫描；不能的只能改文件后重编译或重跑。

共 68 个参数：可直接 `--set` 的 **34** 个，需改文件的 **34** 个。

## 可直接扫描

| 参数 | 当前值 |
| --- | --- |
| `armor_pitch_deg` | 15 |
| `bullet_speed` | 25 |
| `ca_model` | 0 |
| `center_fire_window_deg` | 15 |
| `center_mode_omega` | 12 |
| `channel_diff_k` | 4.5 |
| `channel_diff_max` | 120 |
| `channel_diff_min` | 40 |
| `channel_diff_threshold` | 70 |
| `fire_sigma_scale` | 0 |
| `fire_tolerance_scale` | 0.9 |
| `fx` | 2400 |
| `gimbal_ff` | 0 |
| `gimbal_tau` | 0.03 |
| `latency` | 0.06 |
| `max_fire_facing_deg` | 30 |
| `max_match_distance` | 0.35 |
| `optimize_yaw` | 1 |
| `pixel_noise` | 1 |
| `q_acc` | 2 |
| `q_adapt_max` | 20 |
| `q_dz` | 0.0001 |
| `q_r` | 0.003 |
| `q_xyz` | 1 |
| `q_yaw` | 3 |
| `r_yaw` | 0.01 |
| `r_yaw_head_on` | 0.02 |
| `refine_corners` | 1 |
| `side_angle_deg` | 20 |
| `singer_tau` | 0.3 |
| `switch_hysteresis_deg` | 3 |
| `use_drag` | 1 |
| `warmup_frames` | 0 |
| `yaw_gain` | 1.3 |

## 需要改配置文件（不支持 --set）

| 参数 | 当前值 |
| --- | --- |
| `armors_num` | 4 |
| `classify_threshold` | 0.7 |
| `cx` | 720 |
| `cy` | 540 |
| `drag_k` | 0.019 |
| `fps` | 25 |
| `fy` | 2400 |
| `gimbal_max_rate` | 12 |
| `gravity` | 9.8 |
| `init_radius` | 0.25 |
| `large_plate_ratio` | 3.2 |
| `lost_time_thres` | 0.4 |
| `max_center_y_diff` | 0.6 |
| `max_gap_ratio` | 3.2 |
| `max_length_ratio` | 1.7 |
| `max_light_aspect` | 12 |
| `max_light_tilt_deg` | 40 |
| `max_match_yaw_diff` | 1 |
| `max_pair_angle_diff_deg` | 12 |
| `max_plate_ratio` | 5 |
| `max_radius` | 0.4 |
| `min_fill_ratio` | 0.4 |
| `min_gap_ratio` | 0.15 |
| `min_light_aspect` | 1.6 |
| `min_light_length` | 8 |
| `min_plate_ratio` | 1.1 |
| `min_radius` | 0.12 |
| `min_switch_omega` | 1 |
| `nis_ema_alpha` | 0.2 |
| `pitch_deg` | 0 |
| `reject_contained_lights` | 1 |
| `tracking_thres` | 3 |
| `width` | 0 |
| `yaw_search_range_deg` | 85 |

## 结论

- 所有影响命中率的连续参数（噪声、开火、滞回、阈值）都可 `--set`，所以自动扫描不会漏项。
- 不支持 `--set` 的多为结构/字符串/枚举参数（模型路径、颜色、类别列表），本就不该在线扫。
- 新增参数时务必同时加 `setParam` 分支，否则它会成为“调不到的暗参数”——A-06 的 `channel_diff_threshold` 就踩过这个坑。

*由脚本从源码解析，改动源码后重跑即可更新。*
