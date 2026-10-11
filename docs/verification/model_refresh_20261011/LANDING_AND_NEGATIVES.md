# YOLO 降落门控与 H 负样本核对

日期：2026-10-11。B 工作树：`/home/xhj/liftrace-worktrees/r2026-board-vision-tests`（核对 HEAD `983330de`）；对照工作树：`/home/xhj/liftrace-worktrees/r2026-board-frame-fix`（`4e774026`）。本说明依据本地当前源码及数据文件，只读核对期间未访问板端、未启动仿真；不代表板端实时节点或参数已核验。

## 降落时 YOLO 是否关闭

两树行为一致：**收到 `/uav_vision/align_mode=landing` 后，YOLO 推理暂停，节点及已加载模型保留；离开 landing 后恢复推理。**

- `vision_ws/src/uav_vision/launch/phase_d_board.launch:30`：两树均启动 `target_detector_rknn`，`required=true`。
- `vision_ws/src/uav_vision/config/target_detector_rknn.yaml:37`：两树均订阅上述模式话题，默认模式为 `disabled`，第 39 行 `pause_in_landing_mode: true`。
- `vision_ws/src/uav_vision/src/uav_vision/detector_stage_gate.py:18`：仅 `landing` 置暂停；第 25 行 `begin()` 在暂停时返回 `None`。
- `vision_ws/src/uav_vision/scripts/target_detector_rknn.py:580`：图像回调在图像转换及推理前直接返回；第 594 行丢弃模式切换前已开始推理的结果。已开始的一次推理不会被强制中止。
- 整机两树均由 `patrol_uav_ws-patrol_planner/src/uav_mission/launch/competition_application.launch:24` 接入此链；08 由 `deployment/board_trials_4x4/common/uav_board_trials/launch/application.launch` 接入，B 为第 25 行，对照树为第 21 行。
- 两树 `patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp:3512` 在接受外部 LAND 后进入 `Land`；第 3536 行要求 `Drone_mode == Land && flag_landing_detect` 才返回 `landing`，第 1362 行发布模式。整机配置生成器 `competition_config.py:194` 开启 H 检测；08 的 `trial_config.py` 对 H 降落开启检测，B 第 258 行、对照树第 257 行（两文件均位于各自 `uav_mission/src/uav_mission/`、`deployment/board_trials_4x4/common/uav_board_trials/scripts/`）。

因此，投后路线、过门和返航途中不能统称 YOLO 已关闭；`disabled` 仅是对准模式，仍允许 YOLO 推理。H 几何检测另由 `landing_detector` 承担，见 `vision_ws/src/uav_vision/src/landing_detector_node.cpp:84`、`:96`。

## H 负样本数量与路径

实际读取以下两套数据集的 manifest 和对应 label：

- v6：`/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v6_5cls_flight_h_20260928/`
- v7：`/home/xhj/liftrace/vision_ws/test_data/yolo_dataset_v7_5cls_pillbox_20261011/`

| 明确标记为 `sim_H_negative` 的数据 | v6 | v7 |
| --- | ---: | ---: |
| 训练原图 | 5 | 5（继承） |
| 训练图，含光照增强 | 80 | 80（继承） |
| 测试原图 | 6 | 6（继承） |
| 验证集 H 负样本 | 0 | 0 |
| 对应 label 存在且为 0 字节 | 86/86 | 86/86 |

训练来源是四张仿真视角及一张标准 H 纹理：

- `/home/xhj/liftrace-worktrees/r2026-high-view-search/logs/high_view_render_seed32_20260913_225236/frames/h{14,24,26,28}_v02_f0.png`
- `/home/xhj/liftrace/simulation_assets/models/landing_h/materials/textures/Hjiangluo.png`

测试来源是同一渲染目录的 `h{14,24,26,28}_v07_f0.png` 四张，以及 `/home/xhj/liftrace-worktrees/r2026-coverage-efficiency-research/logs/coverage_seed32_reference070_20260911_174716/coverage/0002.png`、`0007.png` 两张；这批明确标记的 H 负样本来源为仿真/标准纹理。

来源清单位于对照工作树 `docs/planning/panzer_five_class_20260928/annotations.json:3198`（训练视角）、`:3242`（测试视角）、`:3286`（历史仿真帧）、`:3308`（标准纹理），各条均为 `objects: []`。对照树 `tools/model_finetune/prepare.py:34` 写入空 label，第 57 行为每张训练 H 原图增加 15 个光照变体，故训练共 `5 × 16 = 80` 张。B 的 `vision_ws/scripts/import_pillbox_video_v7.py:83` 以符号链接继承 v6 图像与 label。

空 label 实例：两套数据集下均有 `labels/train/H_v2_z14.txt`、`labels/train/H_standard_asset.txt`、`labels/test/H_v7_z14.txt`，已确认文件大小均为 0 字节。H 是五类投递检测器的背景负样本，不是额外 YOLO 类别。实际训练参数引用分别见 `/home/xhj/liftrace/vision_ws/runs/liftrace_5cls_flight_h_20260928/candidate/args.yaml:4` 和 `/home/xhj/liftrace/vision_ws/runs/liftrace_5cls_pillbox_20261011/candidate/args.yaml:4`。

本次仅新增本独立说明，按用户要求不改共享联调记录、不提交。
