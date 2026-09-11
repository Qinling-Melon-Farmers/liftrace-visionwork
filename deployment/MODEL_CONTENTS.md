# 模型与YAML位置

2026-09-11核对实际R64两包后修订。正式入口所需YAML没有漏打；此前runtime_models只有权重，类别元数据位于ROS包的config目录。为避免单独拷贝权重时漏掉元数据，修订包在runtime_models另附一份同源元数据。

| 文件 | 用途 |
|---|---|
| runtime_models/merged_standard_fp32.rknn | 机载包的RK3588/NPU权重 |
| runtime_models/merged_standard.pt | 仿真包的笔记本YOLO权重，checkpoint自带模型结构与类别 |
| runtime_models/merged_standard_6cls_metadata.yaml | 修订包附带的六类顺序、640×640输入和预处理描述；便于单独拷模型 |
| vision_ws/src/uav_vision/config/merged_standard_6cls_metadata.yaml | 正式ROS板端入口实际默认读取的元数据，内容与上份相同 |
| vision_ws/src/uav_vision/config/target_detector_rknn.yaml | RKNN输入/推理阈值运行配置 |
| vision_ws/src/uav_vision/config/target_detector.yaml | 仿真YOLO推理配置 |
| vision_ws/src/camera_sdk/param/calibration_1280x720.yaml | camera_calibrated_1280x720.launch实际读取的相机标定 |
| patrol_uav_ws-patrol_planner/src/uav_mission/config/ | LIO/任务/控制/规划配套YAML |

六类顺序为0 bridge、1 panzer、2 pillbox、3 tent、4 tank、5 red_cross。正式比赛r2026仍按当前规则过滤tank；不能自行重排names。FP32选用float32/RGB/NHWC、normalize=true，与已选择模型配套。

板端入口链为competition_hardware.launch→phase_d_board.launch→config/merged_standard_6cls_metadata.yaml。两个旧视觉包装入口board_camera_vision.launch/control_handoff_board.launch的metadata路径已改到config，并要求显式传model_path，避免默认指向包内不存在的uav_vision/models权重。若只拷.rknn到其它工程，必须同时携带metadata并正确指定；PT推理不另需训练用data.yaml或架构yaml。

Gazebo模型位于仿真包的vision_ws/src/uav_vision_eval/models与simulation_assets/models：按入口需要SDF/world、model.config、网格和材质，并不要求每个模型有yaml。有些机架直接用-file加载SDF，不依赖模型浏览器的model.config。修订仿真包明确附带optional_models/D435i、fpv_cam、tanke供历史对照；它们不进入默认今年场景。

机载包不含Gazebo世界/飞机模型，这是部署范围选择。PX4及Gazebo插件、MID360插件和扫描CSV、板端RKNN runtime及设备驱动仍是外部环境依赖，包不是完整系统镜像。

历史patrol_full_competition_sim.launch的默认patrol_toudi4.yaml是旧配置名，当前精简树不保留该配置。正式入口已经显式覆盖waypoint_config，运行所需文件存在；不要将这个旧launch不带参数裸启动当作当前交付入口。

旧092f63d8压缩包保留，不回写其源码版本或历史结果。修订包只修路径和随包资源，包含已有五seed报告，不代表修好了近地/随机门/搜索失败，也没有新仿真或实机验证。
