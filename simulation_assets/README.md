# 2026仿真资源

当前机架、场地的工程SDF在`vision_ws/src/uav_vision_eval/models`；本目录冻结其需要的通用iris/GPS、MID360网格、树/箱外观、起降H及五个正式靶标的外部资源，换机时无需依赖本机Astra模型目录。几何和贴图保持本次静态截图使用版本，不在打包时重画或缩放。

模型名称与类别：zhangpeng=tent、dibao=pillbox、qiaoliang=bridge、zhuangjiache=panzer、red_cross=red_cross；landing_h用于两处H。tank不属于今年正式五靶场景。sun和ground_plane也随包提供。箱体为world中的内嵌box，不另依赖bigbox。

来源与大小见MODEL_SOURCES.json，上游许可证随licenses目录保存，单模型原有说明继续保留。本目录是用户要求的必要仿真资源例外（最大MID360 DAE约31MiB），不包含权重、bag或飞行日志。原始MID360其它未引用机架网格不复制。D435i等历史对照模型只在本地仿真包的optional_models中提供，不加入本竞赛当前资源目录。

PX4、ROS/Gazebo、MID360 Gazebo插件及其扫描CSV仍是运行依赖，不把模型资源包称为完整操作系统镜像。见docs/BUNDLES.md。
