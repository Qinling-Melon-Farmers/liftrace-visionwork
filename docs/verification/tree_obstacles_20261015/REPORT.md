# 起飞点H与树障碍处理：2025留存工程对比当前0928

日期：2026-10-15。当前导航来源为 sakelier/liftrace-controlwork 的 liftrace_board_trial_20260928，fetch后仍为 `91459e1a421f72c1635ada3e834798e815dec6f0`。本轮只读分析；未连接板端、修改运行配置或启动仿真。远端配置不能直接等同现场进程已加载参数。

## 起飞点H

正常整场流程不会因为看到起飞点H就降落。H几何检测默认disabled，只有align_mode=landing才处理图像；控制器只在Land阶段且landing检测开关开启时发布该模式。外部降落回调还要求已有LAND动作、新鲜且晚于本次命令的观测、坐标一致，并限制观测距本次H目标不超过配置的0.6m。起飞点(0,0)与本次终点(8.3,-3.5)约9m，不会被当作同一个合法降落锚点。

搜索时target_memory排除landing_pad；五分类YOLO类别中没有H。这里仍须区分“正确识别的H不会参与投递”和“模型有可能把H误分类为某个投递类别”。后者不能由阶段开关证明不存在；H检测在搜索期间关闭，也不能声称依靠全程H形态识别屏蔽所有误报。

已有真实场地手机视频前0～5秒H/地面区间150帧，v8同源PT在conf>=0.5下无YOLO框；这是该视频片段的负样本证据，不是当前板端所有光照/裁切状态的零误报保证。[原报告](../competition_video_20261015/REPORT.md)

源码：
- [H模式门控](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/vision_ws/src/uav_vision/src/landing_detector_node.cpp#L82)
- [控制器H观测锚点检查](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp#L1383)
- [正式H锚点偏移配置](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/control_base.yaml#L33)

## 对比口径

2025参照为本地保留的 `Desktop_patrol_uav_ws-patrol_planner`，用首次集成源码提交0ddc845b交叉检查核心实现。该目录不是Git管理的比赛当日快照，部分配置与首次导入已不同。因此下表称“留存旧版”，不宣称精确还原2025赛场所有参数。

两年主链都是：MID360 → FAST_LIO去畸变/定位 → /cloud_registered_body → FreeDOM静态点云 → Fast-Planner占用/距离场 → 轨迹/控制。没有接入避障的YOLO树类别、树冠语义分割或完整树形模型；五分类模型的优劣不能直接证明树避障好坏。

| 环节 | 留存旧版 | 当前0928正式/08高位默认 | 意义 |
| --- | --- | --- | --- |
| LIO特征提取 | false；每3个有效点取1，保留通过筛选的普通点 | true；输出表面特征pl_surf，边缘特征pl_corn不合入这条输出 | 算法原来就有，启用方式不同；细枝的实际保留量需要对照点云 |
| LIO定位体素 | surf/map均0.5m | 均0.15m | 这是定位降采样；body发布取feats_undistort，不能说避障点云必定按0.15m切掉 |
| FreeDOM输入范围 | 0.2～30m，相对传感器Z±10m | 最终高位配置0.2～6m，相对传感器Z−1～+3.2m | 正赛远端仍继承板端负载/高位profile覆盖，不能只看competition_freedom里的15m |
| FreeDOM分辨率 | 子体素0.1m、粗体素0.4m | 相同：board_load将voxel_depth覆盖为2 | competition_freedom写0，不是最终生效值 |
| 动态清理 | counts_to_free=6、counts_to_revert=20 | 相同；持续融合、射线清理和更新 | static表示保留静态点，不表示一次建完永久不变 |
| 射线增强垂直扇区 | 配置0～59° | 修正为−7～52° | 更符合MID360标称扇区；不能补齐真实没有回波的位置 |
| 规划地图栅格 | 0.1m | 生成器最终0.1m | horizontal_planner的0.05m会被生成覆盖值替换 |
| 常规障碍膨胀 | XY0.25m、上0.20m、下0.10m | 相同；XY按格取整至3格约0.30m | 膨胀扩张已有点，不能补全漏测树枝 |
| 虚拟柱 | 无当前这套结构选择/向上延伸 | 搜索区默认开启，支持结构中部40%～60%轮廓向上延伸 | 不等于把整棵树最大树冠轮廓拉成实心柱 |
| 轨迹检查 | A*占用检查、样条距离惩罚、周期重规划 | 还在发布前对最终/备用曲线每20ms检查占用与净空，周期检查保留 | 下游检查再严格，也看不到未进入地图的真实障碍 |

旧版路径：
- `Desktop_patrol_uav_ws-patrol_planner/src/FAST_LIO/launch/mapping_mid360.launch:8`
- `Desktop_patrol_uav_ws-patrol_planner/src/FreeDOM/config/mid360.yaml`
- `Desktop_patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_manage/launch/patrol_planner_real.xml:59`

当前关键来源：
- [board_load](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/board_load.yaml)
- [高位FreeDOM配置](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/mapping_high.yaml)
- [配置加载顺序](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/uav_mission/launch/competition_mapping.launch)
- [预处理输出pl_surf](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/FAST_LIO/src/preprocess.cpp#L44)
- [body点云发布](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/FAST_LIO/src/laserMapping.cpp#L554)
- [生成后的规划覆盖参数](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/uav_mission/src/uav_mission/competition_config.py#L220)

## 今年的虚拟柱具体保护什么

普通三维占用对所有传入有限点继续生效。额外柱只处理搜索区内、高于地面0.4m的点；局部15cm邻域至少3点、垂直跨度至少20cm才能形成结构支持。

当前启用middle模式：对每个支持连通结构，用其已观测高度40%～60%带的轮廓作为向上柱源。小结构可在实测轮廓内作有限凸包填充，组件跨度上限1.6m、距实测点插值上限0.35m；没有中部回波或组件太大（例如与墙连通）则回退保留实测支持轮廓，不对整面墙做大凸包。

因此外围低枝仍有自身普通3D膨胀，但不必全部延伸到飞行顶端。**规划从低枝上方通过可能是这套策略允许的结果，不能仅凭“有虚拟柱”判断它一定绕过最大树冠投影。** 未观测的枝叶也不会自动补出来；该策略不是树分类算法。

[柱参数](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/uav_mission/config/horizontal_planner.yaml#L19) · [结构/中部带算法](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/include/plan_env/vertical_obstacle_support.h) · [实体膨胀与柱生成](https://github.com/sakelier/liftrace-controlwork/blob/91459e1a421f72c1635ada3e834798e815dec6f0/patrol_uav_ws-patrol_planner/src/Fast-Planner/fast_planner/plan_env/src/sdf_map.cpp#L945)

## 对真实场地的判断

1. **先查点有没有留下，再查规划是否绕开。** LIO特征提取打开后，边缘/零散回波并非全部进入避障输入；FreeDOM又只输出STATIC点。细枝或晃动树叶有潜在漏留路径，但本轮没有同一棵树的原始/处理后点云对照，不能宣布已经造成某次碰撞，也不直接关闭定位优化。
2. **高位观察存在几何与裁剪限制。** 按当前mapping_imu在FC上5cm，FC2.6m时，FreeDOM新点低界约AGL1.65m。此前低位已经积累的点可能仍保留，不能说低树干全部立即消失；但新露出的低枝可能进不了这次观测。MID360水平安装的标称垂直FOV−7～52°，传感器下方0.5m、水平距离2m的物体约−14°，本来就不在瞬时视场内。相机能俯视看到树，不代表雷达此刻也扫到同一部分。
3. **动态清理不是完美的人/树分类。** 人员离开后需要可见射线更新才能消影；遮挡处不能保证立刻清空。枝叶摆动也可能被视作动态。长时间静止预热只能积累可见部分，不能穿过树冠看背面。
4. **现阶段不能把其他队伍问题归因于同一个原因。** 他们的传感器、安装、过滤和规划可能不同。对本机优先检查原始/lio/freeDOM/膨胀四层，再看实际轨迹与机体净空；不要先单纯改YOLO或盲目扩大全部膨胀。
5. **区分几何占用和未观测空间。** 当前cloud路径以点构建占用，局部重建将无占用位置置零；没有点不是独立的“已观测净空证明”。本轮未改变未知空间策略。

MID360规格及近距离细小/低反射物体限制见[官方规格](https://www.livoxtech.com/mid-360/specs)。厂家明确提醒0.1～1m内低反射、细小物体探测效果不能保证；不可把0.1m标称最近距离当成细枝可靠识别距离。

建议下一次针对同一棵树录制 `/livox/lidar`、`/cloud_registered_body`、`/freedom/static_pointcloud`、`/sdf_map/occupancy_inflate`（以实际remap为准）以及pose/trajectory。地面、低位、高位各对照一次，先明确丢点发生在哪一层，再决定是否为避障保留更完整点云、调整垂直保留范围或修改柱轮廓。LIO并行/大核优化与避障点集完整性可以分别处理，不要求回退全部优化。
