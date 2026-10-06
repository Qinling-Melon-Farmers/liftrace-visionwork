# FAST-LIO 快照集成边界（2026-10-06）

EV 来源 `feat/ev-continuity-20261004@69216afdafd65df9b298fb7c18e7d1a84eeca31f`；比较基线为试飞树 `7a7cb6c0a4b7d3b629c82ea238c35bfe9b11300d` 的工作区文件。`comparison.json` 按文件列出比较结果，`full_fast_lio_comparison.diff` 保留原始全文差异，不可整包覆盖。

**主代理已于本轮集成下面4个文件，已完成第二轮ARM快照增量构建及10项生产匹配回归（workers=3）。补丁已冻结，不再刷新或重放。** 本套件代理没有修改这些包文件、执行构建或连接板端。

| 精确文件 | 最小差异 |
|---|---|
| `patrol_uav_ws-patrol_planner/src/FAST_LIO/msg/PredictionState.msg` | 新增完整状态：真实scan末stamp、IMU frame、epoch、valid、姿态/位置/速度、偏置/重力、18×18协方差、过程噪声、加速度缩放与IMU时间偏移 |
| `patrol_uav_ws-patrol_planner/src/FAST_LIO/CMakeLists.txt` | 登记PredictionState.msg及generate_messages的std_msgs依赖；保留试飞树diagnostic_msgs导出与线程配置 |
| `patrol_uav_ws-patrol_planner/src/FAST_LIO/src/IMU_Processing.hpp` | 增加prediction_accel_scale()、prediction_initialized()两个const读取接口 |
| `patrol_uav_ws-patrol_planner/src/FAST_LIO/src/laserMapping.cpp` | 新消息include及状态；雷达/IMU时钟回退使快照invalid；构造完整快照；私有参数prediction_state_enabled默认false；Odometry之后在主线程发布 |

`fast_lio_snapshot.patch` 是这4个文件的精确增量。它不改MP_EN、MP_PROC_NUM、FAST_LIO_MATCH_THREADS、OpenMP等待/绑核，不改ICP、滤波更新、正式EV桥或MAVROS参数，不新增异步回调/共享滤波线程。快照在匹配/滤波校正完成后由原主线程读取，故与现有显式三线程匹配兼容。开启快照会增加状态构造、协方差投影和ROS序列化工作，板端时效/负载增量尚待实测，不能承诺零开销。

默认false仅禁止快照消息发布；publisher仍可能注册，且订阅者为0时跳过构造。时钟回退后invalid不代表旧滤波器完成了重建。不要重复旧位姿改stamp或从旧Odometry补零生成快照。

试飞树与EV树的preprocess.cpp/preprocess.h、local_map_shift.h、匹配容器/测试及其余已存在include相同；这些修复已经进入试飞树，无需再覆盖。EV原始CMake删除了diagnostic_msgs export且登记了包内观察脚本；本最小补丁均未采用。Python观察脚本已由本套件vendored，不要求安装回FAST_LIO包。package.xml的NumPy新增属于Python观察依赖，不是新增C++生产者的必需差异。

## 同一个低空定位入口

主代理已修改 `deployment/low_hover_observation/localization.launch`，采用的实际参数名为：

```bash
# 仅供之后明确获准的地面定位/采集轮次；本轮未执行。
bash deployment/low_hover_observation/start.sh localization hover prediction_state_enabled:=true
```

默认false；将私有prediction_state_enabled传给既有laserMapping节点，不新增第二个LIO。不通过运行中rosparam set尝试启用，也不在空中重启LIO。默认prediction_imu_frame来自源码livox_frame，采集前应与/livox/imu的header.frame_id核对，不一致时由主代理补可配置参数。

`low_hover_prediction_optional.patch` 是主代理集成前准备的另一种参数命名草案（enable_prediction_state）；已被当前入口替代，保留来源，**不得再应用**。

## 并行核查

主代理报告第一轮ARM全部包编译成功，105项专项中103 PASS、2 SKIP；flags有MP_EN/MP_PROC_NUM=3/-fopenmp，链接libgomp。上述由主代理检查，本套件代理未连接板端独立复核。

编译宏证明确实启用了OpenMP匹配路径，单独libgomp不能证明。现场还须确认正在运行的laserMapping实际来自0928新二进制，不能把另一目录R64运行当成该构建的证据。`parallel_check.py`只读已有flags/cache、生产matching测试日志和实际入口日志；缺运行证据时输出runtime_parallel_confirmed=false，使用--require-runtime返回非零。

第二轮应保留 `-DFAST_LIO_MATCH_THREADS=3`，重编fastlio_mapping、生成消息并执行matching_buffers_test等本轮相关回归；构建/运行由主代理负责，本套件没有构建或LIO启动命令。
