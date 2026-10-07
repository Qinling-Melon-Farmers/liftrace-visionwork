# H低位POSCTL交接专项结果

日期：2026-10-08。运行：`logs/seed38_h_posctl_targeted_20261008_050724`；manifest源码HEAD：`33a521a8d1c627e40cfce0b620683977af91521f`。

**低位POSCTL交接真实成功；随后在中杆悬停期间，既有状态保护拒绝5ms未来源戳并取消任务。整轮不记PASS，未验证飞手下降或触地。** 主代理统一停止；包装器exit=0、cleanup PASS且零残留只表示正常收尾，不替代任务验收。

本轮仅Gazebo/SITL专项通过临时launch显式启用中杆，输入`x=y=r=0、z=500、buttons=0`，bag共5017条；未改飞控参数或板端。生产源码、共享REPORT和台账保持冻结，本节不扩展运行时修复。

## 交接物理指标

以下从关闭后的`slot_h_evidence.bag`离线提取，与主代理`handoff_physical_quick.json`一致。**ground_z=-0.22m，坐标系camera_init，FC AGL=FC本地Z-ground_z**；该AGL不是Gazebo真值高度或相机离地高度。

| 事件及源事件时间 | FC本地Z | FC AGL | 到冻结H的XY误差 | 水平速度 | 垂向速度 |
|---|---:|---:|---:|---:|---:|
| REQUESTED 36.598s | 0.145206m | 0.365206m | 0.008631m | 0.018279m/s | -0.016586m/s |
| OBSERVED 36.783s | 0.151078m | 0.371078m | 0.008338m | 0.014126m/s | +0.021360m/s |

冻结参考为控制器锁点后的低位指令XY `(-0.030472870916, -0.056466769427)`，来源`/mavros/setpoint_position/local`，不是视觉真值靶心。FC位置和速度来自同系`/mavros/local_position/odom`；twist按child解释，使用同条完整姿态旋转到父坐标系后计算水平速度及有符号垂向速度。每个事件选其之前最后一个源样本：REQUESTED使用源36.592s（bag36.595s，年龄6ms）；OBSERVED使用源36.767s（bag36.766s，年龄16ms）。没有插值。

请求时满足原0.37m上限；真实回执约185ms后，AGL为0.371078m，比上限高约1.08mm。回执高度不能当作请求判据时高度，两者分别记录，不据此放宽0.35/0.37m与±0.02m条件。

## 事件与后续取消

| 事件 | payload事件时间 | bag记录时间 |
|---|---:|---:|
| REQUESTED | 36.598s | 36.598s |
| OBSERVED | 36.783s | 36.784s |
| CANCELLED | 44.786s | 44.786s |

bag有27条真实POSCTL状态，首条在36.784s，connected/armed均真。高位22.348s锁点，日志计数114帧：视觉计数已提前累计，当前运动窗通过才冻结；首次capture settling为14.347s，等待8.001s。与旧045333等待22.300s相比本轮较短，但两轮波动不能全部归因于条件迁移。

44.786s控制器报`handoff_state_not_ready_or_stale`并发布CANCELLED；44.796s任务层以`landing_failed`发ABORT，PX4仍保持POSCTL。该取消发生在OBSERVED之后8.003s，不是2.5s模式转换等待超时，也不是ASSERT拒绝手动交接。

| 邻近POSCTL状态的bag记录时间 | header源时间 | bag时间减源时间 |
|---|---:|---:|
| 41.786s | 41.786s | 0ms |
| 42.782s | 42.782s | 0ms |
| 43.784s | 43.779s | +5ms |
| 44.786s | 44.791s | **-5ms** |
| 45.784s | 45.783s | +1ms |
| 46.788s | 46.791s | -3ms |

上述状态全部connected=true、armed=true、mode=POSCTL。44.786s失败日志和CANCELLED事件的时刻，与header44.791s共同指向源age约-5ms，命中既有负age保护；不是超过`state_max_age_sec`默认2.5s的陈旧状态。控制器callback/tick两处保护位于`patrol_control.cpp`第543～549、636～646行，git blame均来自既有`c182dafbf`；`33a521a8`控制源码diff仅迁移高位视觉计数/运动窗的两处if条件，未修改这些保护。本轮归档为原有SITL微超前时间戳边界，不归为新并行累计分支回归。

**记录限制：** bag记录时间不等于控制器内部接收时间。ROS多个订阅者的clock/header更新顺序可能使bag时间与header出现小幅负差；不能据此精确复原控制器receipt_age。本轮同时有失败日志、CANCELLED事件和超前状态样本，因此可以定位负源age边界；没有记录内部接收时刻，JSON的controller_receipt_age_sec保留null，不编造接收年龄或网络延迟。

## 独立产物与范围

- 原bag、run.log、manifest和PX4 ULog保留在上述run目录。
- `h_posctl_manual_result.json`：交接/取消全事件、两时刻物理指标、邻近状态时间、范围与收尾结论。
- `h_posctl_physical_metrics.json`：详细源样本、姿态旋转后的速度与冻结参考；主代理`handoff_physical_quick.json`为独立快速核对。
- 离线脚本：`/tmp/seed38_slot_h_20261008/summarize_h_posctl_bag.py`及`finalize_050724_result.py`，只读关闭后的bag，不初始化ROS、不发布输入。

本轮验收口径仅为H中心低速下降后真实切入POSCTL；交接后持续状态监护的原有SITL时间边界明确保留。没有模拟飞手油门下杆，没有触地验收，没有上板，也不把包装器收尾成功或交接子阶段成功等同整轮任务PASS。
