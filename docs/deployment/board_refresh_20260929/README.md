# 2026-09-29 实机部署更新（未开始试飞）

香橙派SSH：`orangepi@192.168.43.59`。继续使用`/home/orangepi/liftrace_board_trials_20260928`，没有覆盖试飞组的`liftrace_r64_onboard_405bda42`。部署revision以板端`DEPLOYMENT_MANIFEST.json`为准。

## 已部署

- 定位静置/飞控与LIO一致后才启动空白FreeDOM，地图新鲜且完整视觉链就绪后才READY，避免收敛前错误观测累积进入导航。
- 支持持续观测位置优先重访与后续竞争位置复核；完整高位结束及提前中断的条件继续分开。
- 第09组高速采集，现场快捷`capture`/`6`，2m高度、0.5/1m/s两档、两次直线往返、无投递。其余五组保留0.5m/s。
- 现场前方6m/左右±1.5m、高位环线内收至5.5m/±1.1m、已知外参、2m上限、30cm末端悬停、人工解锁后的原自动流程继续继承。静态TF、虚拟顶棚关闭、25/20/10cm膨胀、现场障碍柱关闭与三维避障保持。保留雷达专用MID360配置及模型权重。
- 高位粗门槛仍0.60、独立圆环门槛仍0.80；昨夜0.70/0.75是离线对照，没有擅自改为本轮默认。

## 板端只录bag

硬件application只启动轻量`trial_journal.py`，不导入OpenCV、不订阅相机像素、不维护图像队列或视频编码。`run_trial.py`正常结束调用`finish_trial.py`，只写结果和bag索引，不调用ffmpeg。即使手动传record_camera_video=true，硬件入口也不会启动笔记本专用录影节点。

当前板端专项运行目录删除以下文件及旧devel包装器：

- `scripts/trial_recorder.py`：原始/叠加MP4。
- `scripts/finish_recording.py`：旧视频转码收尾。
- `scripts/render_trial_replay.py`：多画面合成。

以上三个脚本仍保留在本机用于既有SITL录像和事后处理，CMake只在文件实际存在时安装旧仿真录影脚本；后续板端打包继续排除这三项。已运行日志、bag、原有相机发布驱动与相机标定不删除。旧源码仅打包留在部署回退归档，未保留为可启动录像入口。

图像仅通过`TrialBag`录入压缩相机话题；CameraInfo、TF、视觉全链、记忆、实际/规划/设定点、任务/释放和末端悬停状态仍录入bag。保留`vision_events.jsonl`、`navigation_pose.csv`等小型诊断文件用于任务结果判断，`result.json`及`index.html`包含bag索引；不再产生camera_frames.csv或新的相机MP4。下载后用本机`tools/bag_replay`生成录像和叠加。

## 检查结果

- 板端uav_board_trials、uav_high_view、uav_mission构建通过；没有本轮C++导航算法改动。
- 板端58项专项测试、26项高位记忆测试、13项粗线索测试通过。
- 九组preview/flight共18套应用和配套定位/地图配置离线展开通过。现场原五组及高速0.5/1两档真实shell入口--check-config通过；04/08的真实航点仍留空，不能把测试展开的样例当现场航点。
- 本机58项专项检查与原八组SITL入口离线展开通过；未启动新仿真。
- 板端未运行rosmaster/roslaunch/MAVROS/patrol_control/PWM。此次未启动preview、flight、相机、舵机，未发送解锁、模式切换或任务开始。
- 板端时钟原落后约14小时，已在无ROS运行时按笔记本校准；NTP仍开启。RTC原始日期异常，下次冷启动需确认时间重新同步。

板端记录：`deployment_results/refresh_20260929/`，替换前源码为`replaced_source_before.tar.gz`；本机归档：`logs/deploy_refresh_20260929/`。检查不代表实飞验收；下一步由现场明确启动后再验证净空建图、高位重访和高速采集。


## 第三组地面初始化修复

部署后用户授权从第三组memory_only开始，确认地面未解锁、机头朝+X。先启动设备输入，再启动专项；现场沿用高位2m、0.5m/s、末端0.3m悬停人工降落、不投递、bag记录。

两次启动均在READY前停止，尚未起飞：

1. `board_memory_only_20260929_155643`：FC/LIO已稳定且新地图就绪，应用批量启动时LIO时间戳短暂落后超过0.3s，原初始化检查立即退出。改为在原30s地图/60s应用就绪期限内等待传输恢复，重新满足连续2s稳定才READY；不放宽0.3s新鲜度、0.2m位置和5°航向门槛。未来时间戳、真实几何不一致、非静置未解锁等仍终止。没有飞行中清图。
2. `board_memory_only_20260929_160435`：上述短暂延迟已进入等待恢复，应用因末端悬停节点ImportError退出。`trial_terminal_hover`导入了catkin devel的`trial_auto_land`包装脚本，而包装脚本不导出函数。现在优先从源码相邻目录导入公共判断函数；清除同一文件中三段完全重复且不可达的接管分支，不改变接管行为。

本机和板端22项回归通过，包含延迟恢复重新计时、真实不一致仍拒绝、catkin包装路径遮蔽、末端高度限制和人工接管保持。板端真实devel包装脚本无节点启动的导入检查通过。修改仅涉及Python启动和导入，不改巡航/规划/投递阈值；随后重启第三组验证实际READY。首两次bag已正常关闭并保留，不能算作飞行验收。

第三次启动`board_memory_only_20260929_160721`已通过真实地面READY：FC/LIO位置差约0.004m、航向差约2.14°，新地图33帧、视觉全链和控制设定点就绪；bag记录中。仍待飞手手动解锁，地面READY不等于整圈验收。
