# 9月28日晚板端试飞回放

复用 tools/bag_replay，将各轮原始分卷按时间拼接后离线生成：原始相机、视觉叠加、轨迹动画、多画面汇报。10fps、1倍速，不重跑检测，不发布ROS控制。原始分卷保留；视频通过完整解码与时长检查后删除临时合并bag。输出位于本机logs，不上传Git。

## 回放入口

- [board_low_multi_20260928_221154](../../../logs/site_download_20260928/board_low_multi_20260928_221154_replay/index.html)
- [board_high_view_20260928_230146](../../../logs/site_download_20260928/board_high_view_20260928_230146_replay/index.html)
- [board_memory_only_20260928_224603](../../../logs/site_download_20260928/board_memory_only_20260928_224603_replay/index.html)

## low_multi

完整录包206.25秒，包含任务前准备。任务SEARCH到结束LAND指令约76.74秒。红十字槽1、panzer槽2均记录raw_actuator_ack，任务终态COMPLETE，2槽COMMITTED；这证明软件收到机构回执，不单独证明实物落点准确。

约任务34.50秒收到红十字释放回执，73.78秒收到panzer释放回执。阶段间发生RESUME，连续两投链路成立。原result仍标INCOMPLETE，因为此前结果汇总依赖auto_land交接，而现场已改为30cm悬停、人工降落；保留原结果，不把它直接改成PASS。

## high_view

完整录包213.06秒。任务启动后约18.93秒ABORT；首因是SURVEY_NO_PROGRESS、board_full_circle_incomplete。目标点(3.21473,-1.10994,1.74060)被规划器报告in_map=1、inflated=1，0.15m邻域无无碰撞替代目标。约8秒无进展后高位整圈专项结束，未进入低空重访、未投递。这不是舵机失败，也不是因为三类识别不齐而直接终止。

障碍柱已关闭，仍存在目标占据。需进一步核对该坐标处FreeDOM原始地图、膨胀图和TF；当前日志不能把占据直接认定为现场实体障碍或虚假地图。ABORT后仍可见旧目标规划重试，需要单独核查任务取消与规划器交接。

高位高度窗口修复后已接受27条粗观测；panzer有两个冲突位置，约(2.31,0.69)和(2.72,-1.00)。尚不能据此判定两个真实目标。trial_memory_count=0不是完全没有粗记忆：整圈未完成，未冻结重访清单，navigation_support仍有观测证据。视觉链已有输出，但本轮不能验收高位—低位重访闭环。

## 录包体积与减负

9月26日旧包101.46秒、270.74MB；今晚low_multi原始三卷206.25秒、1.39GB，均为LZ4。解压消息中FreeDOM点云819MB、占据图219MB、膨胀图266MB，合计约70%；相机532MB。时长和地图记录是主要增量。

本地板端录制默认关闭独立MP4编码，不订阅原始图像、不启动编码定时器，保留状态JSONL/位姿CSV和独立bag。application.launch可用record_camera_video:=true恢复；仿真默认仍录视频。板端同步两次失败，不能称现场已更新，下次通电应同步并检查。没有启动新的飞行。

## 清理范围

仅清理本次site_download_20260928内被排除轮次的bag，以及已完成回放校验后的combined.bag。三轮原始分卷、已有视频、原始板端文件均保留；清单见logs内cleanup_bags.json。

## memory回放与清理结果

最后一次memory为22:46轮，219.10秒，四路视频全部通过完整解码及1倍速时长检查。下载文件大小核对无缺失。共清理冗余bag 8.79GB；保留三轮原始分卷。

## 2026-09-29补充订正

[建图启动与正式视觉链复核](../mapping_startup_20260929/REPORT.md)：memory的target_refiner启动异常退出，并非专项设计上不需要精修；targets仅一条空消息，实录没有confirmed。放宽高度回放保留三类竞争假设，但无冲突可用集合为空，不能把saved集合当有效三目标队列。地图对照见[点云占据报告](../cloud_occupancy_20260929/REPORT.md)。
