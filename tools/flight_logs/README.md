# 飞控 ULog 临时取回工具

`fetch_px4_ulog.py` 只读已封闭的PX4日志，供本轮飞后诊断使用。要求ROS/MAVROS已运行、飞控connected且未解锁；不启动任务、解锁、舵机或仿真，不修改飞控参数，不清除飞控日志。当前PX4使用SDLOG_MODE=0，已在解锁时录制，无需为本轮再改变logger设置。

在机载工程根目录执行：

```bash
source deployment/site_20260928/environment.sh
python3 tools/flight_logs/fetch_px4_ulog.py --list-dir /fs/microsd/log
python3 tools/flight_logs/fetch_px4_ulog.py --log-id 781 --output /tmp/flight_781.ulg
```

781仅为2026-10-03 17:14这一轮的实际索引，不是固定“最新日志”。需先通过MAVROS的`/mavros/log_transfer/raw/log_entry`或地面站日志索引确认要取的编号。不要全量请求数百条历史索引来与下载竞争串口。文件日期可能使用错误的飞控时钟，取回后必须用解锁、模式变化、飞行时长及高度曲线匹配ROS bag。

下载使用LOG_REQUEST_DATA，按90字节分片重组，有界窗口、错ID/偏移/短片拒收、重复与丢片重传、ULog头/长度检查。超时、收到解锁状态、通信失效或人工中断会停止取回并保留`.part`；同名文件不覆盖，重新执行用新的输出路径。结束会发送LOG_REQUEST_END，解除取回模式。此工具不是飞行中的旁路实时录制器。

10月3日现场的FTP Open返回正确8,460,479字节，但Read返回空数据；因此没有继续使用FTP读文件，FTP只用于列目录。改用日志传输后，ID781的8.46MB ULog约30.23秒取回，且现有pyulog分析器成功解析。离线数据重组检查覆盖乱序、重复、错ID、错偏移、截短、末尾短片与目录范围；真实下载验证没有触发执行机构。

回传笔记本后使用既有环境分析：

```bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
python tools/bag_replay/analyze_px4_ev.py /path/to/flight.ulg --output /path/to/summary.json
```

不把日志中的计数变化单独当根因结论；继续核对当时融合状态、选中的EKF实例、输入时效以及日志丢失。详细步骤见[排查流程](../../docs/deployment/board_redeploy_20261001/LIO_PX4_DIAGNOSTIC_WORKFLOW_20261003.md)。

## 2026-10-06：低空电机观察

专用的0.6m悬停、分段前移、四边形入口及无图像诊断录制见[低空观察手册](../../deployment/low_hover_observation/README.md)。ULog工具扩展见[电机/重置查看器](ulg_motor_viewer/README.md)。本地准备不代表已部署或验收，不自动解锁、切模式、投递或降落。
