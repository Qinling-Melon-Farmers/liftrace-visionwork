# R64 ROS计算图 · rqt_graph Nodes only

来源：`logs/r2026_r64_px4_seed11_20260909_151207/ros_system_state.json`，随目录保存[注册快照](ros_system_state.json)。这是R64固定seed11完整37/37 PASS运行记录；飞行源码1983b2f7571cf3d96916d45af5cc8f3a2f8a9ad4，[飞行报告](../../verification/r64_seed11/REPORT.md)。本次仅离线重绘，没有启动ROS Master、仿真或实机。

实际使用ROS Noetic的`rqt_graph.dotcode.RosGraphDotcodeGenerator`，选择`NODE_NODE_GRAPH`，由`qt_dotgraph.PydotFactory`和Graphviz生成DOT/SVG。椭圆为注册节点，连线文字为ROS话题，箭头为发布者→订阅者；同一话题可对应多条边。

| 视图 | 节点 | 边 | 相连话题 | 范围 |
|---|---:|---:|---:|---|
| [全图](rqt_graph_nodes_only_full.svg) | 36 | 207 | 57 | 所有注册节点及相接的发布/订阅边，包括仿真、评测、两路录像、日志 |
| [核心](rqt_graph_nodes_only_core.svg) | 22 | 63 | 34 | 主要感知、任务、规划、控制和许可话题；保留Gazebo作为实际传感器发布者；移除评测/录像/启动辅助 |
| [仅飞行](rqt_graph_nodes_only_flight.svg) | 23 | 75 | 37 | 实际运行链节点筛选，保留TF；排除Gazebo、评测、录像、mock和启动辅助，以及clock/rosout边 |

[三图切换与SVG入口](index.html) · [筛选清单](graph_manifest.json)。SVG可放大；全图广播边较多，宜用核心图阅读主流程。

仅飞行图是SITL快照子图：不虚构本轮未注册的相机SDK、Livox驱动、机械舵机节点。PX4并非此处的ROS节点，MAVROS至PX4的MAVLink链不伪装成ROS话题。`/Servo`与`/legacy/Servo_raw`是服务，不画成话题边；完整服务注册在JSON，调用关系见[接口](../../INTERFACES.md)。注册连接不证明频率、质量或板端资源余量。

相比R60：全图新增`/camera_video_recorder`、`/overview_video_recorder`两个记录节点；仅飞行仍23节点/75边/37话题。本次核心图排除评测/记录/启动节点，并补入LIO→FreeDOM的`/cloud_registered_body`，不能与旧“仅按话题筛选”的核心图直接比数量。没有新增飞行协议。

复现（仓库根目录，仅离线）：

```bash
source /opt/ros/noetic/setup.bash
QT_QPA_PLATFORM=offscreen /usr/bin/python3 top_level_scripts/render_recorded_rqt_graph.py \
  docs/topology/r64/ros_system_state.json docs/topology/r64 --project-root "$PWD"
```

旧图保留在历史报告原路径，当前文档入口更新到本目录。

## PNG / JPEG导出

2026-09-09从同一rqt DOT导出，144dpi、白色背景；未改节点、话题或连线。文字图优先PNG，JPEG便于通用查看。全图较宽，下载原图后放大阅读。

| 视图 | PNG | JPEG |
|---|---|---|
| 全图 | [下载](rqt_graph_nodes_only_full.png) | [下载](rqt_graph_nodes_only_full.jpg) |
| 核心 | [下载](rqt_graph_nodes_only_core.png) | [下载](rqt_graph_nodes_only_core.jpg) |
| 仅飞行 | [下载](rqt_graph_nodes_only_flight.png) | [下载](rqt_graph_nodes_only_flight.jpg) |

渲染脚本现会同时生成SVG、PNG和JPEG；分辨率与解码检查见[raster_manifest.json](raster_manifest.json)。
