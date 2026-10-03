# ROS1 bag 离线回放小工具

输入一个ROS1 `.bag`，生成相机原片、视觉叠加、俯视轨迹动画、多画面汇报视频以及时序报告。无需启动roscore、RViz、飞控或仿真，也不重新运行检测模型。只读取bag；不发布ROS话题或调用服务。

## 快速使用

在Ubuntu20.04/ROS Noetic或对应WSL内解压工具目录，执行：

```bash
bash run.sh /absolute/path/flight.bag /absolute/path/replay_output --fps 10 --frame map
```

路径可以含空格，但要加引号。输出使用专门的新目录；再次使用同一输出目录会覆盖本工具同名产物。`--frame`为俯视地图的显示坐标系；本项目一般为map或camera_init。Windows先进入WSL终端再执行即可。

依赖：ROS Noetic的rosbag/Python3、FFmpeg/ffprobe；渲染Python需要NumPy、SciPy、OpenCV。不需要PyTorch、GPU或模型文件。作者笔记本自动使用已有rl_drone环境；其他机器通过环境变量选择已有Python：

```bash
REPLAY_PYTHON=/path/to/your/env/bin/python bash run.sh flight.bag replay_output
```

ROS不在默认位置时可设置`ROS_SETUP`；ROS读取解释器可设置`ROS_PYTHON`。不要在系统Python中额外安装ML包。先验证依赖，再运行；本工具不会自动安装软件。

## 产物

| 文件 | 内容 |
|---|---|
| index.html | 本地播放器，可切换四种视频 |
| dashboard.mp4 | 左侧下视与视觉/坐标面板，右侧地图/轨迹与高度速度曲线，1920×1080 |
| trajectory.mp4 | 俯视实际轨迹、最后记录规划曲线、设定点轨迹、目标与高度速度动画 |
| camera_annotated.mp4 | 检测框、几何、投影、任务、许可及目标坐标 |
| camera_raw.mp4 | 原始相机画面，缺相机话题时不生成 |
| REPORT.md / summary.json | 数据范围、缺失话题、坐标系限制、分类检测/候选计数 |
| events.csv | 指令、执行结果、投递回执事件 |
| motion.csv / target_coordinates.csv | 位姿速度与目标坐标逐条记录 |
| video_frames.csv | 视频帧与原始图像时间映射 |
| validation.json | FFmpeg完整解码与时长检查结果 |
| flight_summary.png | 全程轨迹与高度速度总览 |

默认10fps，按bag时长1倍速重采样，绝不把3113张图强行按固定相机帧率播放导致时间缩短。`--fps 15`可增加帧率。无音轨。视频成功校验后只清理本工具导出的临时相机帧；原始bag不动。若准备重复渲染，使用`--keep-frames`。

## 话题兼容

默认适配本项目四套板端专项的视觉、任务、MAVROS、TF、规划Marker和膨胀点云。bag的自定义ROS消息由rosbag内嵌定义读取，无需复制板端整个源码工作区。

不同话题名称用JSON覆盖：

```json
{"camera":"/camera/image_raw/compressed","odom":"/mavros/local_position/odom","trajectory":"/planning_vis/trajectory","cloud":"/sdf_map/occupancy_inflate"}
```

```bash
bash run.sh flight.bag output --topics topics.json
```

当前相机入口支持CompressedImage，点云支持float32 x/y/z。优先Odometry，缺失时退回PoseStamped。规划曲线使用Marker的LINE_STRIP/LINE_LIST；仅有Bspline而无Marker时暂不重建，报告会说明缺失。缺失视觉或任务话题显示未知，不伪造结果。特别古老的bag可以生成有记录部分的轨迹/相机回放，不保证具备新视觉面板。

## 阅读回放时的含义

- 绿色是类别检测，青色是几何，黄色是地图/精修拒绝，紫色是有效投影。按图像时间戳30ms内离线匹配；不把前一帧框无限留到当前帧。检测器抽帧与相机不同步时会闪框，不能据此认定掉检。
- 任务状态按录包接收时间；检测框离线回贴到源图像，因此可能先于当时在线任务响应出现。不要用该视频量算未经校正的在线延迟。
- 坐标显示class、id、XYZ、frame、CURRENT/HISTORY与年龄。CURRENT只表示最近候选仍报告map_valid，不等于已允许投递。HISTORY是最后有效记录位置，不会为没有解算成功的panzer虚构XYZ。
- TF按记录时间查询，动态TF最多使用0.5秒旧样本，不插值；未知变换会略去地图元素并记录警告，不把不同坐标系强行叠到一起。原坐标文本仍显示原frame。
- 青色轨迹是定位结果，不是外部真值；粉色是控制设定点，橙色是最后记录的规划Marker。灰色点云是稀疏膨胀图的当前高度±0.3m切片，不能代替精密碰撞距离分析。
- 高度是显示坐标系Z，非直接测距AGL。速度由0.5秒位置差分得到，定位跳变可能产生尖峰；不要将机体系twist的z分量直接解释成升降速度。
- 泛化圆环circle不等于装甲车panzer已确认。现有精修会借用圆环中心，同时仍保留circle辅助记录，因此可能出现circle与类别两个ID；这不等于两个可投递靶。selected_target也不等于任务层已接纳该目标。
- 具体模式以视频中的vision字段为准：landing模式会过滤非H；disabled仍开放搜索感知，不能误读为相机/检测关闭。

## 验证范围

已用2026-09-25 109.10秒实飞bag端到端导出四种视频并检查；通用性测试覆盖时间匹配、静态坐标旋转/逆变换、未知TF及动态过期。另已用2026-09-26 19:12的101.46秒实飞包生成并验证四种回放，报告见`docs/deployment/flight_review_20260926/REPORT.md`（仓库根相对路径）。尚未在队友不同发行版、ROS2或只有原始Image的bag上验收。

分享时可只发`dashboard.mp4`给观看者；发本工具目录给分析者。完整输出目录可连同index.html一起压缩，浏览器本地打开即可。原始bag、data.json及大视频不提交Git。
