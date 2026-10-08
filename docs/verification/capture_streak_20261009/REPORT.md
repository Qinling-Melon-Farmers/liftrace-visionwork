# 冻结前捕获计数衔接（2026-10-09）

原捕获要求保持5个独立新图像、像素偏差≤30px。仅补偿投递的冻结前分支允许：同一缓存图像被重复检查至过期时立即撤销ready和有效证据，但在短间隔内保留此前合格图像计数；新图像和匹配控制反馈均通过原新鲜性／身份／几何检查后才能继续计数。缓存图像不会变成新的计数，也不会被包装成新鲜证据。

`~capture_max_gap_sec`默认1.0秒，可通过原ROS私有参数配置。界限按`当前ROS时间－最后一张已合格独立图像的原始observation_stamp`计算，严格超过1.0秒或时钟回退时清零。这个参数只约束计数历史，不改变原0.5秒图像／反馈新鲜性。重复消息、memory重新发布、定时器和控制心跳不能延长界限；新图像真正偏差越过30px、几何ID或任务事务变化、定位反馈／上下文失效仍重置。

新偏差越界在targets回调立即撤销ready／清零，并推进原单调图像水位，旧反馈不能恢复累计；仍发布原像素偏差用于继续对准。冻结后反馈保持原检查和重置规则，H、释放许可、45秒commit、20cm漂移及slot幂等未改。控制末段严格小于4cm及高度范围以主代理提交`f37b710f`为准，本提交没有修改控制或配置文件。

## 现场依据与时间估计边界

读取B产物`试飞产物/board_full_mission_20261008_231313/local_prefreeze_summary.json`及对应`local_prefreeze_evidence.json`：36次计数归零，34次为observation stale；首次合格在ALIGN后2.464秒，4秒内已到4帧，旧第5帧在23.818秒。摘要的当前观测年龄中位数0.4323秒、P95 0.4945秒，接近原0.5秒期限。这是旧版本的事实，不代表补丁后的确切捕获时刻。

代码中年龄来自目标`last_seen`，DropOffset和控制反馈保留同一原始观测时间；target_memory在检测回调发布，没有等待额外memory定时器。不能通过刷新header来消除上游延迟，也不绕过匹配控制反馈。此补丁消除的是“缓存过期反复清零”导致的重新累计，没有宣称修好0.43秒上游处理延迟。

两轮bag缺少`drop_alignment_feedback`，不能精确恢复当时该回调的接收顺序、有效性和定位年龄；时间反推见本批投递效率说明及试飞轮次报告。优先从相关targets的`last_seen`或ReleaseEvidenceContext的`geometry_target_last_seen`识别源图像；`release_evidence.header－observation_age`只能近似时间，不能把重复发布计成独立图像。生产回调的合成序列验证功能，不是新ALIGN到投递耗时或节时实测。

## 离线验证与运行命令

50项通过：生产DropAligner回调39、观测时间戳5、上下文关联策略6。新增9个序列用例覆盖4帧→缓存过期→新鲜第5帧、ready再次过期立即撤销、长断流和定时器延迟、不同几何ID／任务、新帧>30px即时清零、30px边界、重复消息不加数不续期、新的过期图像拒绝以及odom过期继续重置。原冻结反馈新鲜性／未来时间／取消上下文、旧像素模式和H回归保留。未运行ROS节点、仿真或机构，不需重编译C++。

在WSL执行以下命令；Windows外层使用`wsl -e bash -c '...'`，中文仅在本说明中，不传入WSL参数：

```bash
cd /home/xhj/liftrace-worktrees/r2026-board-frame-fix
source /opt/ros/noetic/setup.bash
source /home/xhj/miniconda3/etc/profile.d/conda.sh
conda activate rl_drone
export PYTHONPATH="$PWD/vision_ws/devel/lib/python3/dist-packages:$PWD/patrol_uav_ws-patrol_planner/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"
for name in test_compensated_drop_alignment.py test_drop_observation_stamp.py test_alignment_context_policy.py; do
  python -m unittest discover -s vision_ws/src/uav_vision/test -p "$name" || exit
done
```

需已有本F编译消息。让catkin devel路径排在源码包之前，以加载生成消息及其源码扩展；不要把系统`/usr/lib/python3/dist-packages`放到conda NumPy之前。没有安装依赖或创建环境。
