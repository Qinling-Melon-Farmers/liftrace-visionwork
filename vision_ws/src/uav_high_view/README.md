# 高位观测研究：离线原型

核心、评估器和回放程序仍仅输出离线研究结果；另新增独立Gazebo静态相机采集入口，只调用仿真模型服务，不包含PX4、MAVROS或机载执行机构。原任务/候选/释放代码保持原状，不把本包建议当作任务指令。

- `core.py`：有界线索表、类别投票/位置歧义、epoch/时钟回退、复访额度、已投递槽记录，显式有向成本的最多60个三目标顺序，以及有预算的下降/回退建议。
- `evaluation.py`：按独立观察段统计物理实例发现、误分类、未匹配确认、位置P95、前三目标组合和Wilson区间。不会将检测器置信度解释为准确率。
- `high_view_replay.py`：读取JSONL、输出JSONL，强制P0待定状态；输入文件不能通过p0_passed字段打开执行。
- `high_view_evaluate.py`：读取带人工/离线真值匹配的观察段；样本足够且点估计达标时也只标记需独立复核，不输出飞行授权。

## 配置与边界

`config/offline.json`使用当前r2026五类profile。0.7置信度、0.5质量和0.25m误差阈值是原型阈值，不是高位实测标定；`uncertainty_m`是外部给出的误差下限，叠加观测散布后保守检查，不是统计置信区间。

每个epoch最多16个不同身份（包括过期身份墓碑）、每身份8次摘要，不自动淘汰后重新计算复访次数。16个身份用完时拒收新身份，适合首版有界研究；高位ID频繁碎裂会使策略退化，需实测后决定如何安全回收。未满3次独立观测或跨度不足0.5秒不成为线索；间隔小于0.25秒不累计，超过2秒重新累计连续段。

跨ID的误差区域重叠时排除歧义，不自动合并；同类出现多个位置明确的线索时，当前排序也保守排除此类，避免直接选其中一个。不是通用多同类靶算法，正式r2026一类一次的槽记忆语义保持。

线索时间戳永不重写。定位/标定变化清空空间线索，同任务的已投递类别与槽位保留；源时钟回退必须提交新的epoch。已知地图路径代价需要同epoch、有效时间且明确traversable；缺失或不可达的边不以欧氏距离补齐。费用包含当前位置/下降固定成本、每槽重捕投递恢复、第三靶到走廊入口和末端储备。

代价接口当前只接受离线声明，还没有连接原规划器或地图代际生产者，**不能把JSON中的traversable=true当成真实路径已验证**。下降证据同样仅是离线契约，实际传感器/自由空间验证未接入。P0状态默认false、命令行固定false。

## 使用

构建/安装后，在已source该工作区的终端使用：

```bash
rosrun uav_high_view high_view_replay.py --config /绝对路径/offline.json --input /绝对路径/events.jsonl --output /尚不存在的路径/results.jsonl
rosrun uav_high_view high_view_evaluate.py --config /绝对路径/offline.json --annotations /绝对路径/segments.json --output /尚不存在的路径/p0.json
```

`rosrun`这里只寻找Python脚本，不启动ROS master或仿真。也可直接调用安装目录脚本。输出路径必须不存在，输入不能被覆盖。回放单行≤64KiB，路径边≤64，槽成本≤48，评估标注批次≤8MiB。

事件类型为context、observation、edge、revisit、delivery_ack、snapshot。格式示例见[合成输入](../../../docs/verification/high_view_p0_p1_20260913/synthetic_replay.jsonl)。delivery_ack仅演示原任务已提交槽位的记录接入，不能由视觉候选或手动改标记冒充实投。

当前实现结果、资源测量和限制见[报告](../../../docs/verification/high_view_p0_p1_20260913/REPORT.md)，下一步真实数据格式见[采样说明](../../../docs/verification/high_view_p0_p1_20260913/DATA_PLAN.md)。

## 独立Gazebo观测入口

launch/gazebo_observation.launch 配合 high_view_gazebo_capture.py，只从已安装机架SDF提取相机，在固定场地网格和指定高度重定位采样。必须经sim_run显式授权，不能与另一套仿真同时运行。不是飞机飞行或导航验证。已完成32/34两布局480帧，[报告与使用边界](../../../docs/verification/high_view_render_20260913/REPORT.md)。
