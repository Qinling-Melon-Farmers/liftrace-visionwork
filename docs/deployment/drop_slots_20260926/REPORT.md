# 三槽补偿核查与2025参考逻辑（2026-09-26）

## 结论与范围

本轮读取最新远端 sakelier/liftrace-controlwork 的板载代码分支，fetch 后仍为 fa62126213995780015d58773511de1e4569cfb8（提交时间9月25日18:24）。用户两张侧拍截图来自9月26日晚新试飞，尚无本轮bag/日志；远端源码不等于已确认今晚实际运行的二进制与参数。

**没有发现三槽表都填成同一槽，也没有发现按靶标类别永远选择一号槽。发现的是三槽补偿没有贯穿视觉对准及释放口到位判定。** 现有实现先对相机主点，再在下降阶段给位置目标追加槽号偏移；不能据此保证第二、三槽的释放口已对准靶心。

本轮仅核查、订正文档，未修改飞行代码、外参或机构输出，未仿真或上板。

## 当前试飞入口的实际参数与调用

来源 minimal_delivery_test.launch，最后再次加载 minimal_delivery_test.yaml，覆盖被包含控制配置的零偏移。默认入口中普通与红十字数组如下：

| 请求槽号 | 普通靶 slot_offsets | 红十字 dynamic_slot_offsets | PWM源码注释的物理仓位 |
|---|---|---|---|
| 1 | [-0.12, 0.00]m | [-0.12, 0.00]m | 后仓，接线待确认 |
| 2 | [0.00, -0.12]m | [0.00, -0.12]m | 右仓 |
| 3 | [0.00, +0.12]m | [0.00, +0.12]m | 左仓 |

这里是**对飞控目标位置的固定camera_init XY增量**，不是已经完整定义、标定并随姿态旋转的机体安装外参。

- 标准靶与红十字分别调用 applyDropSlotOffset(servo_id, false/true)，都按 offsets[servo_id-1] 取对应行，没有固定读取第一行。
- 两个投递分支都使用 servo_id=detect_point_counter+1；正常成功并完成恢复后递增计数。重新启动控制进程会从零计数，因此两次独立启动的单投都使用一号槽是预期行为。尚不清楚今晚两图是否来自同一连续任务。
- guarded_servo_proxy 检查请求槽号与许可槽号一致，向 raw 服务原样传槽号，不把所有请求改成1。
- 编译目标是 actuator_pwm/src/pwm_node1.cpp，其中 req=1/2/3 分别选择不同PWM控制器；变量名front/left/right仍有历史命名，但当前注释是rear/right/left。不能只按变量名判定真实接线。
- Mission Manager/视觉上下文中有 payload_slot；兼容 MissionCommand 不含槽号，旧控制仍用自己的计数。守卫可拒绝槽号失配，但尚非从任务锁槽到物理补偿统一使用同一槽描述的设计。

主要源码（均固定到所查提交）：
- [入口参数](https://github.com/sakelier/liftrace-controlwork/blob/fa62126213995780015d58773511de1e4569cfb8/patrol_uav_ws-patrol_planner/src/uav_mission/config/minimal_delivery_test.yaml#L56)
- [槽位补偿](https://github.com/sakelier/liftrace-controlwork/blob/fa62126213995780015d58773511de1e4569cfb8/patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp#L2900)
- [实际参与编译的PWM源文件](https://github.com/sakelier/liftrace-controlwork/blob/fa62126213995780015d58773511de1e4569cfb8/patrol_uav_ws-patrol_planner/src/actuator_pwm/src/pwm_node1.cpp#L1)

## 两个确定的软件缺口

### 1. 视觉认可的是共同主点，不是当前槽口

drop_aligner 的 CameraInfo 回调把 target_center 更新为 cx/cy；对准误差为目标中心像素减 cx/cy。payload_slot 用于上下文身份和许可绑定，没有参与期望像素/槽口位置的几何计算。

旧控制在 drop_ready 后冻结 waypoint_temp，随后追加该槽偏移并下降。槽位偏移不是完全未执行，也不是三个槽共用同一个数组下标；**只是视觉对准阶段仍共用相同中心，槽口精度没有独立闭环验收。**

外部任务的 dropReleaseReady 服从任务许可；arbiter检查身份、时效、阶段、释放高度及对准证据/承诺锁。下降中允许在锁定飞机XY周围20cm内使用承诺证据。它不接收三槽安装位置，没有“当前释放口距离靶心小于若干厘米”的条件，也没有专门等待追加槽补偿后的到位稳定。这不等于每轮一定提前释放，但代码不能保证该误差已经收敛。不能靠同时恢复一套旧门槛解决，应使现有对准与许可针对同一个槽口目标。

参考：[视觉主点与误差计算](https://github.com/sakelier/liftrace-controlwork/blob/fa62126213995780015d58773511de1e4569cfb8/vision_ws/src/uav_vision/scripts/drop_aligner.py#L123)、[标准靶补偿/释放顺序](https://github.com/sakelier/liftrace-controlwork/blob/fa62126213995780015d58773511de1e4569cfb8/patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp#L2265)、[红十字相同逻辑](https://github.com/sakelier/liftrace-controlwork/blob/fa62126213995780015d58773511de1e4569cfb8/patrol_uav_ws-patrol_planner/src/patrol_control/src/patrol_control.cpp#L3149)。

### 2. 固定世界轴偏移不适应机头转向，正负与物理标签还需核对

applyDropSlotOffset直接加世界XY，不使用机体yaw或完整姿态。其他地方的像素误差已经按yaw旋转，不代表这张槽位表也随之旋转。

若 r_slot 表示飞控中心指向释放口的机体系向量，正确水平几何目标为：

    p_FC,xy = p_target,xy - (R_world_body × r_slot)_xy

例如仅作符号核对：机头沿+X、后仓在飞控后方12cm时，飞控应停在靶心前方12cm（+X），让后仓覆盖靶心。若直接将后仓位置[-0.12,0]当成飞控补偿加上，则理论释放口会在靶心后方24cm。这个例子不是今晚实测误差：仓位标签、实际尺寸、地图与机头方向都尚待核实，不能直接翻转现有参数。

同理，机体系右仓Y<0要求飞控向左补偿，左仓Y>0要求向右补偿。不同旧副本的槽号与左右命名不能代替真实机构位置。

## 去年的补偿逻辑

现存2025参考副本，以及导入提交0ddc845（已混有后来视觉兼容内容），反映的是固定姿态下的经验补偿，并不能证明当年最终赛场部署唯一版本。

主控制链典型顺序：

1. 从视觉像素误差获得对准目标；距离/采样次数满足时缓存靶心。
2. 用投递计数0/1/2选槽1/2/3。
3. 按槽号给缓存目标加固定XY偏移，同时下降。
4. 高度和宽松位置门槛满足后请求相同槽号的舵机。
5. 恢复高度、清理当前事务、递增计数。

| 旧参考 | 槽1目标增量 | 槽2目标增量 | 槽3目标增量 |
|---|---|---|---|
| 0ddc845标准圆靶控制 | X−7cm | Y−7cm | Y+7cm |
| 0ddc845红十字/动态分支 | X−10cm | Y−10cm | Y+10cm |
| Desktop alignment_control_converter | X−10cm | Y+10cm | Y−10cm |

主控制圆靶有70次对准计数，红十字有50次；这是调用采样计数，不是固定秒数。所查旧主控制下降目标Z=0.10m，释放检查Z≤0.17m且到缓存未补偿靶心的三维距离≤0.15m；因此它也不是严格的释放口到位验收。旧原点和机架不同，这些高度不作为今年实机建议。

Desktop converter另有自己的count1/temp_mark，渐降后给临时XY做10cm偏移，并按固定像素轴映射移动。其二、三槽符号与主控制不同；不能把两处直接叠加，也不能认定它们在去年最终入口里同时启用。当前最简投递入口没有启动这个旧converter。

**今年不能只把去年的7cm改成12cm就视为完成安装外参迁移。** 槽号、物理位置、参考轴、补偿符号及到位条件仍须一致。详细旧链来源另见[历史槽位说明](../../planning/finish_time_20260921/LEGACY_SLOTS.md)。

## 与本机分支的关系及后续处理

本机高位研究1d6a255、板端专项fdab4c1，与试飞fa62126的 drop_aligner.py、drop_action.h、applyDropSlotOffset实现逐文本比较一致。因此上述几何缺口是共用链的问题，不能归因于今晚试飞人员把三行复制成一行。

高位研究默认控制表为三槽全零，原有mock成功不证明真实三槽精度。本机四套专项生成配置继承了试飞12cm表，但这只是参数继承，**不是槽位外参已测量正确、或三槽实投已验收**。此前“继承板端成果”的说明按本条补充理解。

下一步最有价值的是核实今晚每次投递的槽号与运行参数，再修统一几何闭环：
- 记录三释放口相对飞控的机体系XYZ、实际req对应位置，区分安装杆臂与飞控目标补偿；在旧链与新视觉中只补一次。
- 任务锁槽后即把该槽纳入对齐目标，下降及放行沿用该目标，防止“相机中心已对齐”替代“所选槽口已对齐”。
- 保留当前身份/时效/机构成功反馈约束，增加槽口误差的到位和稳定条件；偏航0°/90°及三槽分别验证，不重复堆叠另一套任务状态机。
- 今晚日志优先查看 DropSystem 的 Slot N配置、Drop action N ACK、ReleaseArbiter 的 committed slot及原始舵机req；同时保留飞控位姿、设定点、alignment context、release evidence/result。只有侧拍截图不能精确量化12cm级误差。
- 两张图若来自两次重启的单投，都用一号槽本身正常；若是同一轮连续两次成功投递，正常应递进到二号槽，需用上述日志确认。

验证范围：更新远端引用，核对launch参数加载顺序、数组索引、两类投递分支、视觉判据、许可与实际PWM编译源文件，并对本机相关实现逐文本比较。没有今晚数据，不给出本轮落点精度或唯一故障原因。
