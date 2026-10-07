# H 槽位补偿与低位 H 接管：增量适配交付

日期：2026-10-08。最新状态：F `073c8f19`、`95ce182e`、`5f117e18` 增量已适配到 H；按用户授权只交付同名高位研究分支，未部署、未进入 main/F。本轮动态仍在进行，不能报告动态 PASS。文件清单见 `FILES.txt`。

## 最终高位/低位高度分离补丁

- `5f117e18` 的 H 适用八个文件已三方合入：控制 cpp/h、三份 board trial 模板、landing_posctl_config 及两份相关测试。H 不存在的 F competition 配置不整套导入。保留 exact 原靶心→单次补偿→统一冻结以及默认关闭的 recovery。
- H 降落观察阶段使用独立 `capture_height_tolerance_m=0.10`；低位仍使用 `height_tolerance_m=0.02`、距地 `0.35m` 目标及 `0.37m` 上界。新参数通过现有配置生成器传递；高位的速度、XY、连续 odom、图像计数门槛不因高度容差分离而放宽。
- 最新补丁相关离线测试 **96/96 PASS**：补偿几何19、降落运动9、稳定窗10、H exact49、POSCTL配置9。包含高位4–6cm高度误差可通过、低位同等正负误差仍拒绝、回到低位目标后须重新形成稳定窗。输出为 `/tmp/high_view_slot_h_handoff_20261008/capture_test_*.txt`。
- **构建边界**：H 的完整 `-j2` 构建 PASS 对应 `95ce182e` 时序适配（04:06:23）；按用户要求，合入 `5f117e18` 后仅执行 Python 提取 C++/配置离线测试，没有重编整包。因此当前 H 旧运行二进制不代表已包含最终 capture 补丁。F 的 build/controller catkin125 PASS 只记录为来源结果，不能替代 H 最终整包构建。
- **动态边界**：主代理反馈 H040634 专项已到低位并请求 POSCTL，但没有模式回执，仍待另查，不能记成功；F 完整 seed38 正在运行。本子代理未启动仿真，也未把上述现象认定为已修复或动态 PASS。
- 用户已授权中文提交并推送 `feat/high-view-route-speed-20261004`；不推广 exact、不启用 recovery、不改 EV、不部署、不合入 main/F。后续最终整包构建、动态结论和模式回执问题均由主代理调度。

## 过程记录：04:06 时序补丁与构建

后续定向复跑：用户授权后，对最终同步版本执行 `test_compensated_drop_geometry.py` 19 项、`test_landing_motion_settlement.py` 7 项、`test_landing_handoff_stability.py` 10 项、`test_exact_drop_control.py` 49 项，合计 **85/85 PASS**。输出为 `/tmp/high_view_slot_h_handoff_20261008/final_test_*.txt`。仅 Python 提取 C++ 离线测试，没有重编整包、启动 ROS/仿真或修改运行源码；等待 Popper 的高位 capture 容差最小补丁。此结果补齐下文所述最终 C++ 提取测试待办中的这四组，不代表修复或验证了尚未合入的 capture 补丁。

- 以 F 相对 `dbd6e3e2` 的未提交增量三方合并，随后再次承接拒绝原因读取竞态修正。最终逐文件比对确认：合并所用 F 的 `patrol_control.cpp/h`、`drop_aligner.py` 及四份相关测试内容，全部与提交 `95ce182e` 一致。保留 H 自有 exact/recovery 代码，没有整文件替换。F 的五份报告/记录产物不直接复制成 H 验收，由本节说明。
- 控制端区分本征无效、陈旧/异常时钟和有界 future 暂存；pending 不给就绪、不清稳定窗。`freshMotion` 增加可选 `const char** rejection`，调用方使用同次读取的拒绝原因；H landing tick 使用 pending 快照，避免一次 tick 内时钟前进导致判断不一致。
- aligner 对有界未来反馈/同动作 context 暂存，时钟追上后按原身份、时效和几何条件复验；等待超时不放行、不延长原 deadline。H 原靶心单次补偿、统一冻结及重复图像约束保持。
- 两个工作区完整构建均 PASS：`/tmp/high_view_slot_h_handoff_20261008/vision_future_final_build.txt`、`control_future_final_build.txt`。完成时间见 `build_final_completed.txt`。上一轮 `*_future_build.txt` 是被最终竞态修正取代的构建记录。
- H aligner 29 项通过（含 F future 用例和 H exact 组合），见 `future_alignment.txt`。第一版时序合入后控制几何 15、exact 49、H motion 7、外部接管 23 项通过；最终再次同步 F geometry 19 用例及接口签名后，完成了完整构建，尚未在 H 重跑这一批 C++ 提取测试，以便让出主代理仿真资源。F 的 controller125/vision28 PASS 是来源验证，不能记为 H 本地执行结果。
- 本节状态优先于下方初次准备时的构建待办。主代理准备 H-only Gazebo，子代理不启动测试/构建/仿真，不 commit/push；后续由主代理给出动态结果及继续验证时机。

## 来源与范围

- H：`/home/xhj/liftrace-worktrees/r2026-high-view-search`，分支 `feat/high-view-route-speed-20261004`，源 HEAD `62afbef8f9ff0a577b5db64a8e7d07c8767bf81c`。
- F：只读提取 `f79c2a82..073c8f19` 增量，通过 `git apply --3way` 适配；未覆盖整份 controller/aligner，未修改 F/B/nav/main/EV。
- 修改旧链之前保存 H 自身 52 个受跟踪原包文件至 `legacy_baseline/20261008/high_view_slot_h_handoff/`，含 `CATKIN_IGNORE`、原包、`FILES.txt`、`SHA256SUMS`、记录源 HEAD 的 `SOURCE.txt`。未使用 F 快照代替 H 原包；快照字节保持原样。
- F 的 9 个正式 competition 基础文件在 H 不存在，未整套引入：`deployment/competition/` 下的两种 motion candidate、field.example、field_20261007_validated；`uav_mission/config/competition/` 下 control_base、known_rig；`uav_mission` 的 competition_config、hardware_bag、test_competition_hardware。H 使用现有 board trial 生成器及 vcl06 控制配置承接对应参数，不宣称完成 F 正赛部署入口移植。

## 实际接入

1. 槽位解释为实际机体 FLU 杆臂，保留原实测表：后槽 `(-0.12,0)`、右槽 `(0,-0.12)`、左槽 `(0,+0.12)`。控制器用完整四元数旋转杆臂，再从原靶心减去该杆臂的水平分量，生成唯一 FC 目标。板端生成器继承 known_rig 实测表；通用 vcl06 仿真配置仍保留原零槽位表，不能把该默认仿真配置视作已启用实机 12cm 安装参数。
2. 高位先对准补偿后的 FC 目标；新增 `DropAlignmentFeedback` 携带动作身份、源图像/odom 时间、误差和速度。aligner 以匹配反馈生成 ready，允许投口对准时像素偏离主点，不能由像素居中绕过运动稳定条件。
3. 高位捕获通过后统一冻结，再下降。释放同时检查实际投口与冻结 FC 的水平误差 `<=0.04m`、FC/投口水平速度 `<=0.05m/s`、至少 3 帧不同 odom 且持续 `>=0.3s`；同时保留高度、垂速、新鲜度、动作 deadline、接管、许可和异步 RPC 门控。
4. H 的 exact 实验若启用，只发布本帧原靶心；随后只调用一次 `setCompensatedDropTarget`。新链不再消费旧 exact 冻结/计数/释放门控。冻结后的无效图像和新中心不能覆盖 FC 目标、刷新源时间或重启低位追踪。旧实验逻辑只在显式关闭新补偿链时隔离保留。
5. `phase_d.launch` 的反馈默认跟随 `require_alignment_context OR exact_drop_projection`，显式覆盖仍有效。正式 scope 仍为 stable pixel；exact 默认 false，未推广实验投影。
6. POSCTL 候选使用距地 `0.35m` 目标、`0.37m` 接管上界，并要求中心与运动停稳；AUTO.LAND 原 `0.40/0.55m` 配置及流程保持。H 三份 board trial 模板、生成器和配置测试同步。
7. H 原导航恢复保留，本次未修改 Fast-Planner、recovery 消息/配置或目标投影器；恢复仍默认关闭。

## 初次适配离线结果（时序补丁之前）

| 测试文件 | 通过数 |
| --- | ---: |
| patrol_control/test/test_compensated_drop_geometry.py | 15 |
| patrol_control/test/test_exact_drop_control.py | 49 |
| patrol_control/test/test_async_servo.py | 24 |
| patrol_control/test/test_authorization_gate.py | 6 |
| patrol_control/test/test_external_landing_handoff.py | 23 |
| patrol_control/test/test_landing_handoff_stability.py | 10 |
| patrol_control/test/test_landing_motion_settlement.py | 5 |
| uav_mission/test/test_landing_posctl_config.py | 9 |
| uav_vision/test/test_compensated_drop_alignment.py | 21 |
| uav_vision/test/test_drop_geometry.py | 15 |
| uav_vision/test/test_drop_observation_stamp.py | 5 |
| 合计 | 182 |

控制测试提取当前生产方法编译运行；视觉测试使用真实生成消息及生产回调、mock ROS 传输。新增 exact+补偿圆环/红十字组合用例验证一次补偿、统一冻结、冻结后无效/新图像不改目标；视觉组合用例验证不再调用旧槽位补偿或第二份 body TF。原投影几何用例另行回归。

原始离线输出：`/tmp/high_view_slot_h_handoff_20261008/`。控制几何为 `control_geometry.txt`，exact 为 `exact_control.txt`；其余最终输出为同名 `test_*.txt`，投影几何最终输出为 `test_drop_geometry_conda.txt`。早期失败输出保留，仅对应移植测试桩重复字段、H 场地坐标/重复帧假设或系统 Python 缺 scipy；修正测试适配后通过，没有放宽 H 的不同源图像约束。scipy 测试改用既有 `rl_drone` 环境，未安装依赖。

仅执行过 `uav_vision_generate_messages_py -j1` 生成新消息 Python 类型以供离线测试；没有完整构建 H 控制器，没有启动 ROS、Gazebo、PX4 或硬件。当前旧二进制不能视为已包含本补丁。主代理正在 F073 上跑的 seed38 结果不能替代 H 验收。

## 主代理后续入口

主代理待独占仿真结束后，用 H/H 联合 overlay 完整生成消息并构建控制器；继续沿用项目构建流程，`VISION_WS=$H/vision_ws`、`UAV_WS=$H/patrol_uav_ws-patrol_planner`。本子任务不启动该构建或任何动态轮次。

离线复跑环境（WSL bash 内执行）：

```bash
cd /home/xhj/liftrace-worktrees/r2026-high-view-search
source /opt/ros/noetic/setup.bash
source vision_ws/devel/setup.bash
source patrol_uav_ws-patrol_planner/devel/setup.bash --extend
nice -n 10 /usr/bin/python3 -m unittest discover \
  -s vision_ws/src/uav_vision/test -p test_compensated_drop_alignment.py
```

其余表中测试按对应目录/文件替换；`test_drop_geometry.py` 另先激活现有 `rl_drone` 并用该环境 `python`。测试不启动节点。动态入口、case 与开始时间由主代理统一决定。

本轮交付含29个代码/测试/配置文件，另加本报告、文件清单、联调变更记录和 H 原包快照。三方合并冲突已解；`git diff --check` 通过（legacy 快照排除）。原四个无关 untracked 试飞资产不纳入交付。用户最终授权提交/推送同名 H 分支，不进入 main/F。
