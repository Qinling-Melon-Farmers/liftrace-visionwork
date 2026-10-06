# seed31 第三次诊断：恢复通过，下降追心漂移（2026-10-07）

**INTERRUPTED_DIAGNOSTIC_FAIL；d393faef 未通过，不运行 seed38。** 本轮成功释放2槽并分别恢复，第三槽panzer因承诺位置漂移持续拒绝。没有完成整场，也没有自主降落验收。

固定源码 `d393faef1b32193e75238cfbcdd91c2a29e8eea6`；原run `logs/drop_precision_seed31_20261007_013415`，原batch `logs/drop_precision_20261006_recoveryfix_batch/matrix.json`。保持原固定10×10/四树/三线/镜头2m场景，双端exact=true、NMS=false、slot zero、collision stop=false；32项预检及运行时复核PASS。运行时未写tracked文件、没有增加ROS节点或编码视频。

## 两次实际mock释放，逐目标同口径

| 目标/槽 | ACK源时间（秒） | 新ACK最近样本误差（cm） | 历史seed31同口径（cm） | 新源时间插值误差（cm） |
| --- | --- | --- | --- | --- |
| red_cross / 1 | 81.381 | **4.283** | 3.317 | **4.338** |
| bridge / 2 | 117.231 | **11.698** | 11.420 | **11.683** |
| panzer / 3 | 未释放 | 缺失 | 4.815 | 缺失 |

两个typed RAW_CALL_STARTED/COMPLETED事务身份一致、成功ACK，最近真实靶实例类别正确，机体中心均位于旋转后名义靶板内；不将缺失第三槽误差填0。任务层接收ACK时间为81.385/117.232，与源时间分别相差4/1ms。原位姿CSV插值不外推且遵守250ms最大间隔。

第一槽83.990秒、第二槽119.817秒有terminal `release_recovery_motion_handoff`；ACK后回升与下一槽推进正常。先前ACK后原地低位卡住未复现。本轮实际接触episode计数 **0**，但不能据此或2槽正确宣称整场成功。两点误差均未显示较历史改善。机体/model-origin中心与mock ACK不等于快递真实脱离/撞地落点。

## 第三槽最早漂移时间线

原semantic XY=(3.976923,-0.583723)，真实panzer中心=(4.0263,-0.5225)。几何ID保持5，语义目标ID4；两者不能当作同一身份。

| offset源时间（秒） | exact XY（m） | 对原semantic漂移 | 观测特征 |
| --- | --- | --- | --- |
| 132.987 | (3.995163,-0.572518) | 2.14cm | 半径173.7px，近靶板中央 |
| 135.842 | (3.926164,-0.568449) | 5.30cm | 半径354.4px，环几乎覆盖整幅图像 |
| **136.243** | **(3.701570,-0.787473)** | **34.25cm** | 半径突降72.5px，精修中心像素(1084.0,7.94)，在图像顶部边缘；此目标真实中心误差41.91cm |
| 137.794 | (3.606510,-0.774420) | 41.66cm | 半径49.1px，像素(1170.2,29.55)，仍为边缘条带 |
| 176.052 | (3.591910,-0.764358) | 42.53cm | 仍fresh/map_valid；alignment_error7.65cm，曝光容差0.598cm |

许可第一次 `commitment_position_drift` 为 **源138.000/接收138.003秒**，slot3/panzer/decision15。176秒快照中permission denied该reason；几何观测age0.09s、context有效，但ReleaseEvidence为`offset_exceeds_limit`。生产控制166秒以后日志亦为legacy_geometry=true / permission_active=false / fresh=false。**这不是需要放宽许可或伪造旧图像新鲜度的问题。** 承诺边界实际阻止了漂移目标释放。

图像证据见 `panzer_frames/panzer_center_transition.jpg`。135.842/136.243/137.794三帧的原视频时间与offset源时间一致；132.987使用相邻132.953帧，差34ms，已标明。标记来自实际CameraInfo主点加DropOffset，未声称真值像素。可见低位圆环被图像边界截断后，精修中心跳到上缘蓝白条带；支持几何误中心，不支持将此归咎于普通许可过严。尚未对生产轮廓选择做修复或改变NMS。

本轮闭合包有约2.2MiB，所有旧raw文件和视频保留。运行中许可快照 `readonly_arbiter_snapshot_1791309014.json` 是现有active bag的BytesIO内存副本读取：只在内存重新索引，没有写原bag、没有额外ROS节点；它截止176秒，live进度187秒，已注明完成chunk的滞后。正式时间线来自随后闭合bag。

## 停止与cleanup分开记录

主代理在获取具体reason/快照后明确授权停止；ROS193.000保存stop_request，向该batch所属sim_run.sh包装器PID20471发TERM。不是碰撞触发；没有等待动作deadline或600秒，没有改变飞行状态。

runner收尾检查短暂见rosmaster20626/rosout20636，原matrix保留 **INFRA_STOP**。随后独立检查已见零残留，仍调用规定的stop_toudi3_sim.sh并再次精确复查零残留；原失败不改为PASS。`cleanup_recovery.json`与run.log末尾明确Supplementary追加保留此先后顺序。

本轮中断前Gate未生成`gate_status.json`。分析显式标注gate缺失，**未捏造Gate文件/内容、未把缺失当PASS**；诊断FAIL基于第三槽实际阻断和授权中止。完整回程、过门、降落、软件COMPLETE均未发生。

交付：本目录REPORT.md、precision.json、diagnostic_summary.json、exact_path_summary.json、cleanup_recovery.json；原始exact_offsets.jsonl、panzer_timeline.json、stop_request.json及图像供本地复核。没有生成长诊断轮视频，最终可观看关键段与中心/槽位/时间标注留到最终31/38仿真结束后；当前38未授权且不得运行。

## 已准备后续入口，未启动

按主代理新方案：下降入口后固定目标，只保留稳定捕获前完整曝光投影/去重，原许可边界不动；此实现与编译由B/主代理负责。评测代理未改runner或production，当前零仿真。

只在NEW_HEAD_REPLACE完成审查/构建/提交且主代理再次明确授权case0时使用：

```powershell
wsl -e bash -c 'cd /home/xhj/liftrace-worktrees/r2026-high-view-search && source /opt/ros/noetic/setup.bash && source vision_ws/devel/setup.bash && source patrol_uav_ws-patrol_planner/devel/setup.bash --extend && source /home/xhj/miniconda3/etc/profile.d/conda.sh && conda activate rl_drone && python -B docs/verification/drop_precision_20261006/run_case.py --case 0 --batch-name drop_precision_20261006_capture_batch --execute-authorized --reviewed-head NEW_HEAD_REPLACE'
```

Windows工作目录C:\Users\ASUS、login:false；新batch尚未创建，原batch/report不覆盖。case38需新的单独审查与启动授权。
