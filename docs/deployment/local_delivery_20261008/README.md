# 2026-10-08 本地交付索引

[原始七项完成范围、当前恢复能力、槽位方向确认、H低位交接候选与各页面截图](FOLLOWUP.md)。

用户已离场，本批不连接板端；全部构建、回放、工作台验证及恢复仿真均在本地进行。正式比赛参数保留原值，08有限场地参数只作独立回放示例。

## 工作区与分支

根工作区 `/home/xhj/liftrace` 已切到更新后的 main，与 origin/main 同为 **6cd0207f**，完整构建和61回归通过，保留用户原有未跟踪文件。当前打开的 `r2026-board-frame-fix` 继续使用整机分支 **f79c2a82**，不是旧副本。主干合并通过既有 main-integration 分支和非快进合并，tag为 `chore/competition-runtime-20261008`；旧资产和贡献历史都保留。

导航本组liveness **ff21f650** 已推送 fork 与 sakelier 的 origin 同名分支；板端参考 **67fc4440** 已推送 origin，两个本地工作树实际更新并完成构建/离线检查，导航main未合并。完整HEAD见 [VERSIONS.json](VERSIONS.json)。

EV预测/观察 **ec618761** 冻结保留并已推送；已采用的LIO三线程/大核绑定和处理优化保留，不撤销优化，也不删除EV历史。本轮恢复不以FC reset/独立高度证明或EV预测为前置依赖。

## 回放、对准与H

[成功轮报告](../../verification/flight_221730_20261008/REPORT.md)及 [本地视频播放器](../../../试飞产物/board_full_mission_20261007_221730/analysis/index.html)已完成。四条完整视频307秒及三投/槽3/H切片解码通过，原bag和参数保留，没有ULog。

第三槽确实向地图Y加12cm。2026-10-08用户进一步确认：左仓投口位于FC中心正左侧约12cm，其余两仓同为对应方向的安装杆臂。按这一明确物理定义，当前固定地图XY相加的补偿符号错误，也未随机身航向旋转：左仓应让FC向右移，右仓向左，后仓向前。正确关系为机体目标XY＝靶心XY－旋转到任务系的投口杆臂XY。12cm测量值保留；本轮只明确问题与修复方向，尚未修改运行代码，也不把条件几何推演写成实测落偏。H本轮87原始/85有效地图观测、约3秒锁定，主要末段约9cm位移发生于POSCTL交接后；没有据此把它误判成形态识别退化。人工上锁时的软件取消与现场成功口径分别保留。

## 工作台

[说明与验证](../../verification/workbench_release_20261008/REPORT.md)。独立正赛卡片调用 competition/start.sh，正式模板与测试示例分开，运动优化/障碍柱为继承或显式on/off。移除单独电机大页，实时页仍保留；日志链接同源，不再写死8791，默认请求8771。

Windows包：`C:\Users\ASUS\Downloads\liftrace_flight_workbench_windows_20261008_final2.zip`。解压运行start_windows.bat。已在Windows实际解压启动、SSH localhost fixture和Edge验证，不依赖WSL运行后端；新机仍需Windows Python和PyYAML/Paramiko等轻量依赖，不是独立exe。本轮没有连接真实OrangePi。

## 三类恢复

高位工作树 docs/verification/navigation_recovery_20261008/REPORT.md 给出实现与结果。真实map/FSM/server/controller + 合成运动模型的虚拟柱退出、普通额外膨胀退出、超高回入三轮均PASS，退出后有新普通轨迹并到原目标，原deadline保持，release_commands=0，仿真收尾无残留。

首轮column退出点只满足占据条件而不满足普通ESDF净空，实际超时FAIL；已保留原记录并修为退出点同时满足普通规划净空/停止容差，再重跑通过，没有降低原净空。

恢复仍是默认关闭的高位候选，未接入稳定整机/main/板端；动态测试不含PX4/Gazebo动力学或runtime Bridge，Bridge依生产方法测试。普通膨胀只放行必要机体净空之外的额外缓冲；真实实体、未知空间等不豁免。进入速度/半径/时限等有界，不承诺任意困境都能退出。下一步应在稳定基线适配并做完整整机动力学验证，再决定部署；正式场地测量与比赛速度仍需相应现场验收。

## 历史与清理

[旧分支索引](../../maintenance/history_recovery_20261008/INDEX.md)：36个删除分支有精确tip，另1个仅观察点；35个保全tag已推远端。原高位、运动约束原分支已恢复。归档中视觉仓缺的12个HEAD都在导航Git找到。261.79MiB离线bundle是导出时快照，原作者/日期/父关系保留。

已清理的两棵无用工作树中，旧VCL06的21项未提交修改先完整归档再移除；不删除原始资产、成功实飞bag或历史分支。没有把多分支共享祖先重复相加当工作量。
