# H视觉/运动并行累计增量适配

状态：**待动态验收，未部署板端**。本分支按授权增量提交推送，不操作main。

## 来源与改动

F源码提交：33a521a8d1c627e40cfce0b620683977af91521f；父提交5f117e18ddd69c0e266e24f41a03ae3e22ee4511。本分支适配起点：538eb310fc29dca5281953af0878c7292bfc0065。

- 精确同步patrol_control.cpp和test_external_landing_handoff.py、test_landing_motion_settlement.py。同步前均与F父提交一致，同步后与33a提交blob一致。
- H视觉10张新图与0.5s运动窗并行累计；短暂运动超限重置运动窗，不再使视觉计数串行等待。freeze仍需视觉与运动两项同时成立，所有阈值保持不变；future pending、物理杆臂和POSCTL接线保留。
- H测试新增超速不清视觉帧、十图齐但仍必须重新形成运动窗的检查；修复动态生成测试方法名称，使nose可发现10项。
- 原样同步[DYNAMIC_FULL.md](DYNAMIC_FULL.md)，该报告描述5f完整任务；保留本枝所有入口/CMake/profile差异。未覆盖或新建共享REPORT.md，主报告引用[F主报告](/home/xhj/liftrace-worktrees/r2026-board-frame-fix/docs/verification/slot_h_settlement_20261008/REPORT.md)。

## 本地验证与来源验证

本轮每树生产生成器/入口离线检查4 PASS，零FAIL/ERROR；日志见[entry_33a521a8_tests.txt](entry_33a521a8_tests.txt)。H运动测试静态/加载收集10项，方法名称支持nose发现；未在本树执行C++用例、未提取编译、未全包重编、未启动或干预ROS/Gazebo。源提交135项控制catkin PASS、编译PASS由主代理反馈，不能计为本树运行测试成绩。

## 动态状态（版本分开记录）

- **5f full041818**：三投、两门、H降落并上锁，任务COMPLETE、413.573s；整场Gate仍FAIL，errors为actual_collision。报告接触为保护包络与树，深度0.144mm；失败检查为contact_ready_zero、contract_errors_zero、zero_collisions，不能因任务完成改判Gate通过。
- **H045333**：已观察到低位POSCTL真实模式交接成功，这是该轮结果。
- **33a最新H专项**：主代理已启动，末报告待更新。**不写最终H版本动态PASS**，更不表示无碰撞整场验收、板端部署、实际包裹落点精度或比赛得分。

代码及本记录按用户授权中文提交、正常推送相应稳定远端，不squash、不改写历史。后续动态报告独立补录。
