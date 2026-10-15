# 2026-10-15：0928远端H降落修复核验

实际fetch：导航仓`origin/liftrace_board_trial_20260928`由`2adc69bf`更新至**91459e1a421f72c1635ada3e834798e815dec6f0**。只更新远端跟踪引用，没有切换/修改导航工作区、连接板端或启动ROS。

## 昨日高位H阻塞：源码已修

[d2e8d780](https://github.com/sakelier/liftrace-controlwork/commit/d2e8d780f96117d6c216b1ceb27f0fe27215959f)在最新tip中保留并接通：

- `landing_high_alignment.h`将H源/接收年龄和相邻H间隔改为调用者传入的有界参数。
- `patrol_control.cpp:4363`生产调用显式传入`external_landing_mark_max_age_sec_`；控制器读取处2345–2346及板试/正赛控制base均为默认0.5s。helper省略参数仍默认0.2s，但生产调用没有漏传。
- odom年龄/间隔及控制循环间隔仍200ms；高位XY≤6cm、水平速度<0.1m/s、持续1秒以及时间推进/倒序检查不变。

独立执行远端`test_landing_motion_settlement.py`：**12/12通过**。使用最新未修改生产头文件、显式0.5s重算昨日同一484条样本：62条ready，首次**281.889475秒**（相对bag起点，LAND接受后约10.68秒）。它复现此前的500ms敏感性结果，证明已知时效阻塞有了源码修复和通过机会；bag录制顺序不等于精确ROS回调顺序，不能把此时刻直接当成修改后实飞下降时刻。

[结果摘要](verification_results.json)、[逐样本回算](replay_latest.csv)、[生产类回算调用](gate_runner.cpp)。测试stdout未单独落盘，摘要明确区分实际测试记录与原始输出。

## 同时存在的另一项变化：低位POSCTL放宽

[b42b8f21](https://github.com/sakelier/liftrace-controlwork/commit/b42b8f213523f85539fd8baa21788737fe18a062)取消H低位POSCTL的水平误差/水平速度检查，外层XY门槛也一并绕过。高位确认、投递及AUTO.LAND外层XY未套用该豁免。

| 最新远端入口 | H目标FC AGL | POSCTL上限 |
| --- | --- | --- |
| 03专项继承08；08默认及正赛速度档 | 0.35m | 0.50m |
| 04走廊、05低空多投 | 0.35m | 0.40m |
| 独立正赛`field_10x10_default.yaml` | 0.35m | 0.40m |

独立正赛默认对准`non_vh`、下降模式1，见该文件144–149行；并非与03/08的vh/D2模式完全相同。

低位不是“只要高度低就无条件切换”：仍有目标相对高度下界（上述0.35目标、0.05容差对应0.30m）、0.15秒/至少3个新odom样本、odom新鲜度/间隔≤200ms、H已锁定、有效反馈和OFFBOARD控制权；低位不检查垂直速度。0.50m上限比此前近地交接方案更早，不能仅用解除卡点推论落点精度提高。

仓内`docs/verification/board_merge_build_20261015/README.md`另记录板端合并后全工作空间构建exit0；本轮没有独立连接现场核实当前节点二进制，也没有新H实飞日志，因此结论是**远端已修复这处高位时效不一致，不是H实飞全部验收通过**。
