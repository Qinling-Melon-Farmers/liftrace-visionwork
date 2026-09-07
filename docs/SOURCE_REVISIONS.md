> R59唯一一轮实际源码e633a9be9fb1232e0b2bcdba5d6b68fb0d61bfd6；其后为接触评测分类和报告修正，未再实跑。原始Gate FAIL，阶段结果见[报告](verification/r59_corridor_landing/REPORT.md)。

# 源码与冻结验收版本

| 用途 | 运行源码/版本 | 实际结论 |
|---|---|---|
| R56完整成功 | cc899f2bc4816caff9f9059adb0208705476f81b | 37/37 PASS |
| R57/toudi4样式0.85m | 703541505f8dac5f2db7be333eda9c2460ed3d70 | 37/37 PASS |
| R57严格0.80m | 655bf2a78927e07516a36ed328ba93f458b747a4 | 三投后目标占据超时FAIL |
| R58最后实跑 | 59ed04b8d310af141670cb47ba1078759c8e02eb | 三投后外墙接触FAIL |
| 当前main | 31d0b2aaa2c5bd9f9554008e42301cfbde666cfa | R56验收及模型适用性说明，未合R58实验 |

最后飞行后仅收口文档、保存未加载的低空方案，不能将其文档提交写作实跑源码。用户暂停十seed，执行0组；当前没有仿真。

R56以来13次提交的完整列表与文件/参数对照见[收口报告](verification/r58_closeout/REPORT.md)。R56合入main保留分叉及功能分支，标签为gate/vcl06-r56-full-competition；成功源码另由sim/r2026-r56-contact-map-pass和sim/r2026-r57-toudi4-085-pass固定。

精简导入起点为导航987b57c6、视觉1b328c13，仅作历史来源。后续运行修复的原始资产及legacy快照保留在导航/main集成来源分支，精简分支不带旧包快照或机械PWM实现。
