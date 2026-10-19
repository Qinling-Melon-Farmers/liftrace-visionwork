# 结项构建、检查与发行边界

日期：2026-10-19。本次只做源码整合、离线检查、构建、资料清理和打包，没有启动ROS节点、仿真、飞行或机构动作。

## 已完成检查

- `BUILD_JOBS=2 bash top_level_scripts/build_competition.sh`：视觉与导航主工作区完整构建退出0。构建日志随外置交付保存。这是本机WSL/x86构建，不是新机OrangePi/ARM重新构建。
- 03、04、08及10的牛耕/回字形、各自后移1m选项：离线preview + check-config + reference-fc生成通过。
- 11临时单投check-config通过：offline=true、control_output=false、servo_output=false、starts_driver=false。
- 独立正赛显式示例、已验场地以及默认10×10配置离线检查通过。默认/示例site_confirmed=false、generation_ready=false是预期，不能直接用于起飞；历史已验场地生成成功不表示新场地已测量。
- 补回导航0928权威`field_10x10_default.yaml`，解决独立正式入口默认文件缺失；保留其未确认状态和原参数，未改成自动批准现场。
- 补回原正式路线fixture `docs/planning/serpentine_20261004/route_candidates.yaml`；相关配置测试18项、0failure/0error、1skip。
- 工作台分发allowlist补齐低空释放、场地选择、六份地图模板及bag分包合并工具。包解压后的引用文件/模块导入检查通过；Node环境下临时投递与场地选择相关16项通过。
- 模型包5317成员CRC通过，2583图像与标签对应，4396软链接已解引用，PT/ONNX/FP16 RKNN及metadata配对保存。

03第一次解析runtime.yaml曾出现PyYAML Reader.forward TypeError，后续同文件连续50次解析和原命令重试通过；未复现，原因未确定，未通过修改解析器绕开。原失败日志随包保留。

## 历史广域测试没有全部通过

161个Python测试文件，共testsRun=1624；最新结果42个failure事件、36个error事件、9项skip。subTest事件不等于失败用例数，不用相减虚构“通过数”。另外46/46自定义探针检查通过。完整输出在外置交付validation/full_test_report.md和unit_results.json。

主要涉及旧场地/阈值/IP等断言与最新0928配置不同、旧C++抽取fixture未提供新增字段、mock缺少调用状态字段，以及新独立单投组不接受旧通用motion覆盖等。此次没有为通过旧测试而改写已运行飞行状态机；也没有逐条在原分支复跑，所以不将所有失败断言为原有问题。恢复研发时应更新fixture并逐项复核。

历史patch和快照包含空白格式问题，git diff --check不是全绿；保留原patch原文，未全仓格式化。源码可构建与当前入口离线生成已验证，不能据此宣称完整动态回归全通过。

## 真实运行证据与未验证部分

10月16日两轮v8实际模型使用、既有成功整场/H及失败回放均留在历史报告。CAPTURE改类退出补丁有部署及192项离线检查，未取得修复后新实飞。EV预测保持冻结；导航恢复和H其他候选模式保留其研究状态。新机的外参、舵机标定、CPU绑定、SDK/NPU驱动版本、场地和模式仍需按下一届指南核实。

本次归档是完整可追溯工程交接，不是重新签发所有候选功能的飞行验收。
