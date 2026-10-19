# H形态主判据上板与同步（2026-10-06）

已在 `orangepi@192.168.43.59:/home/orangepi/liftrace_board_trials_20260928` 部署本次H视觉方案并完成ARM构建、同帧几何复跑和配置检查。没有启动飞行或对飞控发模式/解锁命令。修改前读取到连接正常、地面未解锁、MANUAL，无活动H检测/任务/控制应用。

## 部署边界和默认设置

- 来源：视觉试飞分支 `f0ff8999`；板端旧文件备份在 `~/board_deploy_backups/20261006_h_morphology_165556/`。
- 只安装H检测C++、头文件、YAML、测试和离线工具；板端控制器、Bridge、任务生成器、03/04/08 settings未被覆盖。新二进制由下次正常启动H应用时加载。
- 默认 `landing_h_segmentation=grayscale_otsu`，灰度自适应分割+H两竖/横连笔画及凹口结构，外圈完整优先、裁切时保留笔画兜底。显式 `legacy_hsv`可作对照。分割仅产生形态候选，不能凭深色区域或单一外圈接受H。
- 03保持FC离地1.0m接近、1.2m识别、80cm实靶；末段POSCTL交接，由飞手继续落地。仍须手动解锁及切OFFBOARD。
- 04/08及正式整机保留各自原高度和AUTO.LAND末段，未把H专项高度/交接模式推广过去。
- H检测只在landing阶段启用，原YOLO阶段切换沿用；新视觉不修改10新帧、8cm、0.5秒观测门槛，也不修改投递圆环门槛。

## 板端验证

1. ARM `landing_detector_node`与`h_stroke_detector_test`构建成功；6项C++测试通过。
2. 464张实拍JPEG全部旧/新同帧复跑，对准段旧17/104、新104/104；H接近前230帧两者均无检出。60张构造图为12张H全检出、48张负例全拒绝。
3. ARM与笔记本每帧检出决策全部一致；浮点运算有极小差异（实拍中心最大0.0001px、评分最大0.000002），不要求跨架构浮点逐字相同。
4. 板端单线程几何耗时新方案P50/P95=54.48/64.35ms，旧方案54.98/61.23ms；这是离线几何计时，不包含ROS传输、投影、在线调度，不据此宣称全链路帧率。
5. 现场 `start_test.sh h preview --check-config`通过。九组直接入口中01/02/03/05/06/07/09配置通过；04/08因原有 `corridor_waypoints=[]`、`landing_xy=null`拒绝，仍需填写实测场地坐标，不虚填可执行默认航线。
6. 板端8项workbench profile回归通过：含03/04/08接线、1.2m高度、笔画兜底、POSCTL/AUTO.LAND区分、相同末段门槛、无效模式与场地参数拒绝。04/08使用仅在临时测试目录中的构造坐标，未修改部署配置。

本轮产物在板端 `logs/h_morphology_arm_20261006/`，已拉回本机 `logs/h_detection_quality_20261006/arm/`。板端摘要见[arm_summary.json](arm_summary.json)，跨架构比较见[cross_platform_comparison.json](cross_platform_comparison.json)。本机旧新同帧视频在 `logs/h_detection_quality_20261006/h_morphology_comparison.mp4`；图像/ULog原因见[原分析](REPORT.md)。

## 同步范围

视觉试飞、整机候选、高位研究、导航板端参考的检测核心已统一；导航liveness原视觉副本较旧，本轮补齐H阶段门控、笔画兜底、外圈完整度和形态主分割，构建文件只增加H测试目标。该分支现有控制器已发布`/uav_vision/align_mode`，入口也已显式启用H笔画兜底。

导航liveness验证：检测器完整翻译单元语法编译通过（使用本机试飞工作区已生成且与该分支一致的TargetDetection消息头），独立C++6项通过；没有将此称为导航全仓构建。其他四个副本的H核心与板端验证来源一致。

## 剩余验收

完成的是板端可编译、图像几何回归和静态接线验收；新视觉与新POSCTL交接组合仍需80cm真靶实飞。优先观察1.2m对准、反光/偏色、下降裁切后中心连续性以及切POSCTL后的飞手落地。整场与走廊+H还需实测几何和相应实飞，不把本轮离线检出率等同整机自动降落通过。
