# 本地机载包与仿真包

2026-09-09。两份包均从精简分支的干净提交导出，存于该worktree的`deliverables/`，不写其他盘、不进入git、不发布失败全场Release。包内`BUNDLE_MANIFEST.json`含精确源码版本、文件清单和明确选择的权重来源；同目录`r61_bundles_<版本>.json`记录大小与读回检查。

| 包 | 包含 | 外部依赖/适用范围 |
|---|---|---|
| liftrace_r61_onboard_<版本>.tar.gz | 视觉/相机、导航/LIO/地图/规划/任务/控制源码、配置、部署文档、RK3588 FP32 RKNN权重 | 板端重新编译；已有ROS/SDK/NPU驱动、MAVROS/设备链；机械组Servo实现另供。用于联调准备，未获R61实机验收 |
| liftrace_r61_simulation_<版本>.tar.gz | 精简工程、当前world/机架/完整依赖模型与贴图、几个历史相机对照模型、YOLO权重、报告/截图 | ROS/Gazebo、PX4 SITL/MID360插件与扫描CSV、已有推理Python环境另配。新场景静态验证不等于飞行PASS |

[机载使用说明](../deployment/README_ONBOARD.md) · [仿真使用说明](../deployment/README_SIMULATION.md)。包只解引用必要Catkin工作区CMakeLists等软链接，所有归档成员为普通文件，可移机；不复制本机build/devel、凭据、旧参考仓、全场bag或录屏。

仿真资源12个外部模型已冻结在`simulation_assets/models`并随feature推送；来源和许可证随包。最大的MID360 DAE约31MiB，为用户明确要求的必要模型资产例外；权重仍只进本地包。机架SDF与新地图分别在`iris_mid360_ks2a543_installed/model.sdf`和`r2026_horizontal_field/field.world`。

可复现打包（不会启动任何ROS、飞行或执行器）：

```bash
python top_level_scripts/build_competition_bundles.py \
  --onboard-rknn /path/to/merged_standard_fp32.rknn \
  --sitl-weights /path/to/merged_standard/weights/best.pt \
  --optional-model-root /path/to/PX4/sitl_gazebo-classic/models
```

Python使用已有环境；本机为conda rl_drone。脚本只导出git已跟踪文件和调用方显式选定的权重/对照模型，归档后逐文件读回、检查路径与压缩完整性。当前源码版本与最终实跑版本分别记录，打包和构建成功不改变R60十seed2/10或main的R56验收结论。


## 2026-09-09生成与检查结果

本地包均对应`fd147c72a82ed64d346eb49a5644384032424df7`，之后提交仅补交付检查记录：机载`liftrace_r61_onboard_fd147c72.tar.gz`为17,473,765 bytes，1225文件；仿真`liftrace_r61_simulation_fd147c72.tar.gz`为38,927,840 bytes，1417文件。位于精简worktree的deliverables目录，README.md为本地索引。

两包完整读回及内容/模型引用检查PASS；机载包重新解包后视觉+导航在WSL Ubuntu20.04 x86_64从源码完整构建PASS。未在ARM板端构建或运行，不把该结果写成上板/飞行验收。详见[bundle_validation.json](verification/r61_layout_search/bundle_validation.json)。
