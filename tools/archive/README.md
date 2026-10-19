# 机载运行工程交接包生成

使用Python标准库，不联网、不启动ROS或硬件。先提交所有已跟踪修改，再把外置RKNN与metadata安装到runtime_models；不复制build/devel、Git历史或凭据。使用已生成工作台ZIP：

```bash
python tools/archive/build_handover.py --output /path/to/liftrace_onboard_handover.tar.gz --workbench-zip /path/to/workbench.zip
```

生成器按SCOPE_PREFIXES/SCOPE_FILES明确选入当前运行源码和依赖：导航工作区src，uav_vision/uav_high_view/camera_sdk，正赛及试飞入口与场地配置，工具和构建脚本，必要上手文档。模型只带RKNN及metadata/说明，PT/ONNX/训练集另交，不塞进板端运行包。

不包含Visual、detect_ws、Desktop历史工程、legacy_baseline、部署历史源码快照、大批复盘报告、仿真世界、Git bundle、bag、录像和个人材料。当前依赖包自身附带的测试/示例保留，避免拆坏包内构建规则。

保留Linux可执行位和仓内软链接。包内README_FIRST.md与ARCHIVE_MANIFEST.json说明版本和成员；压缩后逐成员读取检查。Linux/WSL用tar解压，原生Windows工作台单独解压ZIP，不需要完整ROS工程。

例外只包含一份专项生成器实际读取的基础YAML：docs/verification/fov_landing_inner_20260919/seed_2672/fast_runtime.yaml；不带同目录历史报告。
