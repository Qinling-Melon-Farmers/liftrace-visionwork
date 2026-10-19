# 完整交接包生成

使用Python标准库，不联网、不启动ROS或硬件。先提交所有已跟踪修改，再把外置PT/ONNX/RKNN与metadata安装到runtime_models；不复制build/devel或凭据。使用已生成工作台ZIP：

```bash
python tools/archive/build_handover.py --output /path/to/liftrace_competition_complete.tar.gz --workbench-zip /path/to/workbench.zip
```

源码来自当前Git跟踪文件，模型来自`--models`（默认runtime_models），工作台ZIP单独指定。保留Linux可执行位和仓内软链接。包内README_FIRST.md与ARCHIVE_MANIFEST.json说明版本和成员；压缩后逐成员完整读取检查。数据集模型独立ZIP、Git历史bundle以及历史回放不重复塞进此包。

新机推荐Linux/WSL用`tar -xzf`解压，原生Windows工作台单独解压ZIP运行，不依赖完整ROS工程。不要把临时profile、SSH私钥或密码文件提交进Git。
