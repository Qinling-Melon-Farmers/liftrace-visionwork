### 2026-10-11：五分类模型RKNN导出与候选包工具

- 改动范围：tools/model_release/；实际模型与归档留在/home/xhj/liftrace-deliverables/new_video_20261011/，不入Git。
- 具体改动：新增inspect/export/package CLI、固定五类顺序与640/RGB/float32归一化/9通道契约；使用FP16非量化导出及进程内ONNX兼容；允许导出与评测并行、最终摘要后补，拒绝pending通知；ZIP+tar包附同源PT/ONNX/RKNN、metadata、中文替换回滚/仅配置预览命令、配方统计和评测摘要，拒绝符号链接、凭据文本与覆盖已有归档。
- 验证结果：最终新模型Detect/nc=5和[1,9,8400]通过；6+2张PT/ONNX/Toolkit CPU解码一致性PASS；15项工具检查PASS。最终两归档各19文件，ZIP CRC/逐项字节/相对路径/无符号链接/无数据集原图bag/文本凭据检查PASS。新版14:44 bag403张摘要与最新final/test_summary.json、source_statistics.json已入包；证据：PACKAGE_VALIDATION.json与tools/model_release/FINAL_EXPORT_REPORT.md。
- 遗留问题：候选未做板端NPU、10min ROS或实飞验收。旧805帧仍有tent→pillbox@0.801；新bag五处人工ROI不是全片准确率或AP。
- 下一步：主代理确认Feynman提交完毕后统一追加本条并提交本工具；不自动部署、不替换0928旧模型。

本条目前仅为本子任务台账草稿，不是共享日志的已追加条目。
