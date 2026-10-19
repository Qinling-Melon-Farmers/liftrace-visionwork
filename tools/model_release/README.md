# 最新五分类RKNN导出与候选打包

本工具在指定board工作树独立新增，不修改训练/data脚本、部署入口、旧模型或共享日志。所有Python通过既有rl_drone执行，不下载或安装依赖。产物写入调用者指定的新目录，已有输出目录/归档直接拒绝覆盖。

## 训练交接

导出负责人请求Feynman（01a129b2-938c-78b0-973b-170523fa50d1）提供最终best.pt的绝对路径、明确最终确认、训练配方、来源统计、测试摘要及验证图像列表。来源统计至少说明新视频与旧数据比例、五类实例数、H负样本量及训练/验证/测试拆分；摘要应说明按视频/序列的拆分及未做的测试，不能把模拟器结果当NPU测试。

用户已告知训练开始，预计最终权重位于/home/xhj/liftrace/vision_ws/runs/liftrace_5cls_new_video_20261011/candidate/weights/best.pt；只登记路径，未读取训练中的best。当前会话无代理消息工具，Feynman交接经主线程提供；最终训练确认现已落盘于/home/xhj/liftrace-deliverables/new_video_20261011/training_complete_handoff.json。handoff.example.json仅为交接格式；final=false。不要根据训练目录中出现best.pt或results.csv自动认定训练已完成，也不要自动选择最新mtime。最终确认文件必须来自训练agent或主代理已核实完整训练日志的实际交接，注明确认来源，不能用历史模型冒充最终模型。

## 执行

以下是WSL内命令。Windows宿主必须通过wsl -e bash -c执行；中文正文先写UTF-8文件，不内联进Windows→WSL参数。外层使用脚本文件可避免宿主引号丢失。

先确认最终PT的实际names、Detect头、nc=5、非end2end结构与[1,9,8400]输出：

```bash
bash tools/model_release/run.sh inspect --weights /absolute/final/best.pt
```

本轮已明确采用FP16非量化路径，使用下方命令中的--precision fp16。工具保留严格FP32请求检查：默认precision已设为fp16；显式--precision fp32才请求严格FP32。当前Toolkit2 2.3.2明确拒绝float_dtype=float32，返回非零并在export.json写BLOCKED，不降级、不产生伪FP32模型。显式选择0928兼容的FP16非量化路线后：

```bash
bash tools/model_release/run.sh export \
  --weights /absolute/final/best.pt \
  --handoff /absolute/final/handoff.json \
  --model-id flight_5cls_FINAL_VERSION \
  --precision fp16 \
  --output /absolute/new/export_directory \
  --image /absolute/validation/positive.jpg \
  --image /absolute/validation/H_negative.png \
  --training-recipe /absolute/final/recipe.yaml \
  --source-statistics /absolute/final/source_statistics.json \
  --test-summary /absolute/final/test_summary.md
bash tools/model_release/run.sh package \
  --export /absolute/new/export_directory \
  --output /absolute/new/flight_5cls_FINAL_VERSION_candidate.tar.gz \
  --zip-output /absolute/new/flight_5cls_FINAL_VERSION_candidate.zip
```

model-id必须由交接版本明确指定，禁止复用0928文件名。支持同时输出tar.gz和ZIP，归档内仅相对路径的普通文件；拒绝符号链接及凭据文本。输出必须是新目录。模型文件只在私有输出目录内复制/导出。打包必须包含实际生成PT、ONNX、RKNN、metadata.yaml、classes.txt、中文替换回滚README、训练配方、来源统计、测试摘要和本地检查JSON；缺件或非最终确认拒绝打包。test-summary可在export时省略，以便与评测并行；待评测完成，用package --test-summary /absolute/final/TRAINING_REPORT.txt补入，缺摘要仍拒绝打包。PT_TRAINING_COMPLETE.txt这种pending通知禁止当评测摘要。

metadata只由0928五类契约生成；2025四类metadata不会被读取。默认参考路径位于此工作树，可用--metadata、--detector-config、--decoder显式指定运行端参考。检查执行生产代码中的预处理和解码函数，不启动ROS或硬件。

## 本地检查范围

- 检查五类顺序、Detect/nc=5、无H正类、非end2end输出。
- 实际执行非方形、不同BGR色值的预处理，确认RGB、float32/255、640、114 padding。
- ONNX checker、CPU执行、PT/ONNX逐图数值对照及固定输入输出维度；不在图内追加NMS。
- RKNN仅mean=0/std=1、do_quantization=False；Toolkit模拟器逐图输出与ONNX通过相同生产解码比较。比较阈值0.5/NMS IoU0.45；同类逐框IoU至少0.98、score差不超过0.02。不把这个转换一致性检查当精度测试。
- ONNX 1.19.1移除了onnx.mapping，Toolkit2 2.3.2仍使用该接口。本脚本在进程内根据onnx._mapping的真实dtype映射补上正向及反向dtype映射，记录在export.json；不修改site-packages或conda环境。
- 仅模拟器，hardware_validated/hardware_tested均false。不得宣称板端NPU、FPS、10min或实飞通过。
- 旧模型流程回归必须使用--workflow-smoke及smoke_开头model-id；此类产物禁止打包为最终候选。

## 文件所有权与未提交变更记录

本次只新增tools/model_release/，不提交、不部署。子任务分工，主代理统一追加/提交；联调记录暂存本README而不追加docs/仿真联调变更记录.md，由主代理统一合入共享记录。

日期：2026-10-11。范围：五分类导出/候选包准备。改动：独立inspect/export/package、严格精度请求、生产预处理/解码检查、中文替换回滚模板与最终模型交接格式。验证结果及遗留问题见PREPARATION_REPORT.md。下一步：拿到Feynman最终确认PT及三份训练资料后，先inspect，再按明确精度导出和打包；不得自动上板。

当前交付：最终FP16候选ZIP/tar.gz已完成并核查，含更新后的403帧评测摘要和来源统计。结果见FINAL_EXPORT_REPORT.md。工具提交等待主代理通知Feynman已结束，不并发暂存或修改共享台账。
