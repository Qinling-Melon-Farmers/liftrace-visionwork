# 新视频五分类FP16 RKNN导出结果（2026-10-11）

新版已从主代理确认完成训练的best.pt实际导出，FP16非量化RKNN及本地一致性PASS。正式final/best.pt与已导出PT逐字节一致，配方、来源统计也一致，无需重新转换。新增14:44 bag的403张留出评测及最新正式摘要已纳入最终ZIP/tar.gz；双归档各19项，完整性与逐项内容校验PASS。

## 权重、交接与文件

最终model_id：flight_5cls_new_video_v8_20261011。
权重训练源：/home/xhj/liftrace/vision_ws/runs/liftrace_5cls_new_video_20261011/candidate/weights/best.pt。
正式冻结PT：/home/xhj/liftrace-deliverables/new_video_20261011/final/best.pt。
训练完成交接：/home/xhj/liftrace-deliverables/new_video_20261011/training_complete_handoff.json；正式交接：同父目录final/handoff.json。均为Feynman的明确最终训练确认；主代理已核实20轮、951.229秒、optimizer stripped、最终best验证结束。

导出目录：/home/xhj/liftrace-deliverables/new_video_20261011/rknn_export/。

| 文件 | 字节数/状态 |
| --- | --- |
| flight_5cls_new_video_v8_20261011.pt | 5,440,915 |
| flight_5cls_new_video_v8_20261011.onnx | 10,567,866，实际ONNX中间文件 |
| flight_5cls_new_video_v8_20261011_fp16.rknn | 7,545,013，实际新导出 |
| metadata.yaml | 五类新元数据，内部float16，输入float32 |
| training_recipe.yaml / source_statistics.json | 已拷贝明确配方/统计，与正式交接一致 |
| export.json / validation.json / rknn_simulator.json | 当前导出、本地数值及解码PASS |
| formal_handoff_validation.json | 正式冻结资产一致性PASS |
| formal_image_validation.json / formal_simulator_validation.json | 正式交接指定的新视频帧/H负例补充检查PASS |

没有以旧RKNN充当新模型，没有覆盖0928模型或其metadata。

## 模型与运行契约

实际YOLO11n Detect，nc=5、reg_max=16，stride=8/16/32，前向输出[1,9,8400]。类别ID为0 bridge、1 panzer、2 pillbox、3 tent、4 red_cross，无H正类。

letterbox 640×640、114边框、INTER_LINEAR，BGR转RGB，NHWC批次1、float32除以255一次。RKNN内部float16，do_quantization=False、mean=0/std=1。ONNX为float32 NCHW输入[1,3,640,640]、单输出[1,9,8400]：xywh及五类概率，无objectness，无图内NMS。现有生产解码器按类NMS和逆letterbox。

Torch 2.5.1+cu121、Ultralytics 8.4.33、ONNX 1.19.1、ONNX Runtime 1.19.2、Toolkit2 2.3.2。ONNX dtype映射兼容补丁仅在导出进程内生效，没有安装或降级包。

## 本地检查

- 五类顺序、Detect结构、真实前向维度及实际预处理PASS。
- ONNX checker、CPU执行、6张图PT→ONNX数值PASS；最大绝对差0.0042419434。
- 6张图包含五类各1张历史验证图及H负例；Toolkit CPU模拟器与ONNX经过同一生产解码，5个类别匹配框，最小IoU0.99695438，最大score差0.0003156662；H图无阈值0.5以上输出。
- 正式交接另指定新视频多靶帧new_target_f000390.jpg及H_v7_z14.png：PT/ONNX数值PASS；按同一配置从导出ONNX构建Toolkit CPU模拟器，pillbox/bridge的输出与ONNX匹配，最小IoU0.99454933、最大score差0.0004110932；H负例无阈值0.5以上输出。这里只检查转换一致性，不把两个检测框当作已完成整帧真值评估。
- 共15项定向工具检查PASS：包含pending摘要拒绝、export后补完成摘要、双归档额外评测附件、五类/预处理/输出拒绝及归档完整性、相对路径、凭据文本、符号链接、防覆盖。
- 未加test-summary的真实package请求返回2“Missing required package attachment: test_summary”，未产生不完整ZIP/tar.gz。
- run_trial.py --help及visual_interrupt preview --root/--model/--metadata --check-config已实际返回0、CONFIG_VALID; no ROS nodes started；中文包模板已纳入可复制命令。

上述均在笔记本WSL/CPU/Toolkit模拟器完成。未加载RKNNLite到板端NPU，未做板端吞吐、10min ROS、实飞或实投验收。

## 评测资料与最终打包

正式TRAINING_REPORT.txt、test_summary.json、recipe.yaml、source_statistics.json已出现。历史val/test与805帧PT评测的范围、源相关性及局部ROI限制保留原文，不转写成RKNN精度结论。test无pillbox/tent真值，真实H负例仍缺，不宣称五类全面改善。

新增14:44 bag 403张留出评测已由Feynman完成、主代理审五处ROI。已用package --test-summary正式TRAINING_REPORT.txt，并用--evaluation-attachment纳入最新test_summary.json、403张JSON/中文摘要、正式handoff以及本地formal检查JSON。只纳入模型、配方、统计/manifest和评测摘要，不打数据集、bag、原图或全量预测。

最终已同时生成candidate.tar.gz与candidate.zip，包内均为相对路径普通文件；metadata.yaml、classes.txt、中文README_替换与回滚.md及检查JSON齐备。包继续标“候选，NPU未实跑”，不自动部署或替换旧模型。

## 子任务联调记录

日期：2026-10-11。范围：tools/model_release/及外部rknn_export产物。改动：允许正式权重导出与评测并行，package后补摘要/额外评测附件，FP16默认精度，pending训练通知不得充当评测；实际导出新PT/ONNX/RKNN并写新metadata与本地结果。验证：五类结构、生产契约、6+2张数值/解码与15项工具检查PASS。遗留：板端NPU、10min ROS与实飞未做。下一步：等待主代理确认Feynman提交结束，再按分工提交本工具；不并发写共享日志。子任务分工，主代理统一追加/提交；本子任务不改共享日志、不提交、不部署。

## 最终归档校验与提交准备

输出父目录：/home/xhj/liftrace-deliverables/new_video_20261011/。
- flight_5cls_new_video_v8_20261011_candidate.zip：19,110,107字节。
- flight_5cls_new_video_v8_20261011_candidate.tar.gz：19,094,599字节。
- PACKAGE_VALIDATION.json：PASS，两份归档各19文件；ZIP CRC、tar/ZIP/源文件逐项字节一致、相对路径、无符号链接、无数据集/原图/bag、文本凭据检查通过；摘要和source_statistics与最新final文件一致，403帧附件已存在。

PT评测的5处新bag人工ROI新模型类别正确；其中偏色地堡old panzer@0.861→new pillbox@0.931。旧805帧仍有tent→pillbox@0.801；这不是零误报或全片准确率结论，也不是NPU结果。包未部署。

本轮15项工具检查、bash语法、git diff --check通过。提交内容仅tools/model_release/；提交消息与共享台账条目草稿另备，不暂存、不提交、不推送，等待主代理确认另一代理结束，避免共享Git暂存区或共享日志冲突。
