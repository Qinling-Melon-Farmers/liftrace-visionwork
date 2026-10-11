# 五分类RKNN导出环境与准备结果（2026-10-11）

本轮已明确采用实测通过的FP16非量化转换路径；内部FP16与调用输入float32/255分别记录。严格FP32工具限制保留为历史检查结果，不阻塞本轮交付。最终新视频模型尚待Feynman确认完成，未读取训练中的best.pt，未生成新版最终模型或最终候选ZIP。

## 工作目录与交接

- B：/home/xhj/liftrace-worktrees/r2026-board-vision-tests。
- 数据及权重源：/home/xhj/liftrace/vision_ws。
- 最终模型预计路径：/home/xhj/liftrace/vision_ws/runs/liftrace_5cls_new_video_20261011/candidate/weights/best.pt。
- 训练agent：Feynman，01a129b2-938c-78b0-973b-170523fa50d1。无代理消息工具，最终确认经主线程/训练agent交接；不自动监听或读取中途best。
- 本轮只新增tools/model_release/。未修改训练/data脚本、运行入口、旧模型或共享日志，未提交、推送、部署、连接板端或启动ROS/Gazebo。同工作树其他代理产生的控制/训练改动均未处理。

## 环境结论

| 项目 | 实际版本或结果 |
| --- | --- |
| Python | 既有rl_drone，3.9.25 |
| PyTorch实际加载 | 2.5.1+cu121 |
| Ultralytics | 8.4.33 |
| ONNX | 1.19.1 |
| ONNX Runtime | 1.19.2 |
| RKNN Toolkit2 | 2.3.2，导入、构建、导出及CPU模拟器实测成功 |
| RKNNLite | WSL未安装；本轮无板端NPU测试需求，不安装 |
| FP16非量化 | 当前结构实测成功，do_quantization=False |
| 严格FP32 | config(float_dtype="float32")实测拒绝，明确BLOCKED，无自动降级 |

发现ONNX1.19.1移除onnx.mapping，而Toolkit2仍调用正向/反向dtype映射。已在导出进程内从onnx._mapping构建等价映射，不修改环境包或site-packages。export.json记录shim是否启用。现有Toolkit声明的NumPy/Torch版本范围与此环境存在差异，本次结论以实际CPU构建/模拟器结果为范围，不宣称所有模型与工具功能均兼容。

## 实际0928五类板端契约

依据此B内flight_5cls_20260928_metadata.yaml、target_detector_rknn.py、target_detector_rknn.yaml、phase_d_board.launch、试飞run_trial.py及正赛competition_supervisor.py核查。

类别ID固定为0 bridge、1 panzer、2 pillbox、3 tent、4 red_cross，无H正类。H由现有独立landing_detector处理。2025四类best_rknn_model/metadata.yaml从未用作新包元数据。

预处理：640×640等比例letterbox，114边框，OpenCV INTER_LINEAR，BGR→RGB，float32/255，NHWC批次1；RKNN mean=0/std=1，不重复归一化。ONNX输入[1,3,640,640]，输出[1,9,8400]；xywh像素坐标及五类概率，无objectness、无图内NMS。现有detector按类NMS及逆letterbox保持原逻辑。

metadata中的input_dtype=float32是调用输入类型，不是RKNN内部运算精度。不得把FP16或tfloat32写成FP32。

## 已完成的实际验证

1. inspect历史0928最终PT：实际YOLO11n，Detect头、nc=5、reg_max=16、stride=8/16/32；实际前向输出[1,9,8400]，2,590,815个未融合参数。
2. 用独立smoke输出从该历史PT重新导出ONNX与FP16 RKNN，并非复制旧RKNN。
3. ONNX checker、CPU执行及PT→ONNX逐图数值比较PASS（rtol=1e-3、atol=0.03，实际误差见validation.json）。
4. Toolkit CPU模拟器两张图PASS：testB_08000.jpg解码red_cross ID=4，ONNX/RKNN框IoU=0.9983469，score差0.0001380；H_flight_2.png两侧均无阈值0.5以上检测。仅两张图的转换一致性，不代表全数据集精度或H鲁棒性。
5. 12项定向测试PASS：真实生产预处理、red_cross ID、错误10通道拒绝、四类/错序元数据拒绝、输入类型错误拒绝、未最终确认交接拒绝、FP32不降级、历史smoke不得打最终包、缺附件/模型元数据串包拒绝、凭据文本/符号链接拒绝、ZIP已有文件时不产生部分tar、双归档完整性/相对路径/防覆盖。
6. CLI inspect/export/package帮助、bash语法检查通过；真实历史smoke最终记录为PASS。最终未确认CLI与smoke打包CLI应返回2，不生成候选包。
7. 双归档打包检查使用临时单测fixture并模拟PT/ONNX重载，验证归档内容；不把fixture交付为模型。真实历史PT/ONNX重载另有检查。最终新模型包的完整实物打包仍需最终交接后执行。

可再生验证产物位于B/deliverables/model_release_preparation_20261011/，不入Git。有效历史流程产物为smoke_cli_final/，其中export.json、validation.json、rknn_simulator.json均记录hardware_validated或hardware_tested=false。fp32_probe/记录严格FP32拒绝；其余早期失败目录保留具体兼容错误，不能当有效模型使用。全部smoke产物均禁止被打包为新版最终候选。

## 最终包与下一步

拿到Feynman最终确认后，先对最终PT运行inspect。导出仍要求明确最终交接JSON及训练配方、来源统计、测试摘要、正例和H负例验证图片；缺件直接失败。可复用命令见README.md。

若最终仍采用当前五类Detect结构，最终交接确认后按本轮已选定的FP16非量化路径导出；元数据明确内部float16、输入float32及一次归一化。最终包输出tar.gz及额外ZIP，内容为同源PT/RKNN/实际ONNX、metadata.yaml、classes.txt、中文替换回滚README、训练配方/来源统计/测试摘要及本地验证JSON。仅相对路径普通文件，不含整个数据集或本地符号链接，文本凭据字段拒绝打包。

包继续标“候选，NPU未实跑”，不自动上板，不覆盖0928旧资产。原入口支持--model和--metadata同时覆盖；environment.sh会重设默认模型路径，不能只依赖source前的export。中文说明已要求模型/metadata成对选择与回滚。

## 本轮暂存联调记录

日期：2026-10-11。范围：tools/model_release/及忽略的本地smoke产物。具体改动：独立导出/打包CLI、进程内ONNX兼容、严格精度请求、五类生产契约及数值/解码检查、ZIP+tar候选包、中文替换回滚模板、交接格式、12项检查。验证结果：上述CPU流程PASS；FP32工具拒绝。遗留问题：最终训练交接与最终实物包尚待，NPU/板端稳定性未做。下一步：最终确认后按实际精度导出打包并报告，后续是否提交由主线程安排。子任务分工，主代理统一追加/提交；本子任务记录暂存在本文件，未追加共享台账。

补充记录（2026-10-11）：主线程已确认本轮采用FP16非量化路径，无需严格FP32；修正共享台账与提交的归属为“子任务分工，主代理统一追加/提交”。本次仅修正文档，已核对替换文本；等待最终训练交接，不读取中途best、不导出新版。

补充记录（2026-10-11）：中文候选README加入已核实的run_trial.py --help和visual_interrupt preview --root/--model/--metadata --check-config可复制命令；在B source现有environment.sh后用系统ROS Python实际执行，两项返回0，配置预览输出CONFIG_VALID; no ROS nodes started。注明仅preview会启动ROS，--check-config才在ROS初始化前返回；该检查不验证模型文件内容。未新增部署脚本，仍等待最终交接。子任务分工，主代理统一追加/提交。
