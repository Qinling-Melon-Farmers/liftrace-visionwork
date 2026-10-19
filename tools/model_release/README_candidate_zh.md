# @MODEL_ID@ 五分类视觉模型候选：替换与回滚

本包为本地导出的候选，RKNN请求/实际精度为 @PRECISION@，非INT8量化。尚未进行RK3588 NPU推理、板端ROS连续运行、吞吐或实飞验收。CPU/Toolkit模拟器检查结果只证明输入输出及解码一致性；训练测试摘要的指标保留其数据集和来源范围。

## 包内容

- 同源的PT、实际导出ONNX、@MODEL@、metadata.yaml、classes.txt。
- export.json：最终训练交接、实际架构、工具版本、精度及预处理来源。
- validation.json、rknn_simulator.json：本地检查与逐图一致性。
- training_recipe.*、source_statistics.*、test_summary.*：训练agent提供的原始配方、来源统计及测试摘要。

必须从同一目录选取模型和metadata。2025四类best_rknn_model/metadata.yaml与本包不兼容。H不是模型正类，由原独立landing_detector检测；本包不修改H检测器。

## 输入输出

| 项目 | 约定 |
| --- | --- |
| 类别ID | 0 bridge；1 panzer；2 pillbox；3 tent；4 red_cross |
| 图像 | BGR原图经等比例letterbox至640×640，边框114，OpenCV线性插值 |
| RKNN输入 | RGB，NHWC，批次1，float32；Python只除以255一次 |
| RKNN图内预处理 | mean=[0,0,0]，std=[1,1,1]，不重复除以255 |
| ONNX输入 | float32，NCHW，[1,3,640,640] |
| 输出 | [1,9,8400]，前4通道xywh为letterbox像素坐标，后5通道类别概率 |
| 后处理 | 无objectness、未内置NMS；由现有detector阈值、按类NMS和逆letterbox还原 |

metadata描述输入float32，不代表RKNN内部权重/运算为FP32。Toolkit2 2.3.2拒绝float_dtype=float32；不把FP16、bfloat16或tfloat32标成FP32。输出格式不兼容的模型直接拒绝，不能靠改类别表掩盖。

## 后续人工替换

本说明只提供操作步骤，导出/打包脚本不执行上传或设备操作。

1. 在停止视觉/任务且设备静止时，记录目前实际使用的工程根、启动入口、RKNN绝对路径、metadata绝对路径，以及启动配置中model/metadata覆盖值。保留0928旧模型和旧metadata成对原位。
2. 将本候选完整解包到新的独立版本目录，例如工程内runtime_models/@MODEL_ID@/。不要覆盖flight_5cls_20260928_fp16.rknn或原五类metadata。
3. 对支持参数的入口同时指定新RKNN和metadata的绝对路径：试飞run_trial.py支持“--model 新路径 --metadata 新路径”；board_camera_vision.launch/phase_d_board.launch支持“model_path:=新路径 metadata_path:=新路径”。原environment.sh会重设默认RKNN路径，所以单独提前export不足以可靠选择新模型。
4. 正赛入口的模型/metadata覆盖方式需在所选入口的参数帮助中核实；使用其显式参数同时选两条路径。若入口不能覆盖metadata，先由维护者适配入口，再使用此候选；本包不自动改入口。
5. 首先做地面、仅视觉的板端NPU验证，检查load/init/inference、输出[1,9,8400]、五类ID、真实帧解码、图像颜色/尺度以及负样本H。单图一致性、10min稳定性和时延分别记录，未做项继续标未验收。
6. 验证通过后由现场负责人决定是否晋级。出现输出/metadata不匹配、异常空检测、H误识别或时延积压时，先停用候选，恢复上一套模型路径。


## 仅查看帮助与离线配置预览

下面在已有0928工程的Linux终端执行；先进入现场实际工程根目录。source只加载环境，不启动节点。先将本包解到runtime_models/，使模型和metadata均位于runtime_models/@MODEL_ID@/。预览命令不要求模型已装到该路径，最终使用前仍须成对核对文件。

```bash
cd /home/orangepi/liftrace_board_trials_20260928
source deployment/site_20260928/environment.sh
BOARD_ROOT="$PWD"
TRIAL_SCRIPT="$BOARD_ROOT/deployment/board_trials_4x4/common/uav_board_trials/scripts/run_trial.py"

/usr/bin/python3 "$TRIAL_SCRIPT" --help

/usr/bin/python3 "$TRIAL_SCRIPT" visual_interrupt preview \
  --root "$BOARD_ROOT" \
  --model "$BOARD_ROOT/runtime_models/@MODEL_ID@/@MODEL@" \
  --metadata "$BOARD_ROOT/runtime_models/@MODEL_ID@/metadata.yaml" \
  --check-config
```

已在对应WSL工作树实测入口选项及此离线分支：返回“CONFIG_VALID; no ROS nodes started”。必须保留--check-config；仅写preview仍会启动ROS节点。--check-config在初始化ROS前返回，只验证命令参数和任务配置，不加载RKNN、不检查这两条模型路径的文件内容，也不构成NPU或实机验收。不要在本次预览中加--real-release。包不提供或执行部署脚本。

## 回滚

停止当前使用候选的视觉/任务入口，恢复第1步记录的旧RKNN和旧metadata两个路径及启动覆盖配置。0928基线一般为runtime_models/flight_5cls_20260928_fp16.rknn与vision_ws/src/uav_vision/config/flight_5cls_20260928_metadata.yaml，但以现场已记录的实际版本为准。重新核对640/RGB/NHWC/float32归一化、五类ID和输出通道，按原入口完成仅视觉确认。本包保留在独立目录，不需要覆盖或删除旧资产。

未进行NPU测试时，任何README、验收表或交付消息都应保持“候选，NPU未实跑”。
