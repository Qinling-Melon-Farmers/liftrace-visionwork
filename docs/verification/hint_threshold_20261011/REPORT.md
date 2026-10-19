# 2026-10-11 粗HINT阈值定向调整

结论：本轮选择 **0.75**，仅调整B生产粗线索配置。第二轮相对0.70，地堡位置错误装甲/桥观测减少10/2条，正确装甲/桥仅减少1/3条；两轮红十字仍有多秒观测段。该选择不消除bag2类别冲突，不提高低空确认门槛。此报告更新此前FIELD_AND_RELEASE_REVIEW.md中暂建议0.70的选择依据，保留原报告作为历史分析。

## 生产已有HINT留存

读取外部交付目录的hint_threshold_today_1.json及hint_threshold_today_2.json。沿用其筛选：已发布且投影有效、源图年龄0～0.5秒、记录高位AGL1.8～2.2m、近时位姿、独立源观测。第二轮物体位置取已审图和记忆的约0.5m邻域，并非外部精确真值。这是已有生产发布结果的阈值留存重计，**不是完整manager重跑、不是新模型生产回放，更不是accuracy或召回率**。两份JSON自身没有模型版本字段，不能将这些生产数值归给1011候选；模型间参考另列PT表。

| 观测范围 | 0.60 | 0.70 | 0.75 |
| --- | ---: | ---: | ---: |
| 第二轮地堡位置误panzer | 36 | 30 | 20 |
| 第二轮地堡位置误bridge | 7 | 4 | 2 |
| 第二轮真panzer位置panzer | 188 | 186 | 185 |
| 第二轮真bridge位置bridge | 211 | 198 | 195 |
| 第一轮bridge输出（未逐条空间标注） | 94 | 85 | 84 |
| 第一轮panzer输出（未逐条空间标注） | 158 | 148 | 146 |
| 第一轮red_cross输出 | 38 | 37 | 36 |
| 第二轮red_cross输出 | 134 | 133 | 131 |
| 第一轮tent输出 | 12 | 9 | 3 |
| 第二轮tent输出 | 50 | 49 | 47 |

第一轮全部region=unassigned；第二轮红十字/tent也是unassigned。已审场景支持其类别存在，但不将每一条输出自动标作正确。提高到0.75后的错误装甲/桥仍为20/2，不能声称错误消失。

按同类观测时间排序、相邻间隔不超过1秒划分观测段（仅时间上的连续性参考，不复现memory的空间一致性、最小采样间隔、epoch、冲突和任务状态）：

| 0.75观测范围 | 最长段时间(s) | 段内条数 | 跨度(s) |
| --- | --- | ---: | ---: |
| 第一轮bridge | 96.842–100.879 | 74 | 4.037 |
| 第一轮panzer | 99.230–106.987 | 146 | 7.757 |
| 第一轮red_cross | 107.314–109.408 | 36 | 2.094 |
| 第二轮真bridge位置 | 108.295–114.299 | 114 | 6.004 |
| 第二轮真panzer位置 | 104.701–114.579 | 185 | 9.878 |
| 第二轮red_cross | 115.256–122.509 | 131 | 7.253 |

第二轮红十字相对0.70，首条仅晚约0.040秒，末条早约0.054秒；第一轮红十字首末时刻相同。第一轮tent仅剩107.635–107.837秒、3条、0.202秒，接近配置的0.2秒跨度要求；这些条数仍未考虑最小采样间隔及空间一致性，不能保证产生可用memory。这是0.75对低优先级/弱线索的明确风险，不宣称五类都无影响。

## PT旧/候选模型参考

复用today_bag_compare中420+385张同帧输出，不重新推理：imgsz640、rect=False、原conf0.5、iou0.45、class-aware NMS。下表统计含该预测类别的帧数，类别间可重叠，未全框标真值；这些帧包含不同阶段/高度，**不能与生产HINT观测条数或记忆相等同**。

| PT模型 / 预测类别 | 0.60 | 0.70 | 0.75 |
| --- | ---: | ---: | ---: |
| 0928 / bridge | 109 | 104 | 102 |
| 0928 / panzer | 426 | 424 | 421 |
| 0928 / red_cross | 83 | 83 | 83 |
| 1011 / bridge | 108 | 103 | 100 |
| 1011 / panzer | 423 | 421 | 411 |
| 1011 / red_cross | 86 | 85 | 83 |

bag2已确认地堡5个ROI（000521/523/525/528/530）的0928错误panzer分数为0.789/0.785/0.847/0.755/0.752，三个阈值全部保留5/5；1011在这些ROI恢复pillbox，无panzer输出。候选三个新增pillbox误报0.555/0.649/0.586在0.70和0.75均被过滤；0.60仍保留0.649一例。候选正确边缘红十字000528的0.733会被0.75过滤，后续000530/532仍有0.861/0.882；000525顶部裁切的真装甲0.644在0.70/0.75均被过滤。这里是极少量同一物体重复观察，不称召回。原v6 test缺pillbox和tent，也不能作为这两类充分验收。

## 生产参数入口与修改范围

- 粗投影发布端：/target_map_projector/coarse_min_confidence，读取于vision_ws/src/uav_vision/scripts/target_map_projector.py。
- 高位接收端：/navigation/mission_manager/high_view_full/policy/coarse_min_confidence，读取于uav_mission/high_view_full.py的ingest_coarse；两端应一致，实际门槛受较高者限制。
- B专项生成器trial_config.py原242/302行两值同步0.75，覆盖未来生成的9组配置。
- 独立正赛从survey_policy.coarse_min_confidence生成上述两端。field.example.yaml及四份candidates/{rectangle_motion,snake_motion,snake3_motion,recovery_validated_field_local}.yaml仅此值改成0.75；恢复候选其他参数保持原值。
- tools/flight_workbench/workbench.yaml的正赛默认site_config是field.example.yaml，field_20261007_validated.yaml只列为“可选测试复现·非10×10正赛参数”。deployment/competition/start.sh转发参数，competition_supervisor.py要求显式--site-config。因此历史field_20261007_validated.yaml保持0.60，手动选择它会有意复现旧门槛，不是默认生产漏修。
- 通用SurveyPolicy、projector源码回退值和full_strategy.launch研究入口默认0.60不变；本次修改的是上述显式生产配置，不推广到通用研究/仿真默认。既有生成目录、板端、main均未修改。重新生成本轮生产配置才会使用0.75。

**refined旁路仍存在**：high_view_full.py的_all_hints调用memory.update(catalog.hints(ns),...)，这些精修线索不经过ingest_coarse的coarse_min_confidence。bag2冲突有vision支持，粗HINT提高无法阻止refined通道错类，不能声称消除冲突或替代模型/同位置冲突处理。

低空建议暂不提高：target_memory.yaml的std_class_confidence=0.60、cross_class_confidence=0.80及几何门槛保持原值，未加任何覆盖。现有生产统计只筛高位；PT结果没有同步位姿阶段标签，不能用其混合阶段置信度证明低空提高后仍可靠。低空需单独按同步高度/模式、人工物体ROI及多帧确认统计正确/错误确认，尤其检查弱目标、裁切和投递对准；不能将原始帧数当最终许可或实际命中。

## 复现与验证

在rl_drone环境运行（不启动ROS/GPU）：

~~~bash
python docs/verification/hint_threshold_20261011/recount.py --input-dir /home/xhj/liftrace-deliverables/pillbox_20261011
python -m unittest discover -s deployment/board_trials_4x4/common/uav_board_trials/test -p test_hint_threshold_config.py -v
~~~

recount.py从rows重计三阈值并与既有thresholds逐项断言一致，再输出时间段及两PT模型预测类帧计数。重计成功，未重复推理。

新增配置回归5/5 PASS：9组专项双入口、5份当前正赛配置双入口、0.60/0.70/0.75仅改变粗线索参数的生成差异、低空四项置信度保持、历史配置与工作台实际默认选择。

补充既有test_competition_hardware合跑23项：21 PASS、1既有执行机构资产跳过、1 ERROR。错误为test_snake3_uses_recorded_route_and_rig_checked_camera_height缺docs/planning/serpentine_20261004/route_candidates.yaml（FileNotFoundError），与本次HINT配置无关；未补造文件或扩大修改。定向HINT回归随后独立5/5通过。未构建、未仿真、未部署、未训练、未重跑完整manager。
