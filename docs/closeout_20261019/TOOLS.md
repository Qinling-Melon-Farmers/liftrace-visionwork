# 小工具源码索引

以下目录均纳入最终main；ZIP是发行产物，源码以Git为准。

|工具|入口与说明|
|---|---|
|bag_replay|[bag_replay说明](../../tools/bag_replay/README.md)|
|flight_logs|[flight_logs说明](../../tools/flight_logs/README.md)|
|flight_workbench|[flight_workbench说明](../../tools/flight_workbench/README.md)|
|model_release|[model_release说明](../../tools/model_release/README.md)|
|obstacle_columns|[obstacle_columns说明](../../tools/obstacle_columns/README.md)|
|servo_diagnostics|[servo_diagnostics说明](../../tools/servo_diagnostics/README.md)|
|threshold_replay|[threshold_replay说明](../../tools/threshold_replay/README.md)|

`merge_bags.py`原为未跟踪本地工具，本次补入源码；只合并ROS1顺序分包，Windows无ROS时使用工作台原生回放入口。
模型包生成、Git归档和清理结果在外置交付目录，不包含SSH口令或用户profile。

完整源码模型归档生成器：`tools/archive/build_handover.py`；用法见同目录README。
