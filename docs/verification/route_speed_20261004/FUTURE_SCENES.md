# 后续高位研究仿真场景

仅应用于高位研究工作树。整机候选与板端现场配置未被替换。

默认采用净空内环10×10m、5cm外墙、原始均匀四树、80cm H、相机AGL 2.60m（FC参考点2.76m）；允许靶板不贴墙。seed31/38保留本批生成后的靶位，其他seed按相同净空约束重新生成五类靶位。矩形、双线和三线不改变这四棵树的位置。

只有收到新的明确仿真启动请求后，才执行以下示例；生成场景自身不启动ROS：

~~~bash
# 离线只生成配置（输出目录必须尚不存在）
python3 simulation_tools/prepare_high_view_scene.py --seed 31 --route snake2 --output logs/generated/example_snake2

# 获授权后启动；设置本机五分类模型实际路径
export UAV_VISION_MODEL_PATH=/path/to/five_class.pt
SIM_RUN_AUTHORIZED=1 top_level_scripts/run_competition_sim.sh --seed 31 --route rectangle
~~~

route可选rectangle、snake2、snake3、rectangle_baseline。rectangle_baseline同时关闭本次运动衔接并显式将独立直线权重设0；其他三种设2。保留旧入口参数用run_competition_sim.sh --legacy ...，不会默认返回旧随机树场景。

入口调用现有sim_run.sh，沿用单实例锁、真实overlay检查、统一logs与收尾；没有提供绕过授权的默认值。本轮仅离线生成rectangle_baseline/rectangle/snake2/snake3（含额外seed42），检查10m净空、固定四树、五靶和独立权重，shell语法及无授权拒绝均通过，未因此启动新仿真。

八轮历史产物固定2b0678b9；以后运行读取当前代码，不能把后续运行视为同版本复测。
