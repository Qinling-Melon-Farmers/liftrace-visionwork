# 2026-10-08：已验收运行链选择性集成

本次依用户授权，从整机分支 f79c2a82 选择运行闭包进入既有 feat/r2026-main-integration，再以 --no-ff 合入 main。用户确认的08现场整场链路成功是运行实现的实跑依据；原始产物还记录末段POSCTL交接后人工上锁导致取消，不将其改写为全部自动Gate PASS。此次本机验证是整包构建、回归和配置接线，不是新增实飞。

## 合并范围与历史

- 选择 FAST-LIO、FreeDOM、Fast-Planner、patrol_control、actuator_pwm、uav_mission、uav_vision、uav_high_view、相机与必要工具依赖，以及 deployment/competition 独立入口。
- 保留 main 的原始资产/历史目录，不接受来源精简分支对这些目录的删除；历史 Livox1_2 加 CATKIN_IGNORE，激活 Livox2，避免重复包。
- 保留 feature 分支、提交作者及父关系。采用 ours 作为**待审查合并的基础树**，随后明确加入选定文件才提交；这不是空的“假装全量已接收”合并。来源的非选定文档、试飞工具/样本、研究试验与旧目录清理未进入最终树。以后同步这些未选文件必须再次显式选择，不能凭 ancestry 宣称已引入。
- 不接 EV 预测/观察、实验完整投影或新回退算法。EV 专项按用户2026-10-08指示冻结保留；本轮恢复研究不以 FC reset 处理为前提。
- 旧控制与舵机已在 legacy_baseline/20261008_main_integration 保存原包、文件清单与SHA256（修改旧链规定的快照），原始作者可由源提交与合并历史追踪。

## 参数与入口

保持原正式模板及两份 candidates 的所有YAML解析值：field.example为FC高位2.6m、释放0.45m、H观察0.9m、速度1.2m/s、加速度1m/s²，前视1.0/0.4/0.15m。两个运动候选仍保持其原设计。模板用中文注释，门/H待现场测量、site_confirmed=false。有限场地实飞配置另存 field_20261007_validated.yaml，不能作为比赛默认。

使用 deployment/competition/start.sh；工作台保留在试飞分支独立发行，通过同一入口调用。运动优化/障碍柱可显式on/off或省略继承YAML。模板缺坐标时只允许离线检查，不能启动飞行。

已验收的舵机被动启动、后仓2100us释放/1700us锁止与独立实体包已集成；服务启动不复位已装载机构。LIO构建显式FAST_LIO_MATCH_THREADS=3，配置绑定大核。没有在本轮操作板端。

## 验证

- 在本目录重新运行 BUILD_JOBS=2 bash deployment/competition/build.sh：完整x86_64构建成功。
- 61项生产方法/配置回归通过；四套profile的应用preview/flight接线、两种定位对齐和建图离线展开通过，未启动ROS节点。
- 正式模板 start.sh preview --check-config 输出 CONFIG_VALID，site_confirmed=false；原比赛三份YAML与源F解析值一致。
- 独立运行不依赖board_trials。原跨配置测试读取03专项的测试文件，精简合入后改为优先读真实专项、不存在时使用带来源的仅测试fixture；全局与正式参数断言不跳过。
- 首轮测试有两项集成环境错误：缺专项测试样本、手工PYTHONPATH把源码置于catkin消息包之前。已补隔离fixture并使用真实devel环境，61项重跑通过；不修改运行阈值解决测试错误。
- 少量从旧源带入的尾随空格已清除，legacy快照保持原字节；项目改动的git diff --check通过（排除必须保留原字节的legacy快照及原样导入的Livox厂商源码；这两类保留既有尾随空白/CRLF）。

构建、测试命令与小型结果在本目录；大构建日志留logs/main_selected_20261008，不入Git。没有新的Gazebo或现场动态验收，不能凭编译称全尺寸比赛参数全部验证。
