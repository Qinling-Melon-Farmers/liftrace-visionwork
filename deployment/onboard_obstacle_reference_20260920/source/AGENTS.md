# AGENTS.md — 2026 精简整机分支

先读 README.md、VISION_2026_ROADMAP.md、docs/VALIDATION.md、docs/ENVIRONMENT.md、docs/INTERFACES.md，再查看本次相关代码和 git status。任务优先级仅在 ROADMAP 维护。

本分支按用户要求集成今年导航+视觉，机械组舵机实现不包含；原始资产和冻结参考留在来源分支及 git 历史，不在这个精简 checkout 重复保存。不得清理其他 worktree 的原始文件或用户未提交修改。

- 小步修复、实际编译并执行相关检查；每个可验证改动追加 docs/仿真联调变更记录.md（日期、范围、改动、验证、遗留、下一步），再作中文 conventional commit。中文提交通过 UTF-8 文件和 git commit -F，禁止 Windows→WSL 参数内联中文。
- Windows 中所有 WSL 命令使用 `wsl -e bash -c '...'`。文件工具用 WSL UNC 路径。脚本目录用 `${BASH_SOURCE[0]%/*}`。Python/ML 使用既有 conda 环境，系统 Python 仅用于 ROS，禁止系统 pip 安装。
- 仿真只在用户明确要求启动/实跑时运行，且必须由 sim_run.sh 包装；调用方临时设置 SIM_RUN_AUTHORIZED=1。单实例，成功、FAIL、异常、信号均收尾并确认无 ROS/Gazebo/PX4/RViz 残留。同一失败先分析再修复，不无分析反复重跑。
- 实机解锁、起飞、舵机、PWM 等必须单独获得用户确认。SITL 的参数设置、arming 和 mock 不得用于硬件入口。仿真不启动硬件驱动。
- main 只接受实跑验收后的 feature 合并，使用 --no-ff，禁止 squash/fast-forward；合并、推送 main 和删分支需要用户授权。已经明确授权的最终交付不重复询问。合并后立即创建并推送 annotated tag，保留 feature 分支。
- 凭据不入库、不输出；git 凭据仅用 WSL 已有凭据存储。权重、数据集、build/devel、bag、录屏及大日志不入 git；提交小报告并提供原始归档索引。
- 相机话题、内参、安装外参、模型路径、frame 用参数。模拟图像和投影必须一致。真值只能用于评测，不进入规划/选靶/控制。
- OrangePi 实时主路径使用 RKNN/NPU，OpenCV 做轻量几何。不能把笔记本推理结果写成板端验收。
- 保留实际使用的接口兼容；不增加重复任务权威，不凭接口名称删掉当前链需要的服务。旧 patrol_control 状态机只做必要补丁，先保存可审计快照于来源开发分支；不要把旧快照重新塞进本精简分支。
- 不做无关哈希、敏感信息检查或过度审计。只有 legacy 保护快照或确有意外改动疑问时才需要哈希。
