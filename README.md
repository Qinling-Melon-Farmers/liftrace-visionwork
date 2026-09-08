# 2026无人机竞赛整机工程

**R62 seed11已实际完成建图、自动起飞、搜索巡航、panzer第1槽仿真投递确认，并恢复搜索。** 此处按用户目标主动收尾，不是整场PASS，未运行seed1–10。源码8bedcc0，相机视频与原始记录已保留；main仍为R56历史验收。

[本轮运行报告与轨迹](docs/verification/r62_operational/REPORT.md) · [运行条件](docs/verification/r62_operational/operational_status.json) · [首投证据](docs/verification/r62_operational/first_release_evidence.json) · [本地交付包](docs/BUNDLES.md)

本视觉来源分支同步模型/视觉与报告，其导航副本不是当前整机权威；完整复现/打包使用feat/r2026-competition-integrated。机载包用于联调准备，未实机验收。

当前场内为9.6m内净、四组树箱、两处左右错列0.80m通口；外围仅简单墙/柱/箱体提供点云，不做人群拟真。实测安装为相机FC下16cm、IMU下21cm、落地离支撑面6cm；保守碰撞包络55×55×40cm。新模型修复了消息类型、展开link引用、接地抖动、FC惯性采样和SITL悬停初值，保持真实碰撞/健康检查。

本批次关闭420秒提前返航，保留600秒总限时与动作失败处理；硬件默认仍启用提前返航。24个固定牛耕航点范围X[-4.3,4.3]/Y[0,7.1]，名义搜索FC AGL1.4m，不是在线自适应覆盖。

```bash
bash top_level_scripts/build_competition.sh
# 仅在明确授权后运行；本次阶段验收不代表这个完整入口已全场PASS。
UAV_VISION_MODEL_PATH=/absolute/path/best.pt SIM_STORAGE_GUARD_PATH=/mnt/f SIM_NO_RECORD=1 SIM_RUN_AUTHORIZED=1 bash top_level_scripts/run_competition_sim.sh field_seed:=11 record_camera_video:=true
```

所有日志留当前WSL项目logs，默认0bag；原生相机录像默认关闭、参数开启。Windows侧使用wsl -e bash -c。外部PX4/Gazebo/Livox插件与推理环境仍需配置。

[任务优先级](VISION_2026_ROADMAP.md) · [环境](docs/ENVIRONMENT.md) · [安装/高度](docs/CAMERA_AND_FLIGHT.md) · [实机](docs/HARDWARE.md) · [验收](docs/VALIDATION.md) · [规则](docs/competition/RULES_20260906.md)

历史：[R60先导与十seed2/10](docs/verification/r60_full_matrix/REPORT.md)、[R62早期启动失败](docs/verification/r62_seed11_gate/REPORT.md)、[R60实际11布局](docs/verification/r61_layout_search/layouts.html)、[rqt三版图](docs/verification/r60_full_matrix/topology/index.html)。历史报告保持各自版本/场景，不改写成当前全场通过。
