# 2026-10-16 工作台现场配置交付

两套工作台同步；仅工作台配置选择、地图读取/编辑、测试和离线参考副本变更。未改飞行控制、部署源配置或共享变更记录，未提交。导航最终来源由主代理确认：239c63bd。

默认04走廊专项使用 `04_corridor_landing/site_waypoint_map_20261016.yaml`；默认08使用 `08_full_mission/site_waypoint_map_20261016.yaml`。10默认关闭后移选项，牛耕使用 `site_lawnmower_20261016.yaml`（原六点X=1/3.6/6.2），回字使用 `site_nested_rectangles_20261016.yaml`（显式两框，无search_waypoints_xy）。两份 `*_shiftback_1m_20261016.yaml` 仅由明确勾选后移1m启用。完整路径均位于board_root下 `deployment/board_trials_4x4/` 对应组。

命令和地图context/draft/save/import/disable共同携带coverage_pattern和coverage_shiftback；按所选文件隔离草稿和命令缓存。显式搜索点、两框在导入/保存/overlay中保留，不根据更新边界重算。手动覆盖优先；手动覆盖启用时路线选项禁用，先恢复原现场配置再切换。前后端来源不符时拒绝旧context/draft，旧运行后端缺新配置时提示重载。

离线地图严格读取随包对应文件，明确标注未核对板端部署。在线读文件失败提示未部署或无法读取，不回退昨天地图或离线副本。六份参考保存在 `waypoint_templates/sites/deployment/board_trials_4x4/`，逐文件与导航源内容一致。04为6个走廊点、H(7.51,-3.85)、捕获1.1m；没有独立仅走廊无H卡片。fixed另有05多投接走廊/H，本次未改。

新区域76.63㎡理想FOV参考：默认低位1.8/1.9m为88.56%/89.91%，可选后移为99.74%/100%，默认高位2.6m为93.46%。手动几何时显示需重算，不代表实飞检出率。

验证：源码工作台23项通过；fixed工作台32项通过、1项源码依赖测试跳过。每套包含9项test_site_selection、9项test_low_coverage_backup；旧地图编辑回归分别5项和11项（fixed其中1项跳过）；fixed另有4项launch_contract。site_selection内每套运行32组前后端命令比对，覆盖两pattern、默认/后移、两高度和preview/check/flight命令。Node路线计算回归与三个修改JS的语法检查通过。测试仅构造命令或使用mock，未执行preview、板端节点、SSH、仿真或飞行。

复现：激活既有rl_drone，补充PYTHONPATH=/usr/lib/python3/dist-packages以读取本机已有pexpect（未安装依赖），PATH加入已有Node目录；执行上述unittest文件和 `node tests/waypoint_coverage.test.cjs`。现有会话server未重启，未声称live默认已切换；须会话结束后重载新版后端并刷新网页。板端六份文件是否部署仍需单独核对。

主要变更：workbench.yaml、server.py、wb_board.py、wb_site_sources.py、wb_low_coverage.py、wb_waypoints.py、wb_waypoint_validation.py；web/low-coverage.js、waypoint-bridge.js、waypoint-map.js；tests/test_site_selection.py、site_selection.test.cjs、test_low_coverage_backup.py、test_waypoints.py；六份离线参考。
