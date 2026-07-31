# `toudi3.world` 仿真资产与当前状态清单

> 截至 2026-07-14：本文件是资产状态速查表，不是启动入口。详细重建、材质和启动说明以 [`TOUDI3_GAZEBO_ASSET_SETUP_GUIDE.md`](TOUDI3_GAZEBO_ASSET_SETUP_GUIDE.md) 为准；完整仿真操作以 [`docs/TOUDI3_FULL_SIM_GUI_GUIDE.md`](docs/TOUDI3_FULL_SIM_GUI_GUIDE.md) 为准。

## 1. 目的

这份清单用于后续去板端或旧环境补拷 `toudi3.world` 所需资产，避免再次只拷回部分工作区却漏掉 world/model 依赖。

## 2. 当前本机已有资产

当前本机已经存在，可直接复用：

### 2.1 PX4 模型

- `iris_mid360`
- `mid360`
- `big_box3`
- `big_box4`
- `juniper_Tree`

位置：

- `/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/`

### 2.2 可用 launch

- `astra_example.launch`

位置：

- `/home/xhj/PX4-Autopilot/launch/astra_example.launch`

### 2.3 世界文件

- [patrol_uav_ws-patrol_planner/toudi3.world](/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world)

## 3. 当前资产状态

### 3.1 PX4 外部目录缺失的旧 launch

- `patrol_world.launch`

当前 PX4 外部目录不存在：

- `/home/xhj/PX4-Autopilot/launch/patrol_world.launch`

仓库内已重建可维护入口：

- `patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_world.launch`

推荐使用 `roslaunch patrol_control patrol_world.launch`，不再要求把文件复制到
PX4 外部目录。完整 SITL 模式仍要求新版 PX4 包路径可被 `rospack` 解析。

### 3.2 已重建模型目录

这些模型被 `toudi3.world` 直接引用，现已按材质脚本、纹理、`model.config` 和
`model.sdf` 完整重建：

- `dibao/`
- `qiaoliang/`
- `tanke/`
- `zhangpeng/`
- `zhuangjiache/`

另外新增：

- `red_cross/` 独立随机靶标模型，视觉和碰撞尺寸为 `0.35×0.35 m`；
- `landing_h/` H 降落模型，当前仿真假设视觉和碰撞尺寸为 `1×1 m`，放置在
  `toudi3.world` 的 `(0,0)`，与 toudi3 专用航路最终 `Land_point` 对齐。

模型落地位置：

- `/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/`

分发包：

- `/home/xhj/liftrace/toudi3_gazebo_assets_20260714.tar.gz`

引用证据在：

- [toudi3.world](/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world:1205)
- [toudi3.world](/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world:1560)
- [toudi3.world](/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world:1619)
- [toudi3.world](/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world:1678)
- [toudi3.world](/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world:1737)

## 4. `toudi3.world` 依赖总表

| 依赖 | 本机状态 | 说明 |
| --- | --- | --- |
| `iris_mid360` | 已有 | PX4 模型已存在 |
| `mid360` | 已有 | PX4 模型已存在 |
| `big_box3` | 已有 | `toudi3.world` 直接引用 |
| `big_box4` | 已有 | `toudi3.world` 直接引用 |
| `juniper_Tree` | 已有 | `toudi3.world` 直接引用 |
| `dibao` | 已重建 | `Mark/Diffuse1`，模型尺寸 `1×1 m` |
| `qiaoliang` | 已重建 | `Mark/Diffuse3`，模型尺寸 `1×1 m` |
| `tanke` | 已重建 | `Mark/Diffuse4`，模型尺寸 `1×1 m` |
| `zhangpeng` | 已重建 | `Mark/Diffuse5`，模型尺寸 `1×1 m` |
| `zhuangjiache` | 已重建 | `Mark/Diffuse6`，模型尺寸 `1×1 m` |
| `red_cross` | 已新增 | `Mark/RedCross`，模型尺寸 `0.35×0.35 m` |
| `landing_h` | 已新增 | `Mark/LandingH`，仿真假设尺寸 `1×1 m`，固定放置于 `(0,0)` |
| `patrol_world.launch` | 仓库内已重建 | `/home/xhj/liftrace/patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_world.launch`；PX4 外部目录不复制 |

## 5. 资产分发与启动位置

### 5.0 H 贴图格式核验

`landing_h` 使用 `Mark/LandingH` 材质，贴图文件必须是实际 PNG，而不只是 `.png`
扩展名。当前已将原始 JPEG 内容保留为 `model-refine/Hjiangluo_source.jpg`，并把
真实 PNG 同步到 PX4 模型目录：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/landing_h/materials/textures/Hjiangluo.png
```

用 `file` 检查应显示 `PNG image data`。修改后要重启 Gazebo，避免旧材质缓存继续显示
错误贴图。

如果从板端拿到上述资产，建议按下面位置落地：

### 5.1 launch

不再建议复制到 PX4 外部目录。使用仓库内版本：

- `/home/xhj/liftrace/patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_world.launch`

### 5.2 模型目录

复制到：

- `/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/`

当前已形成：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
  dibao/
  qiaoliang/
  tanke/
  zhangpeng/
  zhuangjiache/
  red_cross/
  landing_h/
```

## 6. 去板子上优先查找的路径

优先搜这些位置：

1. 机载 PX4 目录下的 `models/`
2. 当时仿真工作区或 `patrol_sim_ws` 的 `src/models/`
3. 任何包含 `patrol_world.launch` 的目录
4. 任何包含上述 5 个模型名的目录

当前本机已不需要为这六个模型等待板端补拷；其他组员可直接使用根目录分发包。
仍建议拷贝整个模型目录，而不只是单个纹理或脚本文件。

## 7. 模型验证

安装或解压完成后，至少做下面检查：

```bash
ls /home/xhj/liftrace/patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_world.launch
ls /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/dibao
ls /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/qiaoliang
ls /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/tanke
ls /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/zhangpeng
ls /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/zhuangjiache
ls /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/red_cross
ls /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/landing_h
```

world-only 检查不会启动 patrol_control，也不会发布航路点。若只看到飞机不动，这是正常的；
完整旧链应使用：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
bash /home/xhj/liftrace/top_level_scripts/run_toudi3_full_competition_sim_gui_old.sh
```

## 8. 当前判断

当前专用靶标模型和 H 降落模型已在本机恢复，且根目录已有可分发压缩包；旧链 toudi3
无 GUI 回归已能完成起飞、巡航和降落。完整路线默认使用
`patrol_control/config/patrol_toudi3.yaml`，它把 3 个检测点与 world 靶标中心对齐，
并把 H 放在最终降落点。`red_cross` 仍不会自动插入固定比赛位置，因此不能把当前
仿真当作红十字识别或投递验收。
