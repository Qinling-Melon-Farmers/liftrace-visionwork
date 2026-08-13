# toudi3/toudi4 Gazebo 完整资产包

版本：2026-08-13

该 ZIP 用于在另一台 ROS Noetic + Gazebo Classic 11 + PX4 SITL 开发机上恢复
`toudi3.world`/`toudi4_copy.world` 及当前单下视相机 + MID360 仿真机架。

## 目录

- `world/`：`toudi3.world`、`toudi4_copy.world`；
- `models/`：world 外部 URI 依赖的 big_box3、big_box4、juniper_Tree、五类
  标准靶、red_cross、landing_h；
- `models/iris`、`models/gps`、`models/mid360`：新机架的底层模型依赖；
- `models/iris_mid360_downward_camera`：当前单下视相机组合机架；
- `launch/`：当前 toudi4 生成点和完整联调入口参考；
- `route/`：toudi3/toudi4 固定路线与 toudi4 无靶标坐标覆盖策略；
- `truth/`：toudi4 Gazebo world 绝对坐标真值目录。

## 安装

```bash
unzip toudi4_gazebo_assets_complete_20260813.zip
cd toudi4_gazebo_assets_complete_20260813
export GAZEBO_MODEL_PATH="$PWD/models:${GAZEBO_MODEL_PATH:-}"
```

推荐先直接使用上述独立目录，不覆盖目标机已有模型。若确认需要长期安装，再手工将
缺失模型合并到 `~/.gazebo/models/`。

PX4/Astra 的 Gazebo 插件仍需在目标机上编译安装。本包只包含模型、网格、材质、
贴图、world 和路线配置，不包含已编译 `.so`。

## 坐标约定

toudi4 主 H 世界坐标为 `(-0.493412,-1.772690)`，是当前默认飞机 spawn 点。
PX4/MAVROS/`camera_init` 会在此建立本地 `(0,0)`，因此 toudi4 任务坐标使用：

```text
local = world - (-0.493412,-1.772690)
```

返航与 LAND 目标仍是本地 `(0,0)`。旧 toudi3 回归必须显式使用 world spawn `(0,0)`。

## H 资产

`models/landing_h/` 完整包含：

- `model.sdf`、`model.config`；
- `materials/scripts/landing_h.material`；
- `materials/textures/Hjiangluo.png`。

toudi4 主 H 和 H clone 均引用材质 `Mark/LandingH`。
