# TOUDI3 Gazebo 靶标模型资产

本资产包包含 `toudi3.world` 所需的五类标准靶标模型，以及一个独立的红十字随机靶标模型。

## 模型目录

安装目标：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
```

包含：

```text
dibao/         # pillbox，1×1 m，Mark/Diffuse1
qiaoliang/     # bridge，1×1 m，Mark/Diffuse3
tanke/         # tank，1×1 m，Mark/Diffuse4
zhangpeng/     # tent，1×1 m，Mark/Diffuse5
zhuangjiache/  # panzer/car，1×1 m，Mark/Diffuse6
red_cross/     # 红十字，0.35×0.35 m，Mark/RedCross
```

## 纹理来源

- 五类标准靶标使用 `model-refine/dibao(1).png`、`qiaoliang(1).png`、
  `tanke(1).png`、`zhangpeng(1).png`、`zhuangjiache(1).png`，均为手工居中裁切的
  `600×600` PNG。
- 原始未裁切 PNG 保留在仓库 `model-refine/`，未被覆盖。
- 红十字使用 `hongshizi.png`；模型几何尺寸严格为 `0.35×0.35×0.005 m`，不能
  沿用标准靶标的 `1×1 m`。
- 分发包中的这些裁切源图位于 `source_reference/`，模型目录中的纹理已经按对应
  材质文件名复制好。

## 使用

```bash
export GAZEBO_MODEL_PATH=/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH
gazebo /home/xhj/liftrace/Desktop_patrol_uav_ws-patrol_planner/toudi3.world
```

`toudi3.world` 目前内嵌五类标准靶标几何，并通过 `model://.../materials` 加载材质；
`red_cross` 是独立模型，不在 world 中固定放置一个红十字。需要随机红十字时，应通过
Gazebo spawn 或生成 world 的方式设置 pose，并记录随机种子和真值坐标。

## 验证状态

已通过 XML/SDF 解析检查、五类 world 材质 URI 检查和六个模型的尺寸检查；尚未启动
完整 PX4/Gazebo 飞行链路做画面验收。
