# `toudi3.world` 仿真资产补全与靶标照片接入操作说明

## 1. 文档目的

本文档用于完成以下工作：

1. 明确当前 `toudi3.world` 已有与缺失的仿真资产；
2. 从板端或旧环境恢复原比赛场景资产；
3. 在无法找回原靶标模型时，使用现有正上方照片重建五类靶标材质；
4. 配置 Gazebo Classic 的模型搜索路径；
5. 分阶段验证 world、PX4 SITL 与 QGC 联调；
6. 给出常见报错的排查方法。

适用环境：

- Ubuntu / WSL；
- ROS Noetic；
- PX4-Autopilot；
- Gazebo Classic；
- SDF 1.7；
- QGroundControl。

---

## 2. 当前状态与关键结论

### 2.1 当前已有资产

PX4 模型目录：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
```

当前已有：

```text
iris_mid360/
mid360/
big_box3/
big_box4/
juniper_Tree/
```

已有 launch：

```text
/home/xhj/PX4-Autopilot/launch/astra_example.launch
/home/xhj/liftrace/patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_world.launch
```

已有 world：

```text
/home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world
```

### 2.2 当前靶标资产状态

PX4 外部目录中仍缺失旧入口：

```text
/home/xhj/PX4-Autopilot/launch/patrol_world.launch
```

仓库内已经重建可维护版本，推荐通过包名启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/xhj/liftrace/patrol_uav_ws-patrol_planner/devel/setup.bash
roslaunch patrol_control patrol_world.launch
```

完整 PX4 模式前还需按本机新版 PX4 路径注册 `ROS_PACKAGE_PATH`；若只检查
world 与飞机模型，可使用 `start_px4:=false`，不会启动 PX4 或 MAVROS。

原先缺失的五类靶标模型目录已按本指南重建到：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
├── dibao/
├── qiaoliang/
├── tanke/
├── zhangpeng/
└── zhuangjiache/
```

另外新增了独立的红十字模型：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/red_cross/
```

另根据 `model-refine/Hjiangluo.png` 新增 H 降落模型：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/landing_h/
```

该模型使用水平 `1×1 m` 贴图板，当前是仿真工程假设；规则书没有给出 H 贴纸的独立
尺寸。`patrol_uav_ws-patrol_planner/toudi3.world` 将其固定放在 `(0,0)`，与
`patrol_control/config/patrol_toudi3.yaml` 的最终 `Land_point` 对齐。

贴图资产已做 MIME 格式核验：原先的 `model-refine/Hjiangluo.png` 文件内容实际是
JPEG，只是扩展名写成了 `.png`。原始字节保留在
`model-refine/Hjiangluo_source.jpg`，当前 `Hjiangluo.png` 已转换为真实 PNG，并
同步到 PX4 的 `landing_h/materials/textures/`。如果 Gazebo 已经运行，应先关闭旧的
`gzserver/gzclient` 后重新启动，避免 OGRE 继续持有旧材质缓存。可用下面命令检查：

```bash
file /home/xhj/liftrace/model-refine/Hjiangluo.png
file /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/landing_h/materials/textures/Hjiangluo.png
```

两处应均显示 `PNG image data`；世界内模型名应由
`rosservice call /gazebo/get_world_properties` 返回 `landing_h`。

五类标准靶标的独立模型和 world 材质资源使用 `1×1 m`；红十字模型使用规则要求的
`0.35×0.35 m`，不与标准靶标共用尺寸。分发包位于仓库根目录：

```text
/home/xhj/liftrace/toudi3_gazebo_assets_20260714.tar.gz
```

说明：`toudi3.world` 当前仍将五类标准靶标几何内嵌在 world 中，新增模型目录主要
提供其 `model://.../materials` 材质资源和可单独 spawn 的完整模型；`red_cross` 为
独立随机靶标模型，本次不把一个固定红十字位置硬编码进比赛 world；`landing_h` 是
为验证旧降落检测链新增的固定仿真模型。

### 2.3 对 `toudi3.world` 的实际检查结果

`world` 文件已经内嵌了五类靶标的几何体、位置和尺寸，并没有通过 `<include>` 加载完整三维模型。

它实际引用的是以下材质资源：

```text
model://dibao/materials/scripts
model://dibao/materials/textures

model://qiaoliang/materials/scripts
model://qiaoliang/materials/textures

model://tanke/materials/scripts
model://tanke/materials/textures

model://zhangpeng/materials/scripts
model://zhangpeng/materials/textures

model://zhuangjiache/materials/scripts
model://zhuangjiache/materials/textures
```

对应材质名称：

| 靶标目录 | world 中的材质名称 |
| --- | --- |
| `dibao` | `Mark/Diffuse1` |
| `qiaoliang` | `Mark/Diffuse3` |
| `tanke` | `Mark/Diffuse4` |
| `zhangpeng` | `Mark/Diffuse5` |
| `zhuangjiache` | `Mark/Diffuse6` |

因此，五类靶标目前并不需要重新建立坦克、帐篷、桥梁等三维网格。  
只要补齐图片纹理和 OGRE 材质脚本，就可以恢复靶标外观。

---

## 3. 推荐处理路线

按以下优先级执行。

### 路线 A：优先恢复原始资产

如果还能访问板端、旧电脑或旧工作区，优先找回：

```text
patrol_world.launch

dibao/
qiaoliang/
tanke/
zhangpeng/
zhuangjiache/
```

整个目录复制，不要只复制其中某张图片。

优先搜索位置：

```text
~/PX4-Autopilot/**/models/
~/patrol_sim_ws/src/
~/catkin_ws/src/
~/liftrace/
~/.gazebo/models/
```

搜索命令：

```bash
find ~ -type f -name "patrol_world.launch" 2>/dev/null

find ~ -type d \( \
  -name "dibao" -o \
  -name "qiaoliang" -o \
  -name "tanke" -o \
  -name "zhangpeng" -o \
  -name "zhuangjiache" \
\) 2>/dev/null
```

如果找到原始目录，直接复制到：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
```

### 路线 B：使用正上方照片重建靶标资产

如果原始五个目录无法找回，可以使用现有正上方照片制作纹理。

这种方式足以支持：

- Gazebo 相机画面显示；
- YOLO 靶标识别验证；
- 相机视角、飞行高度和识别距离测试；
- 靶标坐标投影与搜索路径仿真。

---

## 4. 靶标照片要求

### 4.1 推荐格式

每类靶标准备一张图片：

```text
dibao.png
qiaoliang.png
tanke.png
zhangpeng.png
zhuangjiache.png
```

推荐参数：

| 项目 | 推荐值 |
| --- | --- |
| 格式 | PNG |
| 色彩 | RGB 或 RGBA |
| 位深 | 8 bit |
| 尺寸 | 1024×1024 px，条件允许可用 2048×2048 px |
| 长宽比 | 1:1 |
| 文件名 | 仅使用英文、数字和下划线 |
| 内容 | 包含完整 1 m×1 m 靶标板图案 |

JPG 也可以使用，但 PNG 更适合保留图案边缘和蓝白圆环细节。

### 4.2 照片预处理

即使照片是正上方拍摄，也建议依次完成：

1. 以靶标板四角为边界裁剪；
2. 将图像校正为正方形；
3. 确保蓝白圆环和中心图案完整；
4. 去掉靶标板外的地面、鞋子、阴影和杂物；
5. 统一五张图片的朝向；
6. 适度校正曝光和白平衡；
7. 避免过度锐化、降噪或 AI 重绘；
8. 输出为标准 RGB PNG。

不要只裁中心的坦克、帐篷等图案。  
应保留实际比赛靶标板上的完整视觉结构，包括外圈和背景。

### 4.3 方向统一

建议将所有图片统一为：

```text
图片上边 = 场地规定的靶标正方向
```

如果比赛规则没有规定方向，也应在团队内部固定一种方向，避免训练数据、Gazebo 纹理和真实场地照片方向混乱。

---

## 5. 模型目录结构

目标根目录：

```text
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
```

五类靶标最终应形成：

```text
models/
├── dibao/
│   ├── model.config
│   ├── model.sdf
│   └── materials/
│       ├── scripts/
│       │   └── dibao.material
│       └── textures/
│           └── dibao.png
├── qiaoliang/
│   ├── model.config
│   ├── model.sdf
│   └── materials/
│       ├── scripts/
│       │   └── qiaoliang.material
│       └── textures/
│           └── qiaoliang.png
├── tanke/
│   ├── model.config
│   ├── model.sdf
│   └── materials/
│       ├── scripts/
│       │   └── tanke.material
│       └── textures/
│           └── tanke.png
├── zhangpeng/
│   ├── model.config
│   ├── model.sdf
│   └── materials/
│       ├── scripts/
│       │   └── zhangpeng.material
│       └── textures/
│           └── zhangpeng.png
└── zhuangjiache/
    ├── model.config
    ├── model.sdf
    └── materials/
        ├── scripts/
        │   └── zhuangjiache.material
        └── textures/
            └── zhuangjiache.png
```

说明：

- 当前 `toudi3.world` 真正需要的是 `materials/scripts` 和 `materials/textures`；
- `model.config` 与 `model.sdf` 对当前内嵌式 world 并非绝对必需；
- 仍建议保留完整模型结构，便于后续通过 `<include>` 单独加载和随机放置。

---

## 6. 手动创建单个靶标模型

下面以 `tanke` 为例。

### 6.1 创建目录

```bash
MODEL_ROOT=/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models

mkdir -p "$MODEL_ROOT/tanke/materials/scripts"
mkdir -p "$MODEL_ROOT/tanke/materials/textures"
```

将处理后的图片复制到：

```bash
cp /你的图片路径/tanke.png \
  "$MODEL_ROOT/tanke/materials/textures/tanke.png"
```

### 6.2 创建 `tanke.material`

文件路径：

```text
$MODEL_ROOT/tanke/materials/scripts/tanke.material
```

内容：

```text
material Mark/Diffuse4
{
  technique
  {
    pass
    {
      lighting on
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1

      texture_unit
      {
        texture tanke.png
        filtering anisotropic
        max_anisotropy 8
      }
    }
  }
}
```

材质名称 `Mark/Diffuse4` 必须与 `toudi3.world` 完全一致。

### 6.3 创建 `model.config`

```xml
<?xml version="1.0"?>
<model>
  <name>tanke</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>

  <author>
    <name>RoboCup Team</name>
  </author>

  <description>
    One-meter square Gazebo target board using the tank target texture.
  </description>
</model>
```

### 6.4 创建 `model.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="tanke">
    <static>true</static>

    <link name="link">
      <pose>0 0 0.0025 0 0 0</pose>

      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 0.005</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>1 1 0.005</size>
          </box>
        </geometry>

        <material>
          <script>
            <uri>model://tanke/materials/scripts</uri>
            <uri>model://tanke/materials/textures</uri>
            <name>Mark/Diffuse4</name>
          </script>
        </material>

        <cast_shadows>true</cast_shadows>
      </visual>
    </link>
  </model>
</sdf>
```

其他四类按同样方式创建，只需替换模型名、图片名和材质名称。

---

## 7. 五类材质文件对照

### 7.1 `dibao.material`

```text
material Mark/Diffuse1
{
  technique
  {
    pass
    {
      lighting on
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1

      texture_unit
      {
        texture dibao.png
        filtering anisotropic
        max_anisotropy 8
      }
    }
  }
}
```

### 7.2 `qiaoliang.material`

```text
material Mark/Diffuse3
{
  technique
  {
    pass
    {
      lighting on
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1

      texture_unit
      {
        texture qiaoliang.png
        filtering anisotropic
        max_anisotropy 8
      }
    }
  }
}
```

### 7.3 `tanke.material`

```text
material Mark/Diffuse4
{
  technique
  {
    pass
    {
      lighting on
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1

      texture_unit
      {
        texture tanke.png
        filtering anisotropic
        max_anisotropy 8
      }
    }
  }
}
```

### 7.4 `zhangpeng.material`

```text
material Mark/Diffuse5
{
  technique
  {
    pass
    {
      lighting on
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1

      texture_unit
      {
        texture zhangpeng.png
        filtering anisotropic
        max_anisotropy 8
      }
    }
  }
}
```

### 7.5 `zhuangjiache.material`

```text
material Mark/Diffuse6
{
  technique
  {
    pass
    {
      lighting on
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1

      texture_unit
      {
        texture zhuangjiache.png
        filtering anisotropic
        max_anisotropy 8
      }
    }
  }
}
```

---

## 8. 使用自动脚本创建目录

配套脚本：

```text
create_toudi3_target_assets.sh
```

准备一个图片目录，例如：

```text
/home/xhj/toudi3_target_photos/
├── dibao.png
├── qiaoliang.png
├── tanke.png
├── zhangpeng.png
└── zhuangjiache.png
```

赋予权限：

```bash
chmod +x create_toudi3_target_assets.sh
```

执行：

```bash
./create_toudi3_target_assets.sh \
  /home/xhj/toudi3_target_photos \
  /home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
```

脚本会：

- 建立五个模型目录；
- 复制五张 PNG；
- 创建对应 `.material`；
- 创建 `model.config`；
- 创建可独立使用的 `model.sdf`；
- 检查缺失图片。

---

## 9. 配置 Gazebo 模型搜索路径

临时配置：

```bash
export GAZEBO_MODEL_PATH=\
/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:\
$GAZEBO_MODEL_PATH
```

确认：

```bash
echo "$GAZEBO_MODEL_PATH"
```

永久配置：

```bash
echo 'export GAZEBO_MODEL_PATH=/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH' \
  >> ~/.bashrc

source ~/.bashrc
```

如果 PX4 自带的 `setup_gazebo.bash` 已经添加了该路径，仍建议用下面命令确认，而不是默认认为配置成功：

```bash
echo "$GAZEBO_MODEL_PATH" | tr ':' '\n'
```

---

## 10. 分阶段验证

不要一开始就同时启动 PX4、Gazebo、MAVROS 和 QGC。  
应先确认 world 与材质本身可以正常加载。

### 10.1 检查文件是否存在

```bash
MODEL_ROOT=/home/xhj/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models

for model in dibao qiaoliang tanke zhangpeng zhuangjiache; do
  echo "===== $model ====="
  find "$MODEL_ROOT/$model" -maxdepth 3 -type f
done
```

### 10.2 检查图片格式

安装 ImageMagick 后：

```bash
identify "$MODEL_ROOT/dibao/materials/textures/dibao.png"
identify "$MODEL_ROOT/qiaoliang/materials/textures/qiaoliang.png"
identify "$MODEL_ROOT/tanke/materials/textures/tanke.png"
identify "$MODEL_ROOT/zhangpeng/materials/textures/zhangpeng.png"
identify "$MODEL_ROOT/zhuangjiache/materials/textures/zhuangjiache.png"
```

应确认：

- 文件可读取；
- 宽高相同；
- 图片不是 CMYK；
- 图片不是损坏文件。

### 10.3 检查 world XML/SDF 语法

```bash
gz sdf -k \
  /home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world
```

若当前 Gazebo 版本不支持该命令，可先使用：

```bash
xmllint --noout \
  /home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world
```

### 10.4 单独启动 Gazebo

```bash
gazebo --verbose \
  /home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world
```

检查五类靶标是否：

- 正常显示；
- 图片不是纯灰色；
- 没有粉色或黑色缺材质效果；
- 方向正确；
- 尺寸约为 1 m×1 m；
- 不会掉落、倾斜或被碰撞推走。

### 10.5 检查终端报错

重点排查：

```text
Unable to find uri
Unable to load material
Cannot locate resource
Missing material
OGRE EXCEPTION
Unable to find texture
```

搜索方式：

```bash
gazebo --verbose \
  /home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world \
  2>&1 | tee /tmp/toudi3_gazebo.log

grep -Ei \
  "unable|missing|error|exception|material|texture|resource" \
  /tmp/toudi3_gazebo.log
```

---

## 11. 当前 `patrol_world.launch` 的使用方式

`patrol_world.launch` 是 PX4、Gazebo 和其他节点的集成启动入口，但它不是验证靶标纹理的必要条件。

仓库内的重建入口位于：

```text
patrol_uav_ws-patrol_planner/src/patrol_control/launch/patrol_world.launch
```

它复用 PX4 的 `posix_sitl.launch`，把 world、机型、SDF 和 Gazebo 模型路径
做成参数，并额外提供 world/model-only 检查模式：

```bash
# 仅检查 toudi3.world 和 iris_mid360 spawn
roslaunch patrol_control patrol_world.launch start_px4:=false

# 完整 PX4 模式
export PX4_ROOT=/home/xhj/PX4-Autopilot
export SITL_GAZEBO=$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export ROS_PACKAGE_PATH=/opt/ros/noetic/share:$ROS_PACKAGE_PATH:$PX4_ROOT:$SITL_GAZEBO
export GAZEBO_MODEL_PATH=$SITL_GAZEBO/models:$GAZEBO_MODEL_PATH
roslaunch patrol_control patrol_world.launch start_px4:=true
```

---

## 12. 对当前 world 的建议修正

### 12.1 固定场地模型

当前五类靶标位于父模型 `3` 内，父模型末尾为：

```xml
<static>0</static>
```

如果父模型中的靶标、树木、箱体均属于固定场地，建议备份后改为：

```xml
<static>true</static>
```

备份：

```bash
cp \
  /home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world \
  /home/xhj/liftrace/patrol_uav_ws-patrol_planner/toudi3.world.bak
```

这样可以避免靶标受到重力、碰撞或初始惯性影响。

### 12.2 统一视觉尺寸和碰撞尺寸

当前五类靶标的视觉尺寸为：

```xml
<size>1 1 0.005</size>
```

部分碰撞尺寸为：

```xml
<size>0.496 0.496 0.005</size>
```

这会造成“看到的是 1 m×1 m，但实际碰撞范围约为 0.5 m×0.5 m”。

如果靶标需要参与碰撞，建议统一为：

```xml
<size>1 1 0.005</size>
```

如果靶标只用于视觉识别，且无人机不会接触地面靶标，可以删除其 `<collision>`，减少不必要的物理计算。

### 12.3 不要直接修改原文件

建议维护两个版本：

```text
toudi3_original.world
toudi3_fixed.world
```

其中：

- `original` 保留原比赛环境；
- `fixed` 用于当前仿真和后续迭代。

---

## 13. QGC 联调说明

QGC 不直接读取：

- `toudi3.world`；
- PNG 靶标纹理；
- Gazebo 模型目录。

QGC 只需要通过 MAVLink 连接 PX4 SITL。

因此正确依赖关系是：

```text
靶标 PNG / 材质
        ↓
Gazebo 加载 toudi3.world
        ↓
Gazebo 与 PX4 SITL 交换传感器和动力学数据
        ↓
PX4 通过 MAVLink 连接 QGC
```

当 Gazebo 中靶标显示异常时，应排查：

- `GAZEBO_MODEL_PATH`；
- 模型目录；
- `.material`；
- PNG；
- world 中的材质名称。

不应先排查 QGC。

---

## 14. 后续加入红十字随机靶标

红十字靶标可以沿用同一方案。本次已经按下面的独立模型结构完成：

建议目录：

```text
red_cross/
├── model.config
├── model.sdf
└── materials/
    ├── scripts/
    │   └── red_cross.material
    └── textures/
        └── hongshizi.png
```

当前 `red_cross/model.sdf` 的视觉和碰撞尺寸均为：

```xml
<size>0.35 0.35 0.005</size>
```

不要沿用五类标准投放区的 `1 1 0.005`。

红十字 PNG 原始尺寸约为 `1280×1273`，模型内纹理已规范为 `1280×1280`，避免
贴到正方形几何后发生比例拉伸；`model-refine/hongshizi.png` 原始文件未修改。

后续若要随机放置，不建议为每次布局手工修改 world。可以通过：

- ROS/Gazebo spawn 服务；
- 启动前 Python 脚本生成 world；
- 修改模型 `<pose>`；
- 维护随机种子和真值坐标文件；

实现重复、可记录的随机场景。

---

## 15. 常见问题

### 15.1 靶标显示为灰色

优先检查：

```bash
echo "$GAZEBO_MODEL_PATH"
ls 模型目录/materials/scripts/
ls 模型目录/materials/textures/
```

同时确认 `.material` 内的材质名与 world 完全一致。

### 15.2 图片上下颠倒或旋转 90°

这是纹理坐标和照片方向问题。  
优先旋转 PNG，不要随意修改整个 world 的模型姿态，否则会同时改变靶标在场地坐标系中的方向。

### 15.3 图片看起来模糊

检查：

- 原图是否被过度压缩；
- 是否使用低分辨率截图；
- Gazebo 相机分辨率；
- 相机高度和视场角；
- PNG 是否只包含很小的有效图案；
- 是否启用各向异性过滤。

### 15.4 World 能打开，但某个模型完全消失

执行：

```bash
gazebo --verbose toudi3.world
```

检查对应 `model://模型名/...` 是否能够解析。

Linux 文件名区分大小写：

```text
tanke.png
Tanke.png
```

是两个不同文件。

### 15.5 `roslaunch px4 patrol_world.launch` 报找不到文件

确认：

```bash
rospack find px4
ls /home/xhj/PX4-Autopilot/launch/patrol_world.launch
```

并重新设置：

```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/xhj/PX4-Autopilot
```

---

## 16. 最终验收清单

### 资产

- [ ] `iris_mid360` 已存在；
- [ ] `mid360` 已存在；
- [ ] `big_box3` 已存在；
- [ ] `big_box4` 已存在；
- [ ] `juniper_Tree` 已存在；
- [x] `dibao` 已补齐；
- [x] `qiaoliang` 已补齐；
- [x] `tanke` 已补齐；
- [x] `zhangpeng` 已补齐；
- [x] `zhuangjiache` 已补齐；
- [x] `red_cross` 已补齐，尺寸为 `0.35×0.35 m`；
- [x] `landing_h` 已补齐，当前仿真假设尺寸为 `1×1 m`，固定于 `(0,0)`；
- [x] `patrol_world.launch` 已恢复或重建。

### 图片

- [x] 模型目录中的五张标准靶纹理使用手工居中裁切的 `*(1).png`，尺寸为 `600×600`，
      避免 800×600 原图贴到 1×1 m 几何后发生拉伸；
- [ ] 完整保留靶标板；
- [ ] 图片方向统一；
- [ ] 图片为 RGB/RGBA PNG；
- [ ] 文件名与模型名一致；
- [ ] 没有中文、空格和大小写错误。

### Gazebo

- [ ] `GAZEBO_MODEL_PATH` 包含 PX4 models 目录；
- [ ] `gazebo --verbose toudi3.world` 能启动；
- [ ] 五类靶标均正常显示；
- [ ] 终端无材质、纹理和 URI 报错；
- [ ] 靶标不会掉落或移动；
- [ ] 靶标实际尺寸符合规则。

### PX4 与 QGC

- [ ] PX4 SITL 正常启动；
- [ ] 飞行器模型正常生成；
- [ ] 传感器数据正常；
- [ ] QGC 能建立 MAVLink 连接；
- [ ] 解锁、起飞和基本控制正常；
- [ ] 机载相机画面能观察到靶标；
- [ ] 视觉节点能接收图像并输出识别结果。

---

## 17. 建议的实际执行顺序

```text
整理五张正上方照片
        ↓
裁剪、校正并命名为 PNG
        ↓
运行资产创建脚本
        ↓
配置 GAZEBO_MODEL_PATH
        ↓
单独启动 toudi3.world
        ↓
修复所有材质与模型报错
        ↓
恢复或重建 patrol_world.launch
        ↓
启动 PX4 SITL + Gazebo
        ↓
连接 QGC
        ↓
接入视觉识别节点
        ↓
测试靶标识别、坐标投影和全场搜索
```
