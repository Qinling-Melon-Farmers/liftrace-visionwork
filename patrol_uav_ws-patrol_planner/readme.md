# 1.编译

## 1.1 安装`nlopt`

~~~
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
~~~

## 1.2 安装opencv3

下载opencv3.4.8源码[源码地址](https://github.com/opencv/opencv/releases?page=3)

~~~
cd opencv-3.4.8
mkdir build
cd build
cmake ..
make -j6 #(可以更高)
sudo make install
~~~

## 1.3 编译本程序

~~~shell
git clone https://gitee.com/lulese/patrol_uav_ws.git
cd ~/patrol_uav_ws/ && mkdir src/
mv Fast-Planner FAST_LIO image patrol_control tool CMakeLists.txt ego_planner.zip  readme.md FreeDOM  src/
catkin_make -DROS_EDITION=ROS1
~~~

可能会报错：

~~~
fatal error: fast_lio/Pose6D.h: 没有那个文件或目录
    8 | #include <fast_lio/Pose6D.h>
fatal error: livox_laser_simulation/CustomMsg.h: 没有那个文件或目录
   18 | #include <livox_laser_simulation/CustomMsg.h>
~~~

不用担心，编译几次就好了。

如果还不行，可以把我的文件复制过去

~~~
cp ~/patrol_uav_ws/src/tool/devel_files/CustomMsg.h ~/patrol_uav_ws/devel/include/livox_ros_driver/
cp ~/patrol_uav_ws/src/tool/devel_files/Pose6D.h ~/patrol_uav_ws/devel/include/fast_lio/
~~~

# 2.运行

## 2.1仿真运行

### 2.1.1 配置px4仿真环境

本代码如果在gazebo仿真中使用则需要配置一个带有mid360雷达的无人机模型，且环境中需要一些障碍物，我这里给出一个示例：

<img src="image/2025-05-13 23-23-10 的屏幕截图.png" style="zoom:53%;" />

<img src="image/2025-05-13 23-22-55 的屏幕截图.png" style="zoom:50%;" />

<img src="image/2025-05-13 23-23-46 的屏幕截图.png" style="zoom:52%;" />

<img src="image/2025-05-13 23-23-36 的屏幕截图.png" style="zoom:81%;" />

#### 2.1.1.1 编译功能包

~~~
cd ~/
git clone https://gitee.com/lulese/patrol_sim_ws.git
cd patrol_sim_ws/
catkin_make
~~~

将本功能包加到环境中：

~~~
gedit ~/.bashrc
~~~

在这三行：

~~~
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot/ ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
~~~

上面加上：

~~~
source ~/patrol_sim_ws/devel/setup.bash
~~~

#### 2.1.1.2 复制模型与launch到PX4环境

如果还没配置PX4环境，请跳转到这个教程[ubuntu20.04系统 PX4仿真环境配置教程-超简单](https://blog.csdn.net/woaixiaojiang/article/details/143490014?spm=1001.2014.3001.5502)，配置完成后，将我们的环境加进去，就可以启动本环境了：

对于PX4 1.14及以后的固件：

~~~
cp ~/patrol_sim_ws/src/models/launch/patrol_world.launch   ~/PX4-Autopilot/launch/
cp -r ~/patrol_sim_ws/src/models/iris_mid360/ ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
cp -r ~/patrol_sim_ws/src/models/mid360/ ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
~~~

对于PX4 1.13.3及以前的固件(PX4-Autopilot_1.13.3为PX4仿真文件夹，每个人可能文件夹名不一样)：

~~~
cp ~/patrol_sim_ws/src/models/launch/patrol_world.launch   ~/PX4-Autopilot_1.13.3/launch/
cp -r ~/patrol_sim_ws/src/models/iris_mid360/ ~/PX4-Autopilot_1.13.3/Tools/sitl_gazebo/models/
cp -r ~/patrol_sim_ws/src/models/mid360/ ~/PX4-Autopilot_1.13.3/Tools/sitl_gazebo/models/
~~~

### 2.1.2 启动仿真环境

~~~shell
roslaunch px4 patrol_world.launch
~~~

### 2.1.3 启动建图与定位

本代码使用[fast-lio2](https://github.com/hku-mars/FAST_LIO)定位，[FreeDOM](https://github.com/LC-Robotics/FreeDOM)建图，可以得到鲁棒的定位，和一个剔除动态障碍物鬼影的全局地图，后续使用这个地图进行规划，防止出现碰撞障碍物的情况。

~~~
cd patrol_uav_ws && source ./devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
~~~

### 2.1.4 启动主控程序及规划程序

~~~
cd patrol_uav_ws && source ./devel/setup.bash
roslaunch patrol_control patrol_control_sim.launch
~~~

本代码使用改良后的**fast_planner**：

- 修改了Hybrid A*前端的代价，使得轨迹z轴变化较小；
- 修改重规划的逻辑，改为从当前点位开始；
- 修改了轨迹服务器发布cmd的逻辑，改为发布固定距离后的点位；
- 修改了更改目标点的逻辑，改为找离设定目标点最近的满足要求的点；
- 修改了yaw角规划的逻辑，改为直接从当前yaw平滑过度到目标点的yaw，使用幂函数进行插值。

在主控函数（patrol_control）中，添加了一些功能，比如点位是否检测，降落是否检测，单独设置目标点的yaw角，是否使用规划器等等，均在这个`patrol_sim.yaml`中修改相应参数（有详细注释），`patrol_sim.yaml`中的点位可以复制/删除一行来添加/减少点位。

~~~
# 路点的模式：Pointmode {Takeoff_point , Detect_point, Nothing_point , Land_point};
#                            0             1             2                3 

# 单位：x(m), y(m), z(m), yaw(°), pointmode(Takeoff_point/Detect_point/Nothing_point/Land_point)
waypoints:
  - {x: 0.0, y: 0.0, z: 2.2, yaw: 0.0, pointmode: "Takeoff_point"} # takeoff，必须写在第一个
  - {x: 20.0, y: 0.0, z: 2.0, yaw: 45.0, pointmode: "Nothing_point"}
  - {x: 25.0, y: 0.0, z: 2.0, yaw: 45.0, pointmode: "Nothing_point"}
  - {x: 45.0, y: 1.0, z: 2.0, yaw: 90.0, pointmode: "Nothing_point"}
  - {x: 45.0, y: 32.0, z: 2.0, yaw: 0.0, pointmode: "Nothing_point"}
  - {x: 20.0, y: 25.0, z: 1.5, yaw: 0.0, pointmode: "Land_point"} # land，必须写在最后一个

land_height: 0.3                            #当降落需要检测时，检测目标的固定高度
px4_max_distance: 1.2                       #直接发送点位时，最大变化距离（直接给mavros发送点时决定最大速度）
max_yaw_change: 0.2                         #直接发送点位时，最大变化yaw

# threshould
threshould: 
  takeoff_threshould: 0.3                   #判断是否到达起飞点的阈值
  waypoint_threshould: 0.3                  #判断是否到达路点点的阈值
  aligning_threshould: 0.15                 #判断是否到达对准点的阈值
  landing_threshould: 0.15                  #判断是否到达降落点的阈值
  arrive_yaw_threshould: 0.3                #判断yaw是否到位的阈值

  planner_min_pub_threshould: 0.025         #限制往planner发送目标点的频率
  times_detect_threshould: 40               #路点为对准时，调整到位的次数阈值，判断次数大于这个阈值即认为已经对准
  waypoint_adjust_max_second_threshould: 20 #路点为对准时，最大调整时间阈值，超过阈值直接去下一个点
  land_adjust_max_second_threshould: 10     #路点为对准时，最大调整时间阈值，超过阈值直接降落

# bool switch
switch:
 flag_planner_px4: 0 # 0: planner   1: px4  ，设为0则使用planner在路点之间规划路径，设为1则使用直线在路点之间飞行
 flag_landing_detect: 0 # land detect flag  ，该标志位判断是否需要精确降落
 auto_land: 1 # auto_land flag  ，该标志位判断是否需要切入auto land
~~~

### 2.1.5 效果

<img src="image/1.png" style="zoom:60%;" />

<img src="image/2.png" style="zoom:52%;" />

<img src="image/4.png" style="zoom:80%;" />

<img src="image/5.png" style="zoom:55%;" />

## 2.2实物运行

### 2.2.1 启动传感器

读取飞控

~~~
roslaunch mavros px4.launch
~~~

读取雷达

~~~
roslaunch livox_ros_driver2 msg_MID360.launch
~~~

### 2.2.2 启动建图与定位

~~~
cd patrol_uav_ws && source ./devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
~~~

该 launch 默认同时启动 `fastlio_to_px4_vision`，把 FAST-LIO 的 `/Odometry`
转换为 `/mavros/vision_pose/pose`，由 MAVROS 发送给 PX4 作为外部视觉位置。
首次更新代码后需要重新编译工作空间：

~~~
cd ~/patrol_uav_ws
catkin_make
source ./devel/setup.bash
~~~

如需暂时关闭向 PX4 发送当前位置，可使用：

~~~
roslaunch fast_lio mapping_mid360.launch publish_px4_vision:=false
~~~

正常实物启动顺序不变：先启动 MAVROS 和 Livox 驱动，再启动本 launch，最后启动
`patrol_control_real.launch`。可用以下命令确认数据正在发送：

~~~
rostopic hz /mavros/vision_pose/pose
rostopic echo -n 1 /mavros/vision_pose/pose
~~~

PX4 1.15 需要在 QGroundControl 中设置 `EKF2_EV_CTRL=3`（水平位置位 0 + 垂直位置
位 1），使 EKF2 融合外部视觉三维位置。建议先只融合位置，不启用外部视觉航向；确认坐标方向、原点和高度正确后，
再决定是否融合航向。修改 PX4 参数后重启飞控。

这里的默认imu参数是雷达的imu。

~~~
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/mavros/imu/data"
    time_sync_en: false         # ONLY turn on when external time synchronization is really not possible
    time_offset_lidar_to_imu: 0.0 # Time offset between lidar and IMU calibrated by other algorithms, e.g. LI-Init (can be found in README).
                                  # This param will take effect no matter what time_sync_en is. So if the time offset is not known exactly, please set as 0.0

preprocess:
    lidar_type: 1                # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR, 
    scan_line: 4
    blind: 0.5

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree:    360
    det_range:     100.0
    extrinsic_est_en:  false      # true: enable the online estimation of IMU-LiDAR extrinsic
    extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
    extrinsic_R: [ 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1]
~~~

### 2.2.3 启动主控程序和规划程序

~~~
cd patrol_uav_ws && source ./devel/setup.bash
roslaunch patrol_control patrol_control_real.launch
~~~

# 3. 主要参数说明

> 快速找参数可以在code 中搜索快速查找

前文提到的`patrol_sim.yaml`中关于主控程序的参数，均有详细注释，这里不再阐述。

## 3.1 规划参数

### 3.1.1 膨胀系数sdf_map/obstacles_inflation

位置：`patrol_uav_ws/src/Fast-Planner/fast_planner/plan_manage/launch/kino_algorithm.xml`

~~~
<param name="sdf_map/obstacles_inflation"     value="0.25" /> 
<param name="sdf_map/obstacles_inflation_up"     value="0.2" />  <!-- 往上膨胀距离 -->
<param name="sdf_map/obstacles_inflation_down"     value="0.1" /> <!-- 往下膨胀距离 -->
~~~

表示把障碍物左右前后膨胀0.25米，往上膨胀0.2米，往下膨胀0.1米，那么无人机在规划时就可以看做为质点，相当于把安全距离这个要素在地图中考虑。这里考虑无人机是一个质点偏上的正方体。

### 3.1.2 地图大小map_size

位置：`patrol_uav_ws/src/Fast-Planner/fast_planner/plan_manage/launch/kino_replan.launch`

这里设置了地图大小，规划器只能在这个范围内规划，单位都是m

~~~
<!-- size of map, change the size in x, y, z according to your application -->
<arg name="map_size_x" value="100.0"/>
<arg name="map_size_y" value="100.0"/>
<arg name="map_size_z" value="50.0"/>
~~~

### 3.1.3 点云话题cloud_topic

位置：`patrol_uav_ws/src/Fast-Planner/fast_planner/plan_manage/launch/kino_replan.launch`

~~~
<arg name="cloud_topic" value="/freedom/static_pointcloud"/>
~~~

这个话题来选择把什么点云当做规划的点云地图输入，当前这个使用了全局点云地图，而且是比较稠密的，还可以选择使用freedom的其他话题降低计算量但是可能点云比较稀疏，或者可以选择fastlio的实时点云输出来当做规划地图，但是可能会视野受限。

### 3.1.4 yaw角规划开始距离traj_server/adjust_distance_yaw

位置：`patrol_uav_ws/src/Fast-Planner/fast_planner/plan_manage/launch/kino_replan.launch`

这里使用了距离目标点固定距离才开始调节yaw角到目标yaw角，这个距离就是这个参数，距离越小，调节越急，距离越大，调节越缓慢。

~~~
<!-- yaw角调节开始距离 -->
<param name="traj_server/adjust_distance_yaw" value="4" type="double"/>
~~~

### 3.1.5 目标点预瞄距离traj_server/target_dist

位置：`patrol_uav_ws/src/Fast-Planner/fast_planner/plan_manage/launch/kino_replan.launch`

~~~
<!-- 发送目标点的预瞄距离 -->
<param name="traj_server/target_dist" value="0.3" type="double"/>
~~~

由于本程序用的类似于纯跟踪算法，即发送轨迹中距离无人机当前固定距离的点为目标点，所以这个距离不能设置太大，设置太大会导致速度很快，而且有撞障碍物的风险，也不必设置太小，太小会导致速度很小。

### 3.1.6 局部地图大小sdf_map/local_update_range_*

位置：`patrol_uav_ws/src/Fast-Planner/fast_planner/plan_manage/launch/kino_algorithm.xml`

~~~
<param name="sdf_map/local_update_range_x"  value="8.5" /> 
<param name="sdf_map/local_update_range_y"  value="8.5" /> 
<param name="sdf_map/local_update_range_z"  value="20.5" /> 
~~~

如果想降低计算量，就减小局部地图，相应的还需要修改前端的路径搜索最大长度：

位置：`patrol_uav_ws/src/Fast-Planner/fast_planner/plan_manage/launch/kino_algorithm.xml`

~~~
<param name="search/horizon" value="7.0" type="double"/>
~~~

这个值最好不大于局部地图大小。

### 3.1.7 搜索步长search/search_time_resolution

位置：`patrol_uav_ws/src/Fast-Planner/fast_planner/plan_manage/launch/kino_algorithm.xml`

~~~
<param name="search/search_time_resolution" value="0.3" type="double"/><!-- 搜索时间分辨率 -->
<param name="search/search_acc_resolution" value="0.25" type="double"/><!-- 搜索加速度分辨率 -->
~~~

越小找的越仔细，计算量越大

## 3.2 地图参数

### 3.2.1 全局地图更新大小

位置：`patrol_uav_ws/src/FreeDOM/config/mid360.yaml`

~~~
sensor:
  min_range: 0.2    # Ignore points closer than this distance
  max_range: 20.0   # Ignore points farther than this distance
  min_z: -20.0      # Ignore points with z-value lower than this (relative to LiDAR)
  max_z: 20.0       # Ignore points with z-value higher than this (relative to LiDAR)
~~~

可以设置这个来滤除过远的点云，降低远处点云测量不准的影响。

## 3.3 检测参数

### 3.3.1 检测结果接收话题

位置：`patrol_uav_ws/src/patrol_control/launch/patrol_control_real.launch`

位置：`patrol_uav_ws/src/patrol_control/launch/patrol_control_sim.launch`

~~~
<remap from="/detect/waypoint_mark" to="/detect/waypoint_mark"/><!-- 后面字符串填上你的话题，消息类型：geometry_msgs::PoseStamped -->
<remap from="/detect/land_mark" to="/detect/land_mark"/>        <!-- 后面字符串填上你的话题，消息类型：geometry_msgs::PoseStamped -->
~~~

# 4. 录制bag
~~~
rosbag record /tf /fastplanner/goal /mavros/setpoint_position/local /freedom/static_pointcloud /planning_vis/trajectory /cloud_registered /sdf_map/occupancy_inflate /freedom/static_voxels
~~~
