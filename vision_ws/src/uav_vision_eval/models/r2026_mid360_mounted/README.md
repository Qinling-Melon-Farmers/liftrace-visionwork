# MID360模型装配

传感器ray/IMU坐标保持原值；仅CAD外壳的visual/collision局部z增加0.04m，使IMU位于外壳内、外壳能装在飞控上方。此偏移是网格装配假设，不是新的实测外参。原始网格包围盒z为[-0.0259171,0.03418]m；外层安装平移后，外壳底部约FC+0.01996m，顶部+0.08006m，IMU仍为FC+0.05m。


R62重试：恢复先前成功仿真的publish_pointcloud_type=1，匹配SITL FAST-LIO的PointCloud2输入；实机mid360_hardware.yaml仍用真实CustomMsg，不随之切换。闭合CAD外壳不是光学扫描窗口，原点在外壳内部；仅将该自身外壳对射线透明，保留其全部实体碰撞和接触记录，机体/旋翼及环境遮挡保持。
