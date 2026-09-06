# R56：按 1 m H 与安装相机修正取景高度

本轮飞行参数仅改两处：最后 H 航点及 external_landing.capture_height 从 0.75 m 同步到 1.60 m。前三门坐标/高度、旧 POST_DELIVERY_ROUTE 状态机、Fast-Planner 和 H 识别/稳定帧/下降/AUTO.LAND 判据不变。

实际 H 模型为 1×1 m、厚 0.005 m，外圈占满纹理直径；机体至相机为 -0.08588 m。离线名义投影在机体高 0.75 / 0.956194 / 1.60 m 时，外圈半径分别约 550.24 / 419.13 / 240.32 px。前两项出画且超过原 300 px 半径上限；1.60 m 能完整取景。

使用源码中未修改的 detectLandingPad / validateHStructure 函数及当前 YAML 参数离线编译，三个重投影输入的结果依次为未检出、未检出、检出，3/3 符合预期。重投影使用实际靶纹理，但不是 R55 相机录像；不模拟全部光照/姿态/镜头条件。

R55 首段 bag（0.409—28.128 ROS 秒）无 Image/CompressedImage，已有 515 条 CameraInfo；实时 ROS 图显示 raw 由 /gazebo 发布，记录器订阅的 compressed 无发布者。缺录不能全部归于末段 F 盘耗尽。证据见 r55_camera_recording_proof.json。

本提交是飞行前修复：完整 R56 结果待本次唯一授权整场实跑。R55 继续保持 INCOMPLETE，不推断为 PASS。
