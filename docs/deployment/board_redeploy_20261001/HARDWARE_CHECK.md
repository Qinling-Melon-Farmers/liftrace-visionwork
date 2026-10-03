# 2026-10-01 整机硬件与软件链路检查

板端192.168.3.15，部署目录liftrace_board_trials_20260928。检查运行于地面未解锁，使用memory_only **preview**；没有任务开始、解锁、起飞或舵机动作。

|项目|结果|实际观察|
|---|---|---|
|主机|正常|内核已由部署时5.10升级至6.1.43；需以本次实时检查为准|
|飞控USB/MAVROS|通信正常|ttyACM0，connected=true、armed=false、AUTO.LOITER；没有改模式|
|供电遥测|后续正常|初始0V/未知，后读到25.355V；用户确认当前接外部电源，不能据100%遥测推算电池续航|
|MID360网络|临时修复后正常|enP4p65s0原192.168.2.50，增加192.168.1.100/24后192.168.1.175三次ping全通；原地址保留|
|MID360点云|正常|约10Hz|
|相机|采集正常|video0，1280×720，CameraInfo有效，compressed约10Hz|
|FAST-LIO|运行正常|Odometry约10Hz；启动FC/LIO航向差约72°逐步收敛，地图等稳定后才启动|
|地图|运行正常|camera_init约10Hz，取样16764点，接收时年龄约0.002s|
|RKNN/视觉|链路正常|新内核实际运行通过；8秒YOLO80条、圆环80条、红十字81条，各约10Hz。融合话题约30Hz不是YOLO30fps|
|视觉效果|未验收|取样时三个来源均为空检测；需摆靶检查真实类别/中心与H检测|
|整机预览|READY|先MAPPING_READY，再READY；任务IDLE；navigation_frame_adapter/enable_setpoints=false|
|姿态|记录|一时刻roll约2.35°、pitch约−2.85°，仅静置测量，不是飞行姿态验收|
|舵机配置|部分通过|现场源码较本机已更新，芯片2/3/4地址febf0000/0010/0020与实际匹配；可执行文件晚于源码|
|舵机运行/机构|未验收|重启后通道未导出、export仅root可写；无legacy/Servo_raw服务。服务启动会依次复位三舵机，故本次未启动|

## 雷达网段与重启

这次只添加运行时地址，未改NetworkManager持久配置：

```bash
sudo ip address add 192.168.1.100/24 dev enP4p65s0
```

重启后先用`ip -brief address`检查，已存在则不要重复执行。接口与雷达实物接线确认后再持久化，不能将Wi-Fi地址替换进MID360配置。

## 现场舵机与原部署的差异

当前板端hardware_ws中的源码已经由现场更新：req1后仓Pin16/pwmchip2，req2右仓Pin18/pwmchip3，req3左仓Pin7/pwmchip4；启动会依次复位，然后关闭PWM输出。与本机留存基线不再逐字相同。本次只拷回到logs/hardware_check_20261001/actuator_snapshot，未覆盖现场修改或擅自合并源码。

现场现有初始化脚本为`/home/orangepi/liftrace_deploy/liftrace-visionwork/init_pwm.sh`。该脚本导出2/3/4通道、设置20ms周期、权限和normal极性，不写enable=1；服务节点才执行复位。当前尚未运行该脚本。源码服务回调没有逐项检查PWM写入返回值，因此服务res=true本身不能证明真实机构动作成功。

## 收尾与下一步

预览已SIGINT停止，BAG_CLOSED=PASS；包保留板端`logs/board_memory_only_20261001_180005/flight_debug_0.bag`。只取回文字日志与现场舵机源码快照，没有下载大bag。剩余进程仅roscore/rosout、MAVROS、MID360驱动、相机。

要宣布全硬件就绪，还需现场授权舵机初始化/复位及动作观察、摆靶验证视觉效果。H专项无须舵机，可按操作手册独立执行，但本轮没有进行H实飞。
