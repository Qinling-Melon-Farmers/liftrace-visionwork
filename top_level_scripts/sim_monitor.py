#!/usr/bin/env python3
"""sim_monitor: 实时显示 toudi3 仿真飞行状态（起飞/对准/投递/航点/降落）。

用法: python3 sim_monitor.py [logfile]
  - 无参数: 实时订阅 ROS 话题
  - 有参数: 从仿真日志文件中提取关键事件时间线（离线分析）
"""
import re
import sys

# ---------------- 离线日志分析模式 ----------------
KEY_EVENTS = [
    # (正则, 输出标签)
    (r"Ready for takeoff", "起飞就绪"),
    (r"Takeoff detected", "起飞检测"),
    (r"Armed by external", "武装"),
    (r"Next Point number : (\d+)", "航点 {}"),
    (r"Next Point Mode: (\w+)", "模式 {}"),
    (r"Drop action (\d+) completed", "投递 {} 完成"),
    (r"servo_complete.data: 1", "Servo 应答"),
    (r"CrossDetectionDone.*dis_to_next_position: ([\d.]+)", "对准误差 {}m"),
    (r"no valid circle", "无有效圆环"),
    (r"align_ok: (\d)", "align_ok={}"),
    (r"Landing Point setting", "降落点设置"),
    (r"Landing completed, servo status (\w+)", "降落完成 servo={}"),
    (r"mission over", "任务结束"),
    (r"Time up", "对准超时"),
    (r"Landing detection", "降落检测"),
    (r"Error in XmlRpc", "ROS连接错误"),
    (r"Invalid servo", "非法Servo"),
]


def analyze_log(path):
    print("=" * 70)
    print(f"离线日志分析: {path}")
    print("=" * 70)
    with open(path, errors="replace") as f:
        for line in f:
            sim_t = ""
            m = re.search(r", (\d+\.\d+):", line)
            if m:
                sim_t = f"t={m.group(1)}s"
            for pattern, label in KEY_EVENTS:
                m = re.search(pattern, line)
                if m:
                    args = m.groups()
                    rendered = label
                    for a in args:
                        rendered = rendered.replace("{}", a, 1)
                    print(f"  [{sim_t:>10}] {rendered}")
                    break


def monitor_live():
    import rospy
    import std_msgs.msg
    import geometry_msgs.msg

    rospy.init_node("sim_monitor", anonymous=True)

    last_mode = ""

    def on_point_class(msg):
        global last_mode
        modes = {0: "Takeoff", 1: "Run_point", 2: "Aligning", 3: "Land", 4: "Hover"}
        mode = modes.get(msg.data, f"未知{msg.data}")
        if mode != last_mode:
            print(f"  [{rospy.Time.now().to_sec():.0f}] 状态切换 -> {mode}")
            last_mode = mode

    def on_pose(msg):
        p = msg.pose.position
        print(f"  [t={rospy.Time.now().to_sec():.0f}] 位置 ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

    rospy.Subscriber("/detect/point_class", std_msgs.msg.Int8, on_point_class)
    rospy.Subscriber("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped, on_pose)
    rospy.spin()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        analyze_log(sys.argv[1])
    else:
        monitor_live()
