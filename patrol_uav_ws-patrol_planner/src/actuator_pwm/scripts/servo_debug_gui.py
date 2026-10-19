#!/usr/bin/env python3
"""
实机舵机调试 GUI 客户端（远程电脑运行）。

依赖：Python 3 + tkinter + rospy (ROS1)。
  pip install rospy (或已有 ROS 环境)

使用：
  1. 设置 ROS_MASTER_URI 指向 Orange Pi:
        export ROS_MASTER_URI=http://<orangepi-ip>:11311
  2. 确保 Orange Pi 上已启动:
        roslaunch actuator_pwm launch_all.launch
        rosrun actuator_pwm servo_debug_node.py
  3. 运行本 GUI:
        python3 servo_debug_gui.py
"""

import math
import os
import sys
import threading
import time

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except ImportError:
    print("需要 Python 3 + tkinter。请确认系统安装了 python3-tk。")
    sys.exit(1)

try:
    import rospy
    from std_msgs.msg import Float32MultiArray
    from patrol_control.srv import SetServoDuty, SetServoDutyRequest
except ImportError:
    print("需要 ROS 环境 (rospy)。请 source 你的 ROS workspace。")
    sys.exit(1)


# ============== 常量 (与后端保持一致) ==============

DUTY_MIN_NS = 500_000
DUTY_MAX_NS = 2_500_000
ANGLE_MIN_DEG = 0.0   # 对应 DUTY_MIN
ANGLE_MAX_DEG = 180.0  # 对应 DUTY_MAX

SLOT_INFO = [
    {"slot": 1, "name": "Rear (后)",    "color": "#e74c3c"},
    {"slot": 2, "name": "Right (右)",   "color": "#27ae60"},
    {"slot": 3, "name": "Left (左)",    "color": "#2980b9"},
]

# SlotCalibration.h 里的参考值
INITIAL_DUTY = [1_700_000, 1_000_000, 1_100_000]   # locked
RELEASE_DUTY = [2_100_000, 2_100_000, 2_100_000]   # release
MID_DUTY = [1_500_000, 1_500_000, 1_500_000]       # 中值


# ============== 工具函数 ==============

def duty_to_angle(duty_ns):
    """PWM duty ns -> 角度 (线性映射)。"""
    if duty_ns <= DUTY_MIN_NS:
        return ANGLE_MIN_DEG
    if duty_ns >= DUTY_MAX_NS:
        return ANGLE_MAX_DEG
    ratio = (duty_ns - DUTY_MIN_NS) / (DUTY_MAX_NS - DUTY_MIN_NS)
    return ANGLE_MIN_DEG + ratio * (ANGLE_MAX_DEG - ANGLE_MIN_DEG)


def angle_to_duty(deg):
    """角度 -> PWM duty ns (线性映射)。"""
    if deg <= ANGLE_MIN_DEG:
        return DUTY_MIN_NS
    if deg >= ANGLE_MAX_DEG:
        return DUTY_MAX_NS
    ratio = (deg - ANGLE_MIN_DEG) / (ANGLE_MAX_DEG - ANGLE_MIN_DEG)
    return int(DUTY_MIN_NS + ratio * (DUTY_MAX_NS - DUTY_MIN_NS))


def ns_to_ms(ns):
    return f"{ns / 1_000_000:.2f} ms"


# ============== ROS 桥接 (后台线程) ==============

class ROSBridge:
    """在独立线程中运行 rospy 并管理 service / topic 连接。"""

    def __init__(self):
        self.connected = threading.Event()
        self.status_lock = threading.Lock()
        self.last_status = None  # [rear_duty, rear_en, right_duty, right_en, left_duty, left_en]
        self.service = None
        self._thread = None
        self._initialized = False

    def start(self):
        self._thread = threading.Thread(target=self._run_rospy, daemon=True)
        self._thread.start()

    def _run_rospy(self):
        try:
            rospy.init_node("servo_debug_gui_client", anonymous=True, disable_rostime=True)
            # 等待后端服务出现 (最多 10 秒)
            rospy.loginfo("等待后端服务 /SetServoDuty ...")
            try:
                rospy.wait_for_service("/SetServoDuty", timeout=15.0)
            except rospy.ROSException:
                rospy.logwarn("后端服务未在 15s 内出现；GUI 仍可打开，但调用会失败")
            self.service = rospy.ServiceProxy("/SetServoDuty", SetServoDuty)
            rospy.Subscriber("/servo_debug/status", Float32MultiArray, self._on_status)
            self._initialized = True
            self.connected.set()
            rospy.loginfo("ROS Bridge 已连接。")
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        except Exception as exc:
            print(f"ROS Bridge 异常: {exc}", file=sys.stderr)

    def _on_status(self, msg):
        with self.status_lock:
            self.last_status = list(msg.data)  # 6 floats

    def call_set_duty(self, slot, duty_ns, hold=False):
        """调用 SetServoDuty 服务。返回 (success, message, actual_duty)。"""
        if self.service is None:
            return False, "ROS 服务未连接", 0
        try:
            req = SetServoDutyRequest()
            req.slot = slot
            req.duty_cycle_ns = int(duty_ns)
            req.hold = bool(hold)
            resp = self.service(req)
            return bool(resp.success), str(resp.message), int(resp.actual_duty_ns)
        except rospy.ServiceException as exc:
            return False, f"Service call failed: {exc}", 0

    def disable_all(self):
        """快速将全部三个通道设为 MIN 后立刻 disable (hold=False)。"""
        for slot in (1, 2, 3):
            self.call_set_duty(slot, DUTY_MIN_NS, hold=False)


# ============== GUI ==============

class ServoDebugApp:
    """舵机调试 GUI 主窗口。"""

    def __init__(self, root):
        self.root = root
        self.root.title("Orange Pi Servo Debug Tool")
        self.root.geometry("720x680")
        self.root.configure(bg="#2c3e50")

        # ROS
        self.ros = ROSBridge()
        self.ros.start()

        # 本地滑块变量
        self.slider_vars = []      # 每通道 IntVar (duty ns)
        self.hold_vars = []        # 每通道 BooleanVar
        self.angle_labels = []     # 每通道 angle Label 引用
        self.ms_labels = []        # 每通道 ms Label 引用
        self.status_duty_labels = []   # 状态区 duty label
        self.status_en_labels = []     # 状态区 enable label

        self._build_ui()
        self._poll_status()   # 开始定时刷新状态

    # ---------- UI 构建 ----------

    def _build_ui(self):
        # 顶部连接状态条
        top = tk.Frame(self.root, bg="#34495e", pady=6)
        top.pack(fill="x")
        self.conn_label = tk.Label(top, text="● 连接中...",
                                   fg="#f1c40f", bg="#34495e",
                                   font=("Arial", 11, "bold"))
        self.conn_label.pack(side="left", padx=12)
        self.master_label = tk.Label(top,
                                     text=f"ROS_MASTER_URI={os.environ.get('ROS_MASTER_URI', 'unknown')}",
                                     fg="#bdc3c7", bg="#34495e", font=("Arial", 9))
        self.master_label.pack(side="right", padx=12)

        # 三个舵机通道卡片
        channels_frame = tk.Frame(self.root, bg="#2c3e50")
        channels_frame.pack(fill="x", padx=10, pady=6)

        for i, info in enumerate(SLOT_INFO):
            card = self._build_channel_card(channels_frame, info, i)
            card.pack(fill="x", pady=4)

        # 预设按钮区
        preset_frame = tk.LabelFrame(self.root, text=" 预设 (对全部三通道同时生效) ",
                                     fg="#ecf0f1", bg="#2c3e50",
                                     font=("Arial", 10, "bold"))
        preset_frame.pack(fill="x", padx=10, pady=6)

        btn_cfg = [
            ("Initial (Locked)", INITIAL_DUTY, "#3498db"),
            ("Release",          RELEASE_DUTY, "#e67e22"),
            ("Mid (1.5ms)",      MID_DUTY,     "#9b59b6"),
            ("Min (0.5ms)",      [DUTY_MIN_NS]*3, "#7f8c8d"),
            ("Max (2.5ms)",      [DUTY_MAX_NS]*3,  "#c0392b"),
        ]
        for j, (label, duties, color) in enumerate(btn_cfg):
            btn = tk.Button(preset_frame, text=label, bg=color, fg="white",
                            font=("Arial", 10), relief="flat",
                            command=lambda d=duties: self._apply_preset(d))
            btn.grid(row=0, column=j, padx=4, pady=6, sticky="we")
            preset_frame.columnconfigure(j, weight=1)

        # 安全控制区
        safety_frame = tk.LabelFrame(self.root, text=" 安全控制 ",
                                     fg="#ecf0f1", bg="#2c3e50",
                                     font=("Arial", 10, "bold"))
        safety_frame.pack(fill="x", padx=10, pady=6)

        tk.Button(safety_frame, text="⚠ 全部 DISABLE (最小脉宽后立即关闭)",
                  bg="#c0392b", fg="white", font=("Arial", 11, "bold"),
                  relief="flat", padx=12, pady=8,
                  command=self._safety_disable_all).pack(fill="x", padx=8, pady=6)

        # 实时状态区
        status_frame = tk.LabelFrame(self.root, text=" 实际状态 (后端上报，10Hz) ",
                                     fg="#ecf0f1", bg="#2c3e50",
                                     font=("Arial", 10, "bold"))
        status_frame.pack(fill="x", padx=10, pady=(6, 10))

        for i, info in enumerate(SLOT_INFO):
            row = tk.Frame(status_frame, bg="#2c3e50")
            row.pack(fill="x", padx=8, pady=2)
            tk.Label(row, text=f"[{info['slot']}] {info['name']:14s}",
                     fg=info["color"], bg="#2c3e50",
                     font=("Consolas", 11, "bold")).pack(side="left")
            tk.Label(row, text="duty: ", fg="#bdc3c7", bg="#2c3e50").pack(side="left")
            d_label = tk.Label(row, text="—", fg="#ecf0f1", bg="#2c3e50",
                               font=("Consolas", 11))
            d_label.pack(side="left")
            self.status_duty_labels.append(d_label)

            tk.Label(row, text="   enable: ", fg="#bdc3c7", bg="#2c3e50").pack(side="left")
            e_label = tk.Label(row, text="—", fg="#ecf0f1", bg="#2c3e50",
                               font=("Consolas", 11, "bold"))
            e_label.pack(side="left")
            self.status_en_labels.append(e_label)

    def _build_channel_card(self, parent, info, index):
        card = tk.Frame(parent, bg="#34495e", bd=0, highlightbackground=info["color"],
                        highlightthickness=2)
        card.pack_propagate(False)

        # header
        header = tk.Frame(card, bg="#34495e")
        header.pack(fill="x", padx=8, pady=(6, 2))
        tk.Label(header, text=f"Slot {info['slot']} — {info['name']}",
                 fg=info["color"], bg="#34495e",
                 font=("Arial", 12, "bold")).pack(side="left")

        # duty 滑块 + 数值
        row = tk.Frame(card, bg="#34495e")
        row.pack(fill="x", padx=8, pady=4)

        duty_var = tk.IntVar(value=int(INITIAL_DUTY[index]))
        self.slider_vars.append(duty_var)

        tk.Label(row, text="Duty (ns):", fg="#ecf0f1", bg="#34495e",
                 font=("Arial", 10)).pack(side="left")

        scale = tk.Scale(row, from_=DUTY_MIN_NS, to=DUTY_MAX_NS, orient="horizontal",
                         variable=duty_var, resolution=50000,
                         bg="#34495e", fg="#ecf0f1", troughcolor="#7f8c8d",
                         highlightthickness=0, length=420,
                         showvalue=True, command=lambda v, idx=index: self._on_slider(idx))
        scale.pack(side="left", fill="x", expand=True, padx=8)

        angle_label = tk.Label(row, text="—", fg="#ecf0f1", bg="#34495e",
                               font=("Consolas", 10))
        angle_label.pack(side="left", padx=4)
        self.angle_labels.append(angle_label)

        ms_label = tk.Label(row, text="—", fg="#95a5a6", bg="#34495e",
                            font=("Consolas", 9))
        ms_label.pack(side="left")
        self.ms_labels.append(ms_label)

        # bottom row: Hold + Send + Quick
        bottom = tk.Frame(card, bg="#34495e")
        bottom.pack(fill="x", padx=8, pady=(0, 6))

        hold_var = tk.BooleanVar(value=False)
        self.hold_vars.append(hold_var)
        tk.Checkbutton(bottom, text="保持输出 (hold)", variable=hold_var,
                       fg="#f1c40f", bg="#34495e", selectcolor="#2c3e50",
                       activebackground="#34495e",
                       font=("Arial", 9)).pack(side="left")

        tk.Button(bottom, text="▶ 发送", bg=info["color"], fg="white",
                  font=("Arial", 11, "bold"), relief="flat", padx=14, pady=2,
                  command=lambda idx=index: self._send_single(idx)).pack(side="left", padx=8)

        # 快速按钮 (只动这个通道)
        quick = tk.Frame(bottom, bg="#34495e")
        quick.pack(side="right")
        for label, duty in [("Min", DUTY_MIN_NS), ("Mid", 1_500_000),
                            ("Max", DUTY_MAX_NS), ("Lock", INITIAL_DUTY[index]),
                            ("Rels", RELEASE_DUTY[index])]:
            tk.Button(quick, text=label, bg="#7f8c8d", fg="white",
                      font=("Arial", 9), relief="flat", padx=6, pady=1,
                      command=lambda d=duty, idx=index: self._quick_set(idx, d)).pack(side="left", padx=2)

        # 初始化显示
        self._on_slider(index)
        return card

    # ---------- 交互处理 ----------

    def _on_slider(self, index):
        """滑块值变化时更新标签显示。"""
        duty = int(self.slider_vars[index].get())
        angle = duty_to_angle(duty)
        self.angle_labels[index].config(text=f"{angle:5.1f}°")
        self.ms_labels[index].config(text=f"[{ns_to_ms(duty)}]")

    def _send_single(self, index):
        """发送单个通道的当前滑块值。"""
        slot = index + 1
        duty = int(self.slider_vars[index].get())
        hold = bool(self.hold_vars[index].get())

        # 安全检查
        if not (DUTY_MIN_NS <= duty <= DUTY_MAX_NS):
            messagebox.showerror("范围错误",
                                 f"Duty {duty} 超出安全范围 [{DUTY_MIN_NS}, {DUTY_MAX_NS}]")
            return

        success, msg, actual = self.ros.call_set_duty(slot, duty, hold=hold)
        if success:
            self._flash_msg(f"OK: slot{slot} duty={actual}", "#27ae60")
        else:
            messagebox.showerror("发送失败", f"slot {slot}: {msg}")
            self._flash_msg(f"FAIL: slot{slot} - {msg}", "#c0392b")

    def _quick_set(self, index, duty_ns):
        """快速按钮：设置滑块值并立即发送。"""
        self.slider_vars[index].set(int(duty_ns))
        self._on_slider(index)
        self._send_single(index)

    def _apply_preset(self, duties):
        """对三个通道同时发送预设 duty 值。"""
        if not messagebox.askyesno("确认",
                                    f"将三通道同时设置为:\n  rear={ns_to_ms(duties[0])}\n  right={ns_to_ms(duties[1])}\n  left={ns_to_ms(duties[2])}\n\n是否继续？"):
            return
        for i, duty in enumerate(duties):
            self.slider_vars[i].set(int(duty))
            self._on_slider(i)
            success, msg, actual = self.ros.call_set_duty(i + 1, duty, hold=False)
            if not success:
                messagebox.showerror("预设失败", f"slot {i+1}: {msg}")
                break
        else:
            self._flash_msg("预设应用完成 ✓", "#27ae60")

    def _safety_disable_all(self):
        """紧急全部 disable。"""
        if not messagebox.askyesno("⚠ 安全警告",
                                    "立即将三通道设为最小脉宽并关闭输出？\n此操作不可撤销。"):
            return
        self.ros.disable_all()
        self._flash_msg("三通道已 DISABLE", "#c0392b")

    def _flash_msg(self, text, color):
        """在顶部连接区短暂显示操作结果。"""
        orig = self.conn_label.cget("text")
        orig_fg = self.conn_label.cget("fg")
        self.conn_label.config(text=text, fg=color)
        self.root.after(2000, lambda: self.conn_label.config(text=orig, fg=orig_fg))

    # ---------- 定时刷新 ----------

    def _poll_status(self):
        """每 100ms 检查 ROS 连接状态和后端状态。"""
        # 更新连接状态
        if self.ros.connected.is_set():
            self.conn_label.config(text="● 已连接", fg="#27ae60")
        else:
            self.conn_label.config(text="● 连接中...", fg="#f1c40f")

        # 更新状态显示
        with self.ros.status_lock:
            status = self.ros.last_status  # [r_d, r_en, ri_d, ri_en, l_d, l_en]

        if status is not None and len(status) >= 6:
            for i in range(3):
                duty = int(status[i * 2])
                en = bool(status[i * 2 + 1])
                self.status_duty_labels[i].config(
                    text=f"{duty:>8d} ns  ({ns_to_ms(duty)})  {duty_to_angle(duty):.1f}°")
                self.status_en_labels[i].config(
                    text="ON" if en else "OFF",
                    fg="#27ae60" if en else "#7f8c8d")
        else:
            for d in self.status_duty_labels:
                d.config(text="—")
            for e in self.status_en_labels:
                e.config(text="—", fg="#ecf0f1")

        self.root.after(100, self._poll_status)


# ============== main ==============

def main():
    root = tk.Tk()
    try:
        # 让 ttk 使用更现代的主题 (如果可用)
        style = ttk.Style()
        for theme in ("clam", "vista", "default"):
            try:
                style.theme_use(theme)
                break
            except tk.TclError:
                continue
    except Exception:
        pass

    app = ServoDebugApp(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
