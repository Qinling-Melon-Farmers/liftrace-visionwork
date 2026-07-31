#!/usr/bin/env python3
"""验证 review 5 项修复"""
import sys, os, subprocess

ROS_WS = "/home/xhj/liftrace/vision_ws"

def run(cmd, timeout=10):
    full = f"source /opt/ros/noetic/setup.bash && source {ROS_WS}/devel/setup.bash && {cmd}"
    r = subprocess.run(full, shell=True, capture_output=True, text=True,
                       timeout=timeout, executable="/bin/bash")
    return r.stdout.strip(), r.stderr.strip()

p = 0; f = 0
def check(name, cond, detail=""):
    global p, f
    if cond: print(f"  [PASS] {name}"); p += 1
    else: print(f"  [FAIL] {name} {detail}"); f += 1

# ============================================================
# Fix 1: detect_compat_bridge 消息类型
# ============================================================
print("=" * 60)
print("Fix 1: detect_compat_bridge message types")
print("=" * 60)

# /yolo_detect → std_msgs::String (was TargetDetectionArray)
src = open(f"{ROS_WS}/src/uav_vision/scripts/detect_compat_bridge.py").read()
check("/yolo_detect uses String", "String, queue_size=1" in src and "/yolo_detect" in src)
check("no TargetDetectionArray on old topics",
      "self._yolo_detect_pub" not in src or "TargetDetectionArray" not in
      src[src.find("_yolo_detect_pub"):src.find("_yolo_detect_pub")+120])

# /detect/tank_status → geometry_msgs::PoseStamped (was std_msgs::Bool)
check("/detect/tank_status uses PoseStamped",
      'self._tank_status_pub = rospy.Publisher("/detect/tank_status",' in src and
      'PoseStamped' in src)
check("yolo_detect sends single class string (not array)",
      'yolo_str.data = best_class' in src)

# ============================================================
# Fix 2: target_memory confirm_frames 参数化
# ============================================================
print("\n" + "=" * 60)
print("Fix 2: target_memory confirm_frames parameterized")
print("=" * 60)

src2 = open(f"{ROS_WS}/src/uav_vision/scripts/target_memory.py").read()
check("_advance_state uses confirm_frames param",
      "def _advance_state(self, confirm_frames)" in src2)
check("no hardcoded >= 3 in _advance_state",
      "self.observe_count >= confirm_frames" in src2)
check("update() passes confirm_frames",
      "self._advance_state(confirm_frames)" in src2)

# ============================================================
# Fix 3: target_memory CONFIRMED 冷却
# ============================================================
print("\n" + "=" * 60)
print("Fix 3: target_memory CONFIRMED timeout → reject cooldown")
print("=" * 60)

check("was_confirmed saved before age()",
      "was_confirmed = (cand.state == ST_CONFIRMED)" in src2)
check("was_confirmed used for reject",
      "if was_confirmed:" in src2)

# ============================================================
# Fix 4: target_memory priority <= 0 过滤
# ============================================================
print("\n" + "=" * 60)
print("Fix 4: target_memory priority <= 0 filtering")
print("=" * 60)

check("priority > 0 in selected_target selection",
      "self._priority.get(t.class_name, 0) > 0" in src2)

# ============================================================
# Fix 5: drop_aligner CONFIRMED gating
# ============================================================
print("\n" + "=" * 60)
print("Fix 5: drop_aligner confirmed-only gating")
print("=" * 60)

src3 = open(f"{ROS_WS}/src/uav_vision/scripts/drop_aligner.py").read()
check("checks state >= 2 (CONFIRMED)",
      "if t.state >= 2:  # CONFIRMED" in src3)
check("no confirmed target → skip",
      '"no confirmed target"' in src3)

# ============================================================
# 综合：编译 + 语法
# ============================================================
print("\n" + "=" * 60)
print("Integration: Build + Syntax")
print("=" * 60)

out, _ = run("python3 -c \""
    "compile(open('{}/src/uav_vision/scripts/detect_compat_bridge.py').read(), 'x', 'exec'); "
    "compile(open('{}/src/uav_vision/scripts/target_memory.py').read(), 'x', 'exec'); "
    "compile(open('{}/src/uav_vision/scripts/drop_aligner.py').read(), 'x', 'exec'); "
    "print('OK')\"".format(ROS_WS, ROS_WS, ROS_WS))
check("All 3 scripts compile cleanly", "OK" in out, out[:100])

out, _ = run(f"ls {ROS_WS}/devel/lib/uav_vision/ | grep -E 'bridge|memory|aligner'")
check("All Python nodes installed", "detect_compat_bridge.py" in out
      and "target_memory.py" in out and "drop_aligner.py" in out)

# ============================================================
print("\n" + "=" * 60)
print(f"RESULTS: {p} passed, {f} failed of {p+f}")
print("=" * 60)

sys.exit(0 if f == 0 else 1)
