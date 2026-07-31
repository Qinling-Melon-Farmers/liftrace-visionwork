#!/usr/bin/env python3
"""Phase B 集成测试：算法、消息、编译产物验证。"""
import subprocess, time, sys, os
import numpy as np
import cv2

ROS_WS = "/home/xhj/liftrace/vision_ws"
TEST_DATA = "/home/xhj/liftrace/vision_ws/test_data"

def ros_source():
    return f"source /opt/ros/noetic/setup.bash && source {ROS_WS}/devel/setup.bash"

def run(cmd, timeout=10, use_ros=False):
    """执行命令并返回 stdout/stderr"""
    if use_ros:
        cmd = f"{ros_source()} && {cmd}"
    r = subprocess.run(cmd, shell=True,
                       capture_output=True, text=True, timeout=timeout,
                       executable="/bin/bash")
    return r.stdout.strip(), r.stderr.strip()

passed = 0
failed = 0
def check(name, condition, detail=""):
    global passed, failed
    if condition:
        print(f"  [PASS] {name}")
        passed += 1
    else:
        print(f"  [FAIL] {name} {detail}")
        failed += 1

# =====================================================
# Test 1: Circle Detector Algorithm (offline)
# =====================================================
print("=" * 60)
print("Test 1: Circle Detector Algorithm (offline)")
print("=" * 60)

img = cv2.imread(f"{TEST_DATA}/circle_test.png")
assert img is not None, "circle_test.png not found"
print(f"  Loaded image: {img.shape[1]}x{img.shape[0]}")

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, (90, 80, 80), (130, 255, 255))
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

best_area = 0
best_center = None
best_r = 0
for c in contours:
    if len(c) < 15: continue
    area = cv2.contourArea(c)
    if area < 50: continue
    try:
        e = cv2.fitEllipse(c)
        if hasattr(e, 'center'):
            center, (w, h) = e.center, e.size
        else:
            center, (w, h) = np.array(e[0]), (e[1][0], e[1][1])
    except: continue
    if w < 1e-3 or h < 1e-3: continue
    if min(w,h)/max(w,h) < 0.85: continue
    r = (w + h) / 4.0
    if r < 10 or r > 300: continue
    if area > best_area:
        best_area, best_center, best_r = area, center, r

assert best_center is not None, "No circle detected!"
print(f"  Detected: center=({best_center[0]:.1f},{best_center[1]:.1f}) r={best_r:.1f}")
check("Center x within 5px", abs(best_center[0] - 400) < 5, f"got {best_center[0]:.1f}")
check("Center y within 5px", abs(best_center[1] - 300) < 5, f"got {best_center[1]:.1f}")
check("Radius ~120px", abs(best_r - 120) < 5, f"got {best_r:.1f}")

# =====================================================
# Test 2: Cross Detector Algorithm (offline)
# =====================================================
print("\n" + "=" * 60)
print("Test 2: Cross Detector Algorithm (offline)")
print("=" * 60)

img2 = cv2.imread(f"{TEST_DATA}/redcross/1.png")
assert img2 is not None, "redcross/1.png not found"
hsv2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
mask1 = cv2.inRange(hsv2, (0, 50, 50), (10, 255, 255))
mask2_c = cv2.inRange(hsv2, (170, 50, 50), (180, 255, 255))
red_mask = cv2.bitwise_or(mask1, mask2_c)
kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel2)
red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel2)
red_px = cv2.countNonZero(red_mask)
check("Red pixels after morphology > 10000", red_px > 10000, f"got {red_px}")

cnts2, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
best2 = None
for c in cnts2:
    area = cv2.contourArea(c)
    if area < 200 or len(c) < 20: continue
    br = cv2.boundingRect(c)
    ar = min(br[2], br[3]) / max(br[2], br[3])
    if ar < 0.6: continue
    hull = cv2.convexHull(c)
    ha = cv2.contourArea(hull)
    s = area/ha if ha > 0 else 0
    if s < 0.6 or s > 0.85: continue
    M = cv2.moments(c)
    if M['m00'] > 0:
        cx, cy = M['m10']/M['m00'], M['m01']/M['m00']
        if best2 is None or area > best2[0]:
            best2 = (area, cx, cy, s)

check("Found valid red cross", best2 is not None)
if best2:
    area, cx, cy, s = best2
    print(f"  Detected: area={area:.0f} center=({cx:.1f},{cy:.1f}) solidity={s:.3f}")
    check("Solidity 0.6-0.85", 0.6 <= s <= 0.85, f"got {s:.3f}")

# =====================================================
# Test 3: Message Import
# =====================================================
print("\n" + "=" * 60)
print("Test 3: Message Import")
print("=" * 60)
out, err = run(f"{ros_source()} && python3 -c \"from uav_vision.msg import TargetDetection, TargetDetectionArray, TargetCandidate, TargetCandidateArray, DropOffset, DropReady; print('OK')\"")
check("6 messages importable", "OK" in out, err[:100])

# =====================================================
# Test 4: Build Artifacts
# =====================================================
print("\n" + "=" * 60)
print("Test 4: Build Artifacts")
print("=" * 60)
out, _ = run(f"ls {ROS_WS}/devel/lib/uav_vision/ 2>/dev/null")
check("cross_detector_node binary", "cross_detector_node" in out)
check("circle_detector_node binary", "circle_detector_node" in out)

out, _ = run(f"ls {ROS_WS}/devel/lib/python3/dist-packages/uav_vision/msg/ 2>/dev/null")
check("6 msg Python files generated", "_TargetDetection.py" in out and "_TargetCandidate.py" in out and "_DropOffset.py" in out and "_DropReady.py" in out and "_TargetCandidateArray.py" in out and "_TargetDetectionArray.py" in out)
check("No stray action messages", "_TargetDetectionAction" not in out)

# Check Python scripts installed
out, _ = run(f"ls {ROS_WS}/devel/lib/uav_vision/ 2>/dev/null")
check("detect_compat_bridge.py installed", "detect_compat_bridge.py" in out)
check("drop_aligner.py installed", "drop_aligner.py" in out)

# =====================================================
# Test 5: Launch File Syntax
# =====================================================
print("\n" + "=" * 60)
print("Test 5: Launch File Syntax")
print("=" * 60)
for launch_file in ["cross_detection.launch", "circle_detection.launch", "phase_b.launch", "uav_vision.launch"]:
    path = f"{ROS_WS}/src/uav_vision/launch/{launch_file}"
    out, err = run(f"python3 -c \"import xml.etree.ElementTree as ET; ET.parse('{path}'); print('XML valid')\"")
    check(launch_file, "XML valid" in out, err[:80])

# =====================================================
# Test 6: YAML Config Validation
# =====================================================
print("\n" + "=" * 60)
print("Test 6: YAML Config Files")
print("=" * 60)
for yaml_file in ["default.yaml", "cross_detector.yaml", "circle_detector.yaml", "drop_aligner.yaml"]:
    path = f"{ROS_WS}/src/uav_vision/config/{yaml_file}"
    out, err = run(f"python3 -c \"import yaml; yaml.safe_load(open('{path}')); print('OK')\"")
    check(yaml_file, "OK" in out, err[:80])

# Check enable_debug_image defaults to false (AGENTS.md constraint)
out, _ = run(f"python3 -c \"import yaml; d=yaml.safe_load(open('{ROS_WS}/src/uav_vision/config/default.yaml')); print(d.get('enable_debug_image'))\"")
check("enable_debug_image defaults to False", "False" in out, f"got '{out}'")

# =====================================================
print("\n" + "=" * 60)
print(f"RESULTS: {passed} passed, {failed} failed of {passed+failed}")
print("=" * 60)

if failed > 0:
    sys.exit(1)
else:
    print("\nAll Phase B verification tests passed!")
