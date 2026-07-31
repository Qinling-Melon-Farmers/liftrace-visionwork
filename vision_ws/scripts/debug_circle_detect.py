#!/usr/bin/env python3
"""诊断蓝色圆环检测：HSV + 形态学 + 椭圆拟合 + 可视化"""
import cv2
import numpy as np
import sys, os

img_path = sys.argv[1] if len(sys.argv) > 1 else "/home/xhj/liftrace/vision_ws/test_data/image/IMG_20250319_100015.jpg"
img = cv2.imread(img_path)
h, w_img = img.shape[:2]
print(f"Image: {w_img}x{h}")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# HSV 蓝色区间 (同 circle_detector.yaml)
mask = cv2.inRange(hsv, (90, 80, 80), (130, 255, 255))
print(f"Blue mask pixels before morphology: {cv2.countNonZero(mask)}")

# 形态学 (同 circle_detector_node.cpp)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
print(f"Blue mask pixels after morphology:  {cv2.countNonZero(mask)}")

contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
print(f"Contours found: {len(contours)}")

def unpack_ellipse(e):
    """兼容 OpenCV 3/4 fitEllipse 不同返回格式"""
    if hasattr(e, 'center'):
        return e.center, e.size
    else:
        return np.array(e[0]), (e[1][0], e[1][1])

# 打印所有轮廓详情
print("\n--- All contours ---")
for i, c in enumerate(contours):
    area = cv2.contourArea(c)
    pts = len(c)
    try:
        ellipse = cv2.fitEllipse(c)
        center, (ew, eh) = unpack_ellipse(ellipse)
        ar = min(ew, eh) / max(ew, eh) if max(ew, eh) > 0 else 0
        r = (ew + eh) / 4.0
        ellipse_ok = True
    except:
        ew = eh = ar = r = 0
        ellipse_ok = False
        center = (0, 0)

    flags = []
    if pts < 15: flags.append("pts")
    if area < 50: flags.append("area")
    if ar < 0.85: flags.append(f"ar={ar:.2f}")
    if r < 10 or r > 300: flags.append(f"r={r:.1f}")

    print(f"  [{i}] area={area:.0f} pts={pts} ellipse={ellipse_ok} "
          f"size=({ew:.0f},{eh:.0f}) ar={ar:.3f} r={r:.1f} "
          f"filters={flags if flags else 'OK'}")

# 保存扩展掩码调试图
dbg = img.copy()
for i, c in enumerate(contours):
    if len(c) >= 15 and cv2.contourArea(c) > 100:
        cv2.drawContours(dbg, [c], -1, (0, 255, 0), 2)
        try:
            ellipse = cv2.fitEllipse(c)
            cv2.ellipse(dbg, ellipse, (0, 0, 255), 2)
        except:
            pass

out_path = os.path.join(os.path.dirname(img_path), "circle_diagnostic.png")
cv2.imwrite(out_path, dbg)
print(f"\nSaved {out_path}")
