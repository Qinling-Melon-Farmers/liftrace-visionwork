#!/usr/bin/env python3
"""诊断十字检测 v2：形态学连接碎片 + 可视化"""
import cv2
import numpy as np

img = cv2.imread("/home/xhj/liftrace/vision_ws/test_data/redcross/1.png")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 红色双区间
mask1 = cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
mask2 = cv2.inRange(hsv, (170, 50, 50), (180, 255, 255))
mask = cv2.bitwise_or(mask1, mask2)

# 形态学闭运算连接碎片
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

print("After morphology red pixels:", cv2.countNonZero(mask))

contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f"Contours after morphology: {len(contours)}")

# 排序、过滤
valid = []
for c in contours:
    area = cv2.contourArea(c)
    if area < 100 or len(c) < 10:
        continue
    br = cv2.boundingRect(c)
    w, h = br[2], br[3]
    ar = min(w, h) / max(w, h) if max(w, h) > 0 else 0
    hull = cv2.convexHull(c)
    ha = cv2.contourArea(hull)
    solidity = area / ha if ha > 0 else 0
    hull_pts = len(hull) if hull is not None else 0

    rect = cv2.boundingRect(c)
    rc = (rect[0] + rect[2] / 2, rect[1] + rect[3] / 2)
    top = bot = left = right = False
    for pt in c:
        y, x = pt[0][1], pt[0][0]
        if y < rc[1] - rect[3] * 0.3: top = True
        if y > rc[1] + rect[3] * 0.3: bot = True
        if x < rc[0] - rect[2] * 0.3: left = True
        if x > rc[0] + rect[2] * 0.3: right = True
    cross_like = top and bot and left and right

    valid.append((area, c, ar, solidity, cross_like, br, hull_pts))

valid.sort(key=lambda x: -x[0])
print(f"Valid contours (area>=100, pts>=10): {len(valid)}")
for i, (area, c, ar, s, cross, br, hp) in enumerate(valid[:15]):
    print(f"  c{i}: area={area:.0f}  pts={len(c)}  aspect={ar:.3f}  "
          f"solidity={s:.3f}  hull_pts={hp}  cross_like={cross}  bbox={br}")

# 保存调试图
dbg = img.copy()
for i, (area, c, ar, s, cross, br, hp) in enumerate(valid[:10]):
    if area > 200:
        cv2.drawContours(dbg, [c], -1, (0, 255, 0), 3)
        cv2.putText(dbg, f"a={area:.0f} s={s:.2f}", (br[0], br[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        m = cv2.moments(c)
        if m['m00'] > 0:
            cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
            cv2.circle(dbg, (cx, cy), 5, (0, 0, 255), -1)

cv2.imwrite("/home/xhj/liftrace/vision_ws/test_data/redcross/diagnostic.png", dbg)
print("\nSaved diagnostic.png")
