#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
圆环断裂修复测试脚本
用于验证降落检测中圆环断裂修复功能的效果
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt

def create_broken_circle_test():
    """创建带有断裂的圆环测试图像"""
    # 创建空白图像
    img = np.ones((400, 400, 3), dtype=np.uint8) * 255
    
    # 绘制完整的黑色圆环
    cv2.circle(img, (200, 200), 100, (0, 0, 0), 20)
    
    # 创建断裂的圆环
    broken_img = img.copy()
    
    # 在圆环上创建几个断裂
    # 断裂1：顶部
    cv2.rectangle(broken_img, (180, 80), (220, 120), (255, 255, 255), -1)
    
    # 断裂2：右侧
    cv2.rectangle(broken_img, (280, 180), (320, 220), (255, 255, 255), -1)
    
    # 断裂3：底部
    cv2.rectangle(broken_img, (180, 280), (220, 320), (255, 255, 255), -1)
    
    # 断裂4：左侧
    cv2.rectangle(broken_img, (80, 180), (120, 220), (255, 255, 255), -1)
    
    return broken_img

def repair_broken_circle(mask):
    """修复断裂的圆环"""
    repaired_mask = mask.copy()
    
    # 1. 使用更大的结构元素进行闭运算，连接断裂
    kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    cv2.morphologyEx(repaired_mask, repaired_mask, cv2.MORPH_CLOSE, kernel_large)
    
    # 2. 使用圆形结构元素进行多次闭运算
    for i in range(3):
        kernel_circle = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        cv2.morphologyEx(repaired_mask, repaired_mask, cv2.MORPH_CLOSE, kernel_circle)
    
    # 3. 使用距离变换来填充小的断裂
    dist_transform = cv2.distanceTransform(repaired_mask, cv2.DIST_L2, 5)
    
    # 4. 阈值化距离变换结果
    dist_thresh = cv2.threshold(dist_transform, 0.7 * cv2.countNonZero(repaired_mask) / (repaired_mask.shape[0] * repaired_mask.shape[1]), 255, cv2.THRESH_BINARY)[1]
    dist_thresh = dist_thresh.astype(np.uint8)
    
    # 5. 合并原始掩码和修复的掩码
    cv2.bitwise_or(repaired_mask, dist_thresh, repaired_mask)
    
    return repaired_mask

def test_circle_repair():
    """测试圆环断裂修复功能"""
    # 创建测试图像
    test_img = create_broken_circle_test()
    
    # 转换为灰度图
    gray = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)
    
    # 创建掩码（黑色区域）
    _, mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
    
    # 修复断裂的圆环
    repaired_mask = repair_broken_circle(mask)
    
    # 寻找轮廓
    contours_original, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_repaired, _ = cv2.findContours(repaired_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 创建可视化结果
    result_img = test_img.copy()
    
    # 绘制原始轮廓（红色）
    cv2.drawContours(result_img, contours_original, -1, (0, 0, 255), 2)
    
    # 绘制修复后的轮廓（绿色）
    cv2.drawContours(result_img, contours_repaired, -1, (0, 255, 0), 2)
    
    # 显示结果
    plt.figure(figsize=(15, 5))
    
    plt.subplot(1, 4, 1)
    plt.imshow(cv2.cvtColor(test_img, cv2.COLOR_BGR2RGB))
    plt.title('原始断裂圆环')
    plt.axis('off')
    
    plt.subplot(1, 4, 2)
    plt.imshow(mask, cmap='gray')
    plt.title('原始掩码')
    plt.axis('off')
    
    plt.subplot(1, 4, 3)
    plt.imshow(repaired_mask, cmap='gray')
    plt.title('修复后掩码')
    plt.axis('off')
    
    plt.subplot(1, 4, 4)
    plt.imshow(cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB))
    plt.title('修复效果对比\n(红色:原始轮廓, 绿色:修复后轮廓)')
    plt.axis('off')
    
    plt.tight_layout()
    plt.savefig('circle_repair_test.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    print(f"原始轮廓数量: {len(contours_original)}")
    print(f"修复后轮廓数量: {len(contours_repaired)}")
    
    # 计算轮廓面积
    original_area = sum(cv2.contourArea(c) for c in contours_original)
    repaired_area = sum(cv2.contourArea(c) for c in contours_repaired)
    
    print(f"原始轮廓总面积: {original_area:.2f}")
    print(f"修复后轮廓总面积: {repaired_area:.2f}")
    print(f"面积改善比例: {(repaired_area - original_area) / original_area * 100:.2f}%")

if __name__ == "__main__":
    test_circle_repair() 