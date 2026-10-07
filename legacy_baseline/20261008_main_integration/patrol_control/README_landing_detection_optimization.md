# 降落检测系统优化说明

## 优化概述

本次优化主要针对降落检测系统的鲁棒性问题，特别是黑色圆环断裂导致的检测失败问题。

## 主要优化内容

### 1. 圆环断裂修复算法

**问题**：二值化图像中黑色圆环经常出现断裂，导致轮廓检测失败。

**解决方案**：
- 使用多层形态学操作连接断裂区域
- 采用距离变换填充小的断裂
- 自动合并断裂的轮廓
- 增大形态学核大小以处理更大的断裂

**核心函数**：
```cpp
cv::Mat repairBrokenCircle(const cv::Mat& mask);
std::vector<std::vector<cv::Point>> mergeBrokenContours(const std::vector<std::vector<cv::Point>>& contours);
```

### 2. 多尺度检测

**功能**：在不同尺度下检测目标，适应不同距离的降落标记。

**参数**：
- `scale_factors`: "1.0,0.8,1.2,0.6,1.5"
- `min_scale`: 0.5
- `max_scale`: 2.0

### 3. 时间连续性检查

**功能**：利用历史检测结果提高检测稳定性。

**参数**：
- `history_size`: 10
- `position_threshold`: 50.0
- `radius_threshold`: 20.0

### 4. 自适应图像预处理

**功能**：
- 自适应光照补偿（CLAHE）
- 对比度增强
- 噪声抑制（双边滤波）
- 边缘增强

### 5. 放宽检测限制

**参数调整**：
- 黑色检测明度上限：50 → 80
- 圆环半径范围：20-200 → 15-300
- 圆形度阈值：0.4 → 0.3
- H字符检测限制大幅放宽

## 配置文件更新

### 新增参数

```yaml
# 圆环断裂修复
image_preprocessing:
  enable_circle_repair: true
  morph_kernel_size: 5  # 增大核大小

# 多尺度检测
multi_scale:
  enable: true
  scale_factors: "1.0,0.8,1.2,0.6,1.5"

# 时间连续性
temporal_continuity:
  enable: true
  history_size: 10
  position_threshold: 50.0
  radius_threshold: 20.0

# 自适应参数
adaptive_params:
  enable: true
  lighting_compensation: true
  contrast_enhancement: true
```

## 使用方法

### 1. 编译代码
```bash
cd /home/orangepi/patrol_uav_ws-patrol_planner
catkin_make --pkg patrol_control
```

### 2. 启动节点
```bash
roslaunch patrol_control landing_detector_node.launch
```

### 3. 启用检测
```bash
rostopic pub /detect/landing_control std_msgs/Bool "data: true"
```

## 测试工具

### 圆环断裂修复测试
```bash
cd src/patrol_control/script
python3 test_circle_repair.py
```

该脚本会生成测试图像，验证圆环断裂修复功能的效果。

## 性能优化建议

### 如果检测率仍然不够：
1. 进一步放宽 `circularity_threshold` 到 0.2
2. 增大 `morph_kernel_size` 到 7 或 9
3. 调整 `scale_factors` 参数

### 如果误检较多：
1. 启用更严格的时间连续性检查
2. 减小 `position_threshold` 和 `radius_threshold`
3. 收紧黑色检测的HSV参数

### 如果圆环断裂严重：
1. 增大 `morph_kernel_size` 参数
2. 启用 `enable_circle_repair` 功能
3. 调整形态学操作的核大小

## 调试技巧

### 1. 查看可视化窗口
启用 `show_image: true` 可以查看：
- 原始图像和检测结果
- 黑色分割掩码
- 修复前后的对比

### 2. 查看调试信息
启用 `debug: true` 可以查看：
- 检测到的圆环参数
- H字符检测结果
- 时间连续性检查结果

### 3. 参数调优
根据实际环境调整：
- `black_segmentation/v_max`: 控制黑色检测严格程度
- `morph_kernel_size`: 控制形态学操作强度
- `scale_factors`: 控制多尺度检测范围

## 版本信息

- **版本**: 2.0 - 优化版本
- **主要改进**: 圆环断裂修复、多尺度检测、时间连续性
- **兼容性**: 与原有话题接口完全兼容
- **性能**: 检测成功率显著提升，特别是在光照条件不佳时 