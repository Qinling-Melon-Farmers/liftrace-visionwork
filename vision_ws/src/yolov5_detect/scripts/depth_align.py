#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2

class DepthProcessor:
    def __init__(self):
        # 初始化节点
        rospy.init_node('depth_processor', anonymous=True)
        
        # 创建 CVBridge 用于转换 ROS Image 和 OpenCV 图像
        self.bridge = CvBridge()

        # 初始化中心点坐标（默认为图像中心）
        self.center_x = 320  # 默认640x480图像的中心
        self.center_y = 240
        self.has_center = False
        
        # 采样配置 - 改为缓冲区大小而不是总采样次数
        self.buffer_size = rospy.get_param('~buffer_size', 10)  # 缓冲区大小，默认保存最近10次数据
        self.publish_rate = rospy.get_param('~publish_rate', 2.0)  # 发布频率，默认每0.5秒1次（2Hz）
        
        # 存储采样数据 - 使用循环缓冲区
        self.depth_samples = []
        self.center_samples = []
        self.target_positions = []
        
        # 图像尺寸缓存
        self.image_width = 640
        self.image_height = 480
        
        # 订阅对齐后的深度图像话题
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.center_sub = rospy.Subscriber('/detect/circle_center', Point, self.center_callback)

        # 发布器 - 发布特定像素的深度信息
        self.depth_value_pub = rospy.Publisher('/depth/pixel_depth', Float32, queue_size=1)
        self.depth_point_pub = rospy.Publisher('/depth/pixel_point', PointStamped, queue_size=1)
        # 发布器 - 发布x、y偏移信息
        self.real_bias_pub = rospy.Publisher('/detect/real_bias', PoseStamped, queue_size=1)
        
        # 创建定时器，每秒发布一次平均值
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        
        rospy.loginfo(f"Depth Processor node started. Buffer size: {self.buffer_size}, Publish rate: {self.publish_rate} Hz")
    
    def center_callback(self, point):
        """接收目标像素坐标"""
        self.center_x = int(point.x)
        self.center_y = int(point.y)
        self.has_center = True

    def depth_callback(self, data):
        try:
            # 将 ROS Depth Image 消息转换为 OpenCV 图像（数据类型为 uint16）
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            
            # 获取图像尺寸
            height, width = depth_image.shape
            self.image_width = width
            self.image_height = height
            
            # 检查目标像素是否在图像范围内
            if (0 <= self.center_x < width and 0 <= self.center_y < height):
                # 获取指定像素的深度值 (注意：数组索引是 [y, x])
                depth_value_mm = depth_image[self.center_y, self.center_x]
                
                # 处理无效的深度值（0通常表示无效点）
                if depth_value_mm > 0:
                    # 转换为米
                    depth_value_m = depth_value_mm / 1000.0
                    
                    # 获取图像中心点深度
                    center_x_img = width // 2
                    center_y_img = height // 2
                    center_depth_mm = depth_image[center_y_img, center_x_img]
                    center_depth_m = center_depth_mm / 1000.0 if center_depth_mm > 0 else 0
                    
                    if center_depth_m > 0:
                        # 添加到缓冲区
                        self.add_sample(depth_value_m, center_depth_m, (self.center_x, self.center_y))

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def add_sample(self, depth_value, center_depth, position):
        """添加样本到缓冲区，维持固定大小"""
        self.depth_samples.append(depth_value)
        self.center_samples.append(center_depth)
        self.target_positions.append(position)
        
        # 保持缓冲区大小
        if len(self.depth_samples) > self.buffer_size:
            self.depth_samples.pop(0)
            self.center_samples.pop(0)
            self.target_positions.pop(0)

    def timer_callback(self, event):
        """定时器回调函数，每秒发布一次平均值"""
        if len(self.depth_samples) == 0:
            rospy.logdebug("No valid samples in buffer, skipping publish")
            return
            
        # 计算平均值并发布
        self.calculate_and_publish_average()

    def calculate_and_publish_average(self):
        """计算平均值并发布结果"""
        if len(self.depth_samples) == 0:
            return
            
        # 计算平均值
        avg_target_depth = np.mean(self.depth_samples)
        avg_center_depth = np.mean(self.center_samples)
        
        # 使用最近的目标位置的平均值
        avg_target_x = int(np.mean([pos[0] for pos in self.target_positions]))
        avg_target_y = int(np.mean([pos[1] for pos in self.target_positions]))
        
        rospy.logdebug(f"Publishing average values from {len(self.depth_samples)} samples:")
        rospy.logdebug(f"Average target depth: {avg_target_depth:.3f}m")
        rospy.logdebug(f"Average center depth: {avg_center_depth:.3f}m")
        rospy.logdebug(f"Average target position: ({avg_target_x}, {avg_target_y})")
        
        # 计算目标距离并发布
        self.calculate_target_distance(avg_target_x, avg_target_y, avg_target_depth, 
                                     self.image_width, self.image_height, avg_center_depth)
        
        # 发布平均深度值
        depth_msg = Float32()
        depth_msg.data = avg_target_depth
        self.depth_value_pub.publish(depth_msg)
        
        # 发布带坐标的深度点
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "camera_link"
        point_msg.point.x = avg_target_x
        point_msg.point.y = avg_target_y
        point_msg.point.z = avg_target_depth
        self.depth_point_pub.publish(point_msg)

    def calculate_target_distance(self, target_x, target_y, target_depth, width, height, center_depth_m):
        """
        计算图像中心点到目标点的实际3D距离
        
        计算原理：
        1. 深度相机提供每个像素点在Z轴（深度）方向的距离
        2. 通过相机内参，将像素坐标差异转换为实际的X、Y坐标差异
        3. 使用3D勾股定理 √(ΔX² + ΔY² + ΔZ²) 计算真实的3D空间距离
        
        参数:
        - target_x, target_y: 目标点像素坐标
        - target_depth: 目标点深度值(米)
        - width, height: 图像尺寸
        - center_depth_m: 中心点深度值(米)
        """
        # 图像中心点坐标
        center_x_img = width // 2
        center_y_img = height // 2
        
        if center_depth_m <= 0:
            rospy.logdebug("图像中心点深度无效，无法计算距离")
            return
        
        # D435i相机内参近似值
        # 焦距近似为图像宽度的一半（基于典型FOV约69°）
        fx = fy = 205.47  # 近似焦距
        
        # 1. 计算像素偏移（2D图像平面上的差异）
        pixel_offset_x = target_x - center_x_img
        pixel_offset_y = target_y - center_y_img
        pixel_distance = math.sqrt(pixel_offset_x**2 + pixel_offset_y**2)
        
        # 2. 将像素偏移转换为实际物理偏移（3D空间中的X、Y偏移）
        # 使用目标点的深度进行转换
        x_offset = (pixel_offset_x * target_depth) / fx
        y_offset = (pixel_offset_y * target_depth) / fy
        
        # 3. 计算深度差值（Z轴方向的差异）
        depth_diff = target_depth - center_depth_m
        
        # 4. 使用3D勾股定理计算真实的3D空间距离
        # 距离 = √(ΔX² + ΔY² + ΔZ²)
        distance_3d = math.sqrt(x_offset**2 + y_offset**2 + depth_diff**2)
        
        # 发布x、y偏移信息 (使用PoseStamped格式)
        bias_msg = PoseStamped()
        bias_msg.header.stamp = rospy.Time.now()
        bias_msg.header.frame_id = "camera_link"
        
        # 位置信息：相对于中心点的物理偏移
        bias_msg.pose.position.x = -x_offset
        bias_msg.pose.position.y = -y_offset
        bias_msg.pose.position.z = depth_diff
        
        # 方向信息：保持默认方向（无旋转）
        bias_msg.pose.orientation.x = 0.0
        bias_msg.pose.orientation.y = 0.0
        bias_msg.pose.orientation.z = 0.0
        bias_msg.pose.orientation.w = 1.0
        
        self.real_bias_pub.publish(bias_msg)
        
        # 输出详细计算信息（降低日志级别以减少输出频率）
        rospy.logdebug(f"\n{'='*60}")
        rospy.logdebug(f"中心点到目标点的3D距离计算 (基于{len(self.depth_samples)}次采样的平均值)")
        rospy.logdebug(f"{'='*60}")
        rospy.logdebug(f"图像中心点: ({center_x_img}, {center_y_img}), 平均深度: {center_depth_m:.3f}m")
        rospy.logdebug(f"目标点: ({target_x}, {target_y}), 平均深度: {target_depth:.3f}m")
        rospy.logdebug(f"相机焦距(近似): fx={fx:.1f}, fy={fy:.1f}")
        rospy.logdebug(f"像素偏移: ({pixel_offset_x:+.0f}, {pixel_offset_y:+.0f}) = {pixel_distance:.1f}像素")
        rospy.logdebug(f"物理偏移: X={x_offset:+.3f}m, Y={y_offset:+.3f}m, Z={depth_diff:+.3f}m")
        rospy.logdebug(f"🎯 中心点到目标点的3D距离: {distance_3d:.3f}m")
        rospy.logdebug(f"📡 已发布PoseStamped偏移数据到 /detect/real_bias")
        rospy.logdebug(f"{'='*60}\n")

if __name__ == '__main__':
    try:
        dp = DepthProcessor()
        rospy.spin() # 保持节点运行，等待回调函数触发
    except rospy.ROSInterruptException:
        rospy.loginfo("Depth Processor node shutting down...")
        pass