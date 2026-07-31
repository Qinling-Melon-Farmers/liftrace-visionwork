#!/home/orangepi/miniconda3/envs/QRcode/bin/python3.8
import rospy
import sys
sys.path.append('/home/orangepi/.local/share/Trash/files/miniconda3/envs/craic/lib/python3.8/site-packages')
import cv2 as cv 
import numpy as np
import math
from scipy import stats
from sensor_msgs.msg import Image
from detect_pkg.msg import CircleInfo, ImageInfo

class CircleDetector:
    def __init__(self):
        self.circles_pub = rospy.Publisher("detected_circle", CircleInfo, queue_size=1)
        
        # 可视化开关
        self.view_cv_image = True
        self.view_depth_image_color = True

        # 圆检测参数
        self.min_r = 30    # 最小圆半径(像素)
        self.max_r = 100   # 最大圆半径(像素) 
        self.minDist = 50  # 圆之间的最小距离
        self.param1 = 50   # 边缘检测阈值
        self.param2 = 55   # 累加器阈值
        self.point_number = 36  # 圆周采样点数
        self.w, self.h = 320, 240  # 图像处理尺寸

        # 相机内参
        self.intrinsics = np.array([
            [227.4951, 0.0, 159.0326],
            [0.0, 303.2167, 126.7319],
            [0, 0, 1.0]
        ])

        # 坐标滤波参数
        self.coord_buffer = []  # 坐标缓存队列
        self.buffer_size = 5   # 每收集5个坐标处理一次

    def imgmsg_to_numpy(self, msg):
        """将ROS Image消息转为numpy数组"""
        if msg.encoding == '16UC1':  # 深度图
            return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        elif msg.encoding == 'bgr8':  # 彩色图
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        else:
            raise ValueError("不支持的图像格式: " + msg.encoding)

    def ProcessRGBImage(self, rgb_image):
        """RGB图像预处理"""
        gray = cv.cvtColor(rgb_image, cv.COLOR_BGR2GRAY)
        return cv.medianBlur(gray, 5)

    def DetectCircle(self, rgb_image):
        """使用霍夫变换检测圆形"""
        processed = self.ProcessRGBImage(rgb_image)
        return cv.HoughCircles(processed, cv.HOUGH_GRADIENT, 1, self.minDist,
                             param1=self.param1, param2=self.param2,
                             minRadius=self.min_r, maxRadius=self.max_r)

    def GetDepth(self, x, y, radius, depth_image):
        """在圆周上采样获取深度值"""
        depth_values = []
        points = []
        for i in range(self.point_number):
            angle = 2 * math.pi * i / self.point_number
            px = int(x + radius * math.cos(angle))
            py = int(y + radius * math.sin(angle))
            if 0 <= px < depth_image.shape[1] and 0 <= py < depth_image.shape[0]:
                val = depth_image[py, px]
                if val > 0:  # 忽略无效深度
                    depth_values.append(val)
                    points.append((px, py))

        if not depth_values:
            return 0

        depth = stats.mode(depth_values).mode[0]
        
        # 可视化深度采样点
        if self.view_depth_image_color:
            colored = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
            for pt in points:
                cv.circle(colored, pt, 3, (0, 255, 0), -1)
            cv.imshow('Depth Points', colored)
            cv.waitKey(1)
        
        return int(depth)

    def calculate_filtered_average(self, data_list):
        """计算去除最大最小值后的平均值"""
        if len(data_list) < 3:
            return sum(data_list)/len(data_list) if data_list else 0
        
        # 去除一个最大值和一个最小值
        sorted_data = sorted(data_list)
        trimmed_data = sorted_data[1:-1]  # 去掉第一个(最小)和最后一个(最大)
        return sum(trimmed_data)/len(trimmed_data)

    def publish_filtered_coordinates(self):
        """发布缓冲区内坐标的平均值"""
        if len(self.coord_buffer) == 0:
            return
            
        # 计算平均值
        avg_x = -sum([c[0] for c in self.coord_buffer]) / len(self.coord_buffer)
        avg_y = -sum([c[1] for c in self.coord_buffer]) / len(self.coord_buffer)
        avg_depth = sum([c[2] for c in self.coord_buffer]) / len(self.coord_buffer)
        avg_radius = sum([c[3] for c in self.coord_buffer]) / len(self.coord_buffer)
        
        # 创建并发布消息
        circle_info = CircleInfo()
        circle_info.x = int(avg_x * 1000)   # 转换为毫米并取整
        circle_info.y = int(avg_y * 1000)  # 转换为毫米并取整
        circle_info.depth = int(avg_depth * 1000)  # 转换为毫米并取整
        circle_info.radius = int(avg_radius * 1000)  # 转换为毫米并取整
        self.circles_pub.publish(circle_info)
        print(circle_info.x, circle_info.y, circle_info.depth)
        # 清空缓冲区
        self.coord_buffer = []

    def CircleTransformCoordinate(self, x, y, radius, depth):
        """2D像素坐标转3D世界坐标"""
        if depth == 0:
            return
            
        # 转换为米为单位
        x_c = (x - self.intrinsics[0][2]) * depth / self.intrinsics[0][0] / 1000.0  # 除以1000转换为米
        y_c = (y - self.intrinsics[1][2]) * depth / self.intrinsics[1][1] / 1000.0  # 除以1000转换为米
        depth_m = depth / 1000.0  # 深度也转换为米
        radius_m = radius * depth / self.intrinsics[0][0] / 1000.0  # 半径转换为米
        
        # 添加到坐标缓冲区
        self.coord_buffer.append((x_c, y_c, depth_m, radius_m))
        
        # 当缓冲区满时发布平均值
        if len(self.coord_buffer) >= self.buffer_size:
            self.publish_filtered_coordinates()

    def DrawCircle(self, image, circles):
        """在图像上绘制检测到的圆形"""
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv.circle(image, (i[0], i[1]), i[2], (0, 255, 0), 2)  # 绘制圆形
                cv.circle(image, (i[0], i[1]), 2, (0, 0, 255), 3)     # 绘制圆心

    def CircleDetect(self, image_msg):
        """主检测流程"""
        # 转换ROS消息为numpy数组
        rgb_image = self.imgmsg_to_numpy(image_msg.color_image)
        depth_image = self.imgmsg_to_numpy(image_msg.depth_image)
        
        # 统一图像尺寸
        rgb_image = cv.resize(rgb_image, (self.w, self.h))
        depth_image = cv.resize(depth_image, (self.w, self.h))

        # 检测圆形
        circles = self.DetectCircle(rgb_image)
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                depth = self.GetDepth(x, y, r, depth_image)
                self.CircleTransformCoordinate(x, y, r, depth)

        # 可视化检测结果
        if self.view_cv_image:
            self.DrawCircle(rgb_image, circles)
            cv.imshow('Detected Circles', rgb_image)
            cv.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('circle_detector')
    detector = CircleDetector()
    rospy.Subscriber("image", ImageInfo, detector.CircleDetect)
    rospy.spin()
