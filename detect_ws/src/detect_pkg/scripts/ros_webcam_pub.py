#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
import time

if __name__=="__main__":
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
    rospy.init_node('camera_node', anonymous = True)
    image_pub = rospy.Publisher('/iris_mid360/camera/rgb/image_raw', Image, queue_size = 1)
    compressed_pub = rospy.Publisher('/iris_mid360/camera/rgb/image_compressed', CompressedImage, queue_size = 1)

    while not rospy.is_shutdown():
        ret, frame = capture.read()
        if ret:
            # 获取实际图像尺寸
            height, width, channels = frame.shape
            
            # 将图像尺寸压缩为原来的一半
            print(width , height)
            new_width = width // 2
            new_height = height // 2
            resized_frame = cv2.resize(frame, (new_width, new_height))
            
            # 发布原始图像话题
            ros_frame = Image()
            header = Header(stamp = rospy.Time.now())
            header.frame_id = "Camera"
            ros_frame.header = header
            ros_frame.width = new_width
            ros_frame.height = new_height
            ros_frame.encoding = "bgr8"
            ros_frame.step = new_width * channels  # 正确的步长计算
            ros_frame.data = resized_frame.tobytes()
            image_pub.publish(ros_frame)
            
            # 发布压缩图像话题
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            compressed_msg.data = cv2.imencode('.jpg', resized_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            compressed_pub.publish(compressed_msg)

    capture.release()
    print("quit successfully!")
