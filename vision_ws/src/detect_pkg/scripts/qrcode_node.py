#!/home/orangepi/miniconda3/envs/QRcode/bin/python3.8
import rospy
from pyzbar import pyzbar
from sensor_msgs.msg import Image
from detect_pkg.msg import qrcode  # 替换为你的消息定义路径
import sys
sys.path.append('/home/orangepi/.local/share/Trash/files/miniconda3/envs/craic/lib/python3.8/site-packages')
import cv2
import numpy as np

class QRCodeDetector:
    def __init__(self):
        # 保存二维码信息
        self.qr_info = ""
        self.class1 = ""
        self.class2 = ""
        self.direction = ""
        self.view_image = False 
        # 发布二维码解析结果
        self.pub = rospy.Publisher("qr_info", qrcode, queue_size=10)
    
    def image_callback(self, image_msg):
        # 将ROS图像消息转换为numpy数组
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = np_arr.reshape(image_msg.height, image_msg.width, -1)
            
        # 识别二维码
        decoded_objects = pyzbar.decode(cv_image)

        if decoded_objects:
            data = decoded_objects[0].data.decode("utf-8")
            rospy.loginfo(f"QR Code detected: {data}")
            self.qr_info = data

            # 解析信息，格式：class1,class2,direction
            parts = self.qr_info.split(',')
            if len(parts) == 3:
                self.class1, self.class2, self.direction = parts
        
        # 无论识别与否，发布当前信息
        self.publish_info()

        # 显示图像窗口（如果开启）
        if self.view_image:
            cv2.imshow("QR Code View", cv_image)
            cv2.waitKey(1)

    def publish_info(self):
        qrcodemsg = qrcode()
        qrcodemsg.class1 = self.class1
        qrcodemsg.class2 = self.class2
        qrcodemsg.fallpoint = self.direction
        self.pub.publish(qrcodemsg)
        rospy.loginfo(f"Published QR info: class1={self.class1}, class2={self.class2}, direction={self.direction}")

if __name__ == '__main__':
    qrcode_detector=QRCodeDetector()
    rospy.init_node('qr_code_detector', anonymous=True)
    # 订阅相机图像话题
    rospy.Subscriber("webcam_imgmsg", Image, qrcode_detector.image_callback)
    rospy.spin()
