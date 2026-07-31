#!/home/orangepi/miniconda3/envs/QRcode/bin/python3.8
import rospy
import sys
sys.path.append('/home/orangepi/.local/share/Trash/files/miniconda3/envs/craic/lib/python3.8/site-packages')
import cv2
import pyrealsense2 as rs
import sys
import numpy as np
import time
from sensor_msgs.msg import Image
from detect_pkg.msg import ImageInfo

class RealsenseNode:
    def __init__(self):
        self.image_pub = rospy.Publisher('image', ImageInfo, queue_size=10)

        # RealSense Pipeline Configuration
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        
        # Filters
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filling_filter = rs.hole_filling_filter()

        self.max_depth = 4.0  # meters
        self.view_image = False

    def numpy_to_imgmsg(self, img, encoding="bgr8"):
        """Convert numpy array to sensor_msgs/Image without cv_bridge"""
        msg = Image()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        
        if len(img.shape) == 3:
            msg.encoding = encoding
            msg.step = img.shape[2] * img.shape[1]
        else:  # Depth image
            msg.encoding = "16UC1"
            msg.step = 2 * img.shape[1]
        
        msg.data = img.tobytes()
        msg.header.stamp = rospy.Time.now()
        return msg

    def aligning(self):
        try:
            while not rospy.is_shutdown():
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    continue

                # Apply filters
                depth_frame = self.spatial_filter.process(depth_frame)
                depth_frame = self.temporal_filter.process(depth_frame)
                depth_frame = self.hole_filling_filter.process(depth_frame)

                # Convert to numpy arrays
                color_img = np.asanyarray(color_frame.get_data())
                depth_img = np.asanyarray(depth_frame.get_data())
                depth_img = np.where((depth_img / 1000.0) > self.max_depth, 0, depth_img)

                # Create and publish message
                msg = ImageInfo()
                msg.header.stamp = rospy.Time.now()
                msg.color_image = self.numpy_to_imgmsg(color_img, "bgr8")
                msg.depth_image = self.numpy_to_imgmsg(depth_img, "16UC1")
                self.image_pub.publish(msg)

                # Visualization
                if self.view_image:
                    depth_colormap = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_img, alpha=0.03),
                        cv2.COLORMAP_JET
                    )
                    combined = np.hstack((color_img, depth_colormap))
                    cv2.imshow('Color (Left) vs Depth (Right)', combined)
                    cv2.waitKey(1)
                    
        finally:
            self.pipeline.stop()

if __name__ == '__main__':
    rospy.init_node('realsense_node')
    node = RealsenseNode()
    node.aligning()
    rospy.spin()
