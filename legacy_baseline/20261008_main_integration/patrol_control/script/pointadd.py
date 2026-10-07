#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from collections import deque
import threading

class PointCloudAccumulator:
    def __init__(self):
        rospy.init_node('pointcloud_accumulator')
        
        # Parameters
        self.window_size = rospy.Duration(rospy.get_param('~window_size', 15.0))
        self.target_hz = rospy.get_param('~target_hz', 10.0)
        
        # Circular buffer
        self.cloud_buffer = deque(maxlen=int(self.target_hz * self.window_size.to_sec()))
        self.buffer_lock = threading.Lock()
        
        # ROS interfaces
        self.sub = rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_cb, queue_size=10)
        self.pub = rospy.Publisher('/accumulated_cloud', PointCloud2, queue_size=1)
        
        # Timer-based publication
        self.pub_timer = rospy.Timer(rospy.Duration(1.0/self.target_hz), self.publish_cb)

    def cloud_cb(self, msg):
        """Callback with minimal processing"""
        with self.buffer_lock:
            self.cloud_buffer.append(msg)

    def publish_cb(self, event):
        """Timer callback for stable publishing"""
        if not self.cloud_buffer:
            return
            
        with self.buffer_lock:
            # Get newest clouds within window
            cutoff = rospy.Time.now() - self.window_size
            valid_clouds = [cloud for cloud in self.cloud_buffer 
                          if cloud.header.stamp > cutoff]
            
            if not valid_clouds:
                return
                
            # Merge clouds (optimized)
            merged = self.merge_clouds(valid_clouds)
            merged.header.stamp = rospy.Time.now()
            
        self.pub.publish(merged)

    def merge_clouds(self, clouds):
        """Efficient merging using numpy"""
        from sensor_msgs.point_cloud2 import read_points, create_cloud
        import numpy as np
        
        points = []
        for cloud in clouds:
            points.extend(list(read_points(cloud)))
        
        return create_cloud(clouds[0].header, clouds[0].fields, points)

if __name__ == '__main__':
    try:
        PointCloudAccumulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass