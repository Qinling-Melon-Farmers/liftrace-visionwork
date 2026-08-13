/**
******************************************************************************
  * @file           : base_link2map_tf.cpp
  * @author         : ${USER}
  * @brief          : None
  * @attention      : None
  * @date           : 2025.9.28
  ******************************************************************************
  */

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"

geometry_msgs::PoseStamped current_pose;

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr pose_message) {
    current_pose = *pose_message;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_detection");
    ros::NodeHandle node("~");
    ros::Subscriber pose_sub = node.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 1, pose_callback);
    tf2_ros::TransformBroadcaster broadcaster;

    current_pose.pose.orientation.w = 1;

    geometry_msgs::TransformStamped tfs;
    // geometry_msgs::TransformStamped tf_static;
    tfs.header.frame_id = "map";
    tfs.child_frame_id = "base_link";
    int seq = 0;
    ros::Rate rate(50);
    while (ros::ok()) {
        ros::spinOnce();
        tfs.header.seq = seq++;
        tfs.header.stamp = ros::Time::now();
        tfs.transform.translation.x = current_pose.pose.position.x;
        tfs.transform.translation.y = current_pose.pose.position.y;
        tfs.transform.translation.z = current_pose.pose.position.z;
        tfs.transform.rotation = current_pose.pose.orientation;
        // broadcaster.sendTransform(tf_static);
        broadcaster.sendTransform(tfs);
        rate.sleep();
    }


    return 0;
}
