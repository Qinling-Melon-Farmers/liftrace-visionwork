/**
  ******************************************************************************
  * @file           : simple_cross_detect.cpp
  * @author         : nanoha
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/1
  ******************************************************************************
  */

#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "cv_bridge/cv_bridge.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



int s_min, s_max;
int v_min, v_max;
double DEPTH_THRESHOLD;
double CONTOURS_AREA_THRESHOLD;
bool debug = false;

bool is_red_cross(const cv::Mat &mask, const std::vector<std::vector<cv::Point> > &contours) {
    static ros::Publisher binary_pub = ros::NodeHandle("~").advertise<sensor_msgs::Image>("/cross_detect/binary", 1);

    bool is_red_cross = false;

    // 3. 遍历每个轮廓，分析凸点和凹点
    for (size_t i = 0; i < contours.size(); ++i) {
        // 过滤过小的轮廓（避免噪声）
        double area = contourArea(contours[i]);
        if (area < CONTOURS_AREA_THRESHOLD) // 面积阈值可根据实际调整
            continue;

        // 4. 计算凸包（凸点：凸包的顶点）
        std::vector<int> hull_indices; // 存储凸包顶点在轮廓中的索引
        convexHull(contours[i], hull_indices, false); // false表示返回索引
        int convex_points = hull_indices.size(); // 凸点数量

        // 5. 计算凸缺陷（凹点：凸缺陷的最深点）
        std::vector<cv::Vec4i> defects; // 存储凸缺陷：[start, end, far, depth]
        if (hull_indices.size() >= 3) {
            // 凸包至少3个点才有效
            convexityDefects(contours[i], hull_indices, defects);
        }

        // 过滤有效凹点（排除深度过小的噪声缺陷）
        int concave_points = 0;
        std::vector<cv::Point> concave_points_list; // 存储凹点坐标
        for (const auto &d: defects) {
            int far_idx = d[2]; // 最深点（凹点）在轮廓中的索引
            double depth = d[3] / 256.0; // 深度（注意单位转换）
            if (depth > DEPTH_THRESHOLD) {
                // 只保留深度足够的凹点
                concave_points++;
                concave_points_list.push_back(contours[i][far_idx]);
            }
        }

        if (convex_points == 8 && concave_points == 4) {
            is_red_cross = true;
        } else {
            is_red_cross = false;
        }

        if (debug) {
            // 可视化结果（可选）
            cv::Mat visual = cv::Mat::zeros(mask.size(), CV_8UC3);
            cv::cvtColor(mask, visual, cv::COLOR_GRAY2BGR); // 二值图转彩色以便标注
            cv::drawContours(visual, contours, i, cv::Scalar(0, 255, 0), 2); // 绘制轮廓（绿色）

            // 标记凸点（红色）
            for (int idx: hull_indices) {
                cv::circle(visual, contours[i][idx], 5, cv::Scalar(0, 0, 255), -1);
            }

            // 标记凹点（蓝色）
            for (const auto &p: concave_points_list) {
                cv::circle(visual, p, 5, cv::Scalar(255, 0, 0), -1);
            }
            sensor_msgs::ImageConstPtr image = cv_bridge::CvImage({}, sensor_msgs::image_encodings::BGR8, visual).
                    toImageMsg();
            binary_pub.publish(image);
        }
    }

    return is_red_cross;
}



bool node_control = false;

void node_control_callback(const std_msgs::Bool &control) {
    node_control = control.data;
}


cv::Mat current_image;

void image_callback(const sensor_msgs::ImageConstPtr &image) {
    try {
        const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        current_image = cv_ptr->image;
    } catch (cv_bridge::Exception &e) {
        ROS_WARN_THROTTLE(1, "cv_bridge error: %s", e.what());
    }
}


image_geometry::PinholeCameraModel camera_model;

void camera_info_callback(const sensor_msgs::CameraInfo &camera_info) {
    camera_model.fromCameraInfo(camera_info);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_cross_detect");
    ros::NodeHandle node("~");
    image_transport::ImageTransport it(node);

    // TODO 添加话题
    image_transport::Subscriber image_sub = it.subscribe("/camera/color/image_raw", 1, image_callback);
    ros::Subscriber camera_info_sub = node.subscribe("/camera/color/camera_info", 1, camera_info_callback);
    ros::Subscriber node_control_sub = node.subscribe("/detect/control", 1, node_control_callback);
    ros::Publisher detection_status_pub = node.advertise<std_msgs::Bool>("/detect/cross_status", 1);
    ros::Publisher pixel_offset_pub = node.advertise<geometry_msgs::PoseStamped>("/detect/cross_mark_point", 1);
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener transform_listener(buffer);
    std_msgs::Bool status_msg;

    node.param("red_cross_detection/s_min", s_min, 70);
    node.param("red_cross_detection/s_max", s_max, 255);
    node.param("red_cross_detection/v_min", v_min, 70);
    node.param("red_cross_detection/v_max", v_max, 255);
    node.param("red_cross_detection/depth_threshold", DEPTH_THRESHOLD, 10.0);
    node.param("red_cross_detection/contours_area_threshold", CONTOURS_AREA_THRESHOLD, 500.0);
    node.param("detection/debug", debug, true);

    geometry_msgs::TransformStamped camera2map_transform;

    while (ros::ok()) {
        ros::spinOnce();
        // if (!node_control) {
        //     continue;
        // }
        if (current_image.empty()) {
            status_msg.data = false;
            detection_status_pub.publish(status_msg);
            continue;
        }
        cv::Mat hsv_image;
        cv::cvtColor(current_image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv_image, cv::Scalar(0, s_min, v_min), cv::Scalar(10, s_max, v_max), mask1);
        cv::inRange(hsv_image, cv::Scalar(170, s_min, v_min), cv::Scalar(180, s_max, v_max), mask2);
        cv::bitwise_or(mask1, mask2, mask);

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!is_red_cross(mask, contours)) {
            status_msg.data = false;
            detection_status_pub.publish(status_msg);
            continue;
        }

        cv::Point2d center;
        cv::Moments moments = cv::moments(contours[0]);
        if (moments.m00 <= 0) {
            continue;
        }
        center.x = moments.m10 / moments.m00;
        center.y = moments.m01 / moments.m00;


        // 4. 获取相机和map之间的变换
        try {
            camera2map_transform = buffer.lookupTransform("camera_link", "map", ros::Time(0));
        } catch (tf2::TransformException &e) {
            ROS_ERROR("Ciallo～(∠・ω< )⌒★ %s", e.what());
            continue;
        }

        // 5. 根据中心像素和相机参数求取中心点坐标
        cv::Point3d center_vec = camera_model.projectPixelTo3dRay(center);
        geometry_msgs::Vector3 camera_vec_msg;
        camera_vec_msg.x = center_vec.x;
        camera_vec_msg.y = center_vec.y;
        camera_vec_msg.z = center_vec.z;
        tf2::Vector3 camera_vec;
        tf2::fromMsg(camera_vec_msg, camera_vec);

        tf2::Quaternion rot_quat;
        tf2::fromMsg(camera2map_transform.transform.rotation, rot_quat);

        tf2::Vector3 base_vec = tf2::quatRotate(rot_quat, camera_vec);

        double t = -camera2map_transform.transform.translation.z / base_vec.z();
        double x = base_vec.x() * t + camera2map_transform.transform.translation.x;
        double y = base_vec.y() * t - camera2map_transform.transform.translation.y;

        geometry_msgs::PoseStamped pose_stamped_res;
        pose_stamped_res.header.frame_id = "map";
        pose_stamped_res.header.stamp = ros::Time::now();
        pose_stamped_res.pose.position.x = x;
        pose_stamped_res.pose.position.y = y;
        pose_stamped_res.pose.position.z = 0;
        pose_stamped_res.pose.orientation.w = 1;
        pixel_offset_pub.publish(pose_stamped_res);
        ROS_INFO("camera2map_transform x, y, z, w: %f, %f, %f, %f", camera2map_transform.transform.rotation.x,
                 camera2map_transform.transform.rotation.y, camera2map_transform.transform.rotation.z,
                 camera2map_transform.transform.rotation.w);
        ROS_INFO("base_vec x, y, z: %f %f %f", base_vec.x(), base_vec.y(), base_vec.z());


        // 发布检测状态
        status_msg.data = true;
        detection_status_pub.publish(status_msg);
    }

    return 0;
}
