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
#include <algorithm>


int s_min, s_max;
int v_min, v_max;
double DEPTH_THRESHOLD;
double CONTOURS_AREA_THRESHOLD;
bool debug = false;
int morphology_kernel_size;


int is_red_cross(const cv::Mat &mask, const std::vector<std::vector<cv::Point> > &contours) {
    static ros::Publisher binary_pub = ros::NodeHandle("~").advertise<sensor_msgs::Image>("/cross_detect/binary", 1);

    auto max_contour_ptr = std::max_element(contours.begin(), contours.end(),
                                            [](const std::vector<cv::Point> &contour1,
                                               const std::vector<cv::Point> &contour2) {
                                                return cv::contourArea(contour1) > cv::contourArea(contour2);
                                            });

    int index = max_contour_ptr - contours.begin();

    // // （可选）简化轮廓，减少点数量（提高直线检测效率）
    // std::vector<cv::Point> approx;
    // cv::approxPolyDP(*max_contour_ptr, approx, 3, true); // 3为近似精度
    //
    // // 正确做法：将轮廓转为单通道8位图像
    // cv::Mat contourImg = cv::Mat::zeros(mask.size(), CV_8UC1); // 创建单通道8位空白图像（全黑）
    // // 绘制轮廓到空白图像，轮廓位置设为255（白色）
    // drawContours(contourImg, std::vector<std::vector<cv::Point>>{approx}, 0, cv::Scalar(255), 2);
    // // 5. 对轮廓点集应用概率霍夫变换检测直线段
    // std::vector<cv::Vec4i> lines; // 存储直线段：(x1,y1,x2,y2)
    // cv::HoughLinesP(contourImg, lines,
    //             1,           // 距离分辨率（像素）
    //             CV_PI/180,   // 角度分辨率（弧度）
    //             30,          // 累加器阈值（越高，检测到的直线越显著）
    //             10,          // 最小线段长度（像素）
    //             5);          // 线段间最大间隙（像素）
    // cv::Mat result = mask.clone();
    // // 6. 绘制检测到的直线段（红色）
    // for (const auto& line : lines) {
    //     cv::line(result, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2);
    // }
    //
    // // （可选）绘制轮廓（绿色），对比参考
    // cv::drawContours(result, contours, 0, cv::Scalar(0, 255, 0), 10);
    //
    // cv::imshow("space", result);
    // cv::waitKey(0);
    //
    // if (lines.size() != 12) {
    //     index = -1;
    // }
    //
    // return index;

    // 4. 计算凸包（凸点：凸包的顶点）
    std::vector<int> hull_indices; // 存储凸包顶点在轮廓中的索引
    convexHull(*max_contour_ptr, hull_indices, false); // false表示返回索引
    const int convex_points = hull_indices.size(); // 凸点数量

    // 5. 计算凸缺陷（凹点：凸缺陷的最深点）
    std::vector<cv::Vec4i> defects; // 存储凸缺陷：[start, end, far, depth]
    if (hull_indices.size() >= 3) {
        // 凸包至少3个点才有效
        convexityDefects(*max_contour_ptr, hull_indices, defects);
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
            concave_points_list.push_back((*max_contour_ptr)[far_idx]);
        }
    }

    if (convex_points > 20 || concave_points != 4 || convex_points < 8) {
        index = -1;
    }

    if (debug) {
        // 可视化结果（可选）
        cv::Mat visual = cv::Mat::zeros(mask.size(), CV_8UC3);
        cv::cvtColor(mask, visual, cv::COLOR_GRAY2BGR); // 二值图转彩色以便标注

        if (index != -1) {

            cv::drawContours(visual, contours, index, cv::Scalar(0, 255, 0), 2); // 绘制轮廓（绿色）

            // 标记凸点（红色）
            for (int idx: hull_indices) {
                cv::circle(visual, (*max_contour_ptr)[idx], 5, cv::Scalar(0, 0, 255), -1);
            }

            // 标记凹点（蓝色）
            for (const auto &p: concave_points_list) {
                cv::circle(visual, p, 5, cv::Scalar(255, 0, 0), -1);
            }
        }
        cv::putText(visual, std::to_string(convex_points) + " " + std::to_string(concave_points),
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        sensor_msgs::ImageConstPtr image = cv_bridge::CvImage({}, sensor_msgs::image_encodings::BGR8, visual).
                toImageMsg();
        binary_pub.publish(image);
    }

    // if (red_cross_contours_index != -1) {
    //     break;
    // }


    return index;
}


bool node_control = false;

void node_control_callback(const std_msgs::Bool &control) {
    node_control = control.data;
}


cv::Mat current_image;

void image_callback(const sensor_msgs::ImageConstPtr &image) {
    try {
        const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        cv::resize(cv_ptr->image, current_image, cv::Size(640, 512), 0, 0, cv::INTER_AREA);
    } catch (cv_bridge::Exception &e) {
        ROS_WARN_THROTTLE(1, "cv_bridge error: %s", e.what());
    }
}


image_geometry::PinholeCameraModel camera_model;

static sensor_msgs::CameraInfo generate_CameraInfo() {
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = "camera_link";
    camera_info.binning_x = 0;
    camera_info.binning_y = 0;
    camera_info.distortion_model = "plumb_bob";
    // 图像尺寸
    camera_info.height = 1024;
    camera_info.width = 1280;

    // 畸变模型（plumb_bob对应OpenCV的畸变模型）
    camera_info.distortion_model = "plumb_bob";

    // 内参矩阵K (3x3)：[fx, 0, cx; 0, fy, cy; 0, 0, 1]
    camera_info.K = {
        998.743048, 0, 662.188350,
        0, 997.846645, 523.650663,
        0, 0, 1
    };

    camera_info.D = {
        -0.369830, 0.155090, 0.001010, -0.006655, 0.000000
    };

    // 旋转矩阵R（单目相机默认单位矩阵）
    camera_info.R = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    // 投影矩阵P（3x4）：单目相机通常为[fx, 0, cx, 0; 0, fy, cy, 0; 0, 0, 1, 0]
    camera_info.P = {
        832.528288, 0, 600.254944, 0,
        0, 892.656545, 527.853892, 0,
        0, 0, 1, 0
    };
    return camera_info;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_cross_detect");
    ros::NodeHandle node("~");
    image_transport::ImageTransport it(node);

    // TODO 添加话题
    image_transport::Subscriber image_sub = it.subscribe("/camera/color/image_raw", 1, image_callback);
    ros::Subscriber node_control_sub = node.subscribe("/cross/control", 1, node_control_callback);
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
    node.param("red_cross_detection/morphology_kernel_size", morphology_kernel_size, 15);

    geometry_msgs::TransformStamped camera2map_transform;
    camera_model.fromCameraInfo(generate_CameraInfo());
    cv::namedWindow("space", cv::WINDOW_AUTOSIZE);
    while (ros::ok()) {
        ros::spinOnce();
        if (!node_control) {
            continue;
        }
        if (current_image.empty()) {
            ROS_WARN_THROTTLE(1, "no data");
            status_msg.data = false;
            detection_status_pub.publish(status_msg);
            continue;
        }

        if (!camera_model.initialized()) {
            ROS_WARN_THROTTLE(1, "no param");
        }

        cv::Mat hsv_image;
        cv::cvtColor(current_image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv_image, cv::Scalar(0, s_min, v_min), cv::Scalar(10, s_max, v_max), mask1);
        cv::inRange(hsv_image, cv::Scalar(170, s_min, v_min), cv::Scalar(180, s_max, v_max), mask2);
        cv::bitwise_or(mask1, mask2, mask);


        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(morphology_kernel_size, morphology_kernel_size));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);


        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int index = is_red_cross(mask, contours);

        if (index == -1) {
            status_msg.data = false;
            detection_status_pub.publish(status_msg);
            continue;
        }

        cv::Point2d center;
        cv::Moments moments = cv::moments(contours[index]);
        if (moments.m00 <= 0) {
            continue;
        }
        center.x = moments.m10 / moments.m00;
        center.y = moments.m01 / moments.m00;


        // 4. 获取相机和map之间的变换
        try {
            camera2map_transform = buffer.lookupTransform("map", "camera_link", ros::Time(0));
        } catch (tf2::TransformException &e) {
            ROS_ERROR("Ciallo～(∠・ω< )⌒★ %s", e.what());
            continue;
        }

        // 5. 根据中心像素和相机参数求取中心点坐标
        cv::Point3d center_vec = camera_model.projectPixelTo3dRay(center * 2);
        geometry_msgs::Vector3 camera_vec_msg;
        camera_vec_msg.x = center_vec.x;
        camera_vec_msg.y = center_vec.y;
        camera_vec_msg.z = center_vec.z;
        tf2::Vector3 camera_vec;
        tf2::fromMsg(camera_vec_msg, camera_vec);

        tf2::Quaternion rot_quat;
        tf2::fromMsg(camera2map_transform.transform.rotation, rot_quat);
        rot_quat = rot_quat.inverse();

        tf2::Vector3 base_vec = tf2::quatRotate(rot_quat, camera_vec);

        double t = -camera2map_transform.transform.translation.z / base_vec.z();
        double x = base_vec.x() * t + camera2map_transform.transform.translation.x;
        double y = base_vec.y() * t + camera2map_transform.transform.translation.y;

        geometry_msgs::PoseStamped pose_stamped_res;
        pose_stamped_res.header.frame_id = "map";
        pose_stamped_res.header.stamp = ros::Time::now();
        pose_stamped_res.pose.position.x = x;
        pose_stamped_res.pose.position.y = y;
        pose_stamped_res.pose.position.z = 0;
        pose_stamped_res.pose.orientation.w = 1;
        pixel_offset_pub.publish(pose_stamped_res);
        // ROS_INFO("camera2map_transform x, y, z, w: %f, %f, %f, %f", camera2map_transform.transform.rotation.x,
        //          camera2map_transform.transform.rotation.y, camera2map_transform.transform.rotation.z,
        //          camera2map_transform.transform.rotation.w);
        // ROS_INFO("base_vec x, y, z: %f %f %f", base_vec.x(), base_vec.y(), base_vec.z());


        // 发布检测状态
        status_msg.data = true;
        detection_status_pub.publish(status_msg);
    }

    return 0;
}
