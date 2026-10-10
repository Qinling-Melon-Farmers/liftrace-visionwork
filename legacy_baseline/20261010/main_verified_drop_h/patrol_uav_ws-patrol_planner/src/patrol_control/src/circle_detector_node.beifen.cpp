/**
******************************************************************************
  * @file           : circle_detection.cpp
  * @author         : anontokyo
  * @brief          : None
  * @attention      : None
  * @date           : 2025.9.28
  ******************************************************************************
  */


#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

class CircleDetection {
public:
    CircleDetection(const ros::NodeHandle &nh) : it(nh), transform_listener(buffer) {
        this->node = nh;
        this->image_sub = it.subscribe("/camera/color/image_raw", 1, &CircleDetection::image_callback, this);
        this->control_sub = node.subscribe("/detect/control", 1, &CircleDetection::control_callback, this);
        this->circle_center_pub = node.advertise<geometry_msgs::PoseStamped>("/detect/waypoint_mark_point", 1);
        this->camera_info_sub = node.subscribe("/camera/color/camera_info", 1, &CircleDetection::camera_info_callback, this);
        this->debug_img_pub = it.advertise("/detect/binary_result_image/compressed", 1);

        // load_params
        // 半径范围参数 (用于过滤)
        node.param("circle_detection/min_radius", radius_min, 10.0);
        node.param("circle_detection/max_radius", radius_max, 300.0);

        // 色彩分割参数 (*** 已根据实际图像进行调整 ***)
        node.param("color_segmentation/h_min", h_min, 90);
        node.param("color_segmentation/s_min", s_min, 80);
        node.param("color_segmentation/v_min", v_min, 80);
        node.param("color_segmentation/h_max", h_max, 130);
        node.param("color_segmentation/s_max", s_max, 255);
        node.param("color_segmentation/v_max", v_max, 255);

        // 质量评估参数 (基于轮廓和椭圆拟合)
        node.param("quality_assessment/min_contour_points", min_contour_points, 15);
        node.param("quality_assessment/aspect_ratio_threshold", aspect_ratio_threshold, 0.85); // 更严格，因为我们期望是圆

        // 图像预处理参数
        node.param("image_preprocessing/blur_kernel_size", blur_kernel_size, 5);
        node.param("image_preprocessing/morphology_kernel_size", morphology_kernel_size, 31);

        // 检测控制参数
        node.param("detection/debug", debug, true);
    }

    ~CircleDetection() = default;

    void loop() {
        ros::spinOnce();
        if (!this->control.data) {
            return;
        }

        // 1. 判断current_image是否有效
        if (current_image.empty()) {
            ROS_WARN_THROTTLE(1, "no data");
            return;
        }

        // 2. 判断camera_model是否初始化
        if (!camera_model.initialized()) {
            ROS_WARN_THROTTLE(1, "no params");
            return;
        }

        // 预处理
        cv::Mat processed_image = current_image;
        cv::GaussianBlur(processed_image, processed_image, cv::Size(blur_kernel_size, blur_kernel_size), 0);

        // --- 蓝色圆形检测逻辑 ---

        // 1. 色彩分割
        cv::Mat hsv_image, mask;
        cv::cvtColor(processed_image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);

        // 形态学操作，去除噪点，连接断裂区域
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                   cv::Size(morphology_kernel_size, morphology_kernel_size));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // 2. 寻找轮廓
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        cv::RotatedRect best_ellipse;
        double best_area = 0;

        // 3. 遍历轮廓并进行椭圆拟合
        for (const auto &contour: contours) {
            // 轮廓点数太少，无法稳定拟合
            if (contour.size() < min_contour_points) {
                continue;
            }

            const cv::RotatedRect ellipse = cv::fitEllipse(contour);
            const double area = cv::contourArea(contour);

            // 4. 质量评估
            double width = ellipse.size.width;
            double height = ellipse.size.height;

            // 避免除以零
            if (width < 1e-3 || height < 1e-3) continue;

            // a. 长宽比判断，越接近1越像圆
            double aspect_ratio = std::min(width, height) / std::max(width, height);
            if (aspect_ratio < aspect_ratio_threshold) {
                continue;
            }

            // b. 半径范围判断
            double radius = (width + height) / 4.0; // 平均半径
            if (radius < radius_min || radius > radius_max) {
                continue;
            }

            // 选择面积最大的合格轮廓
            if (area > best_area) {
                best_area = area;
                best_ellipse = ellipse;
            }
        }

        if (debug) {
            cv::Mat binary = drawBinaryResultWithQualityParams(mask, contours, best_area == 0 ? nullptr : &best_ellipse);
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            header.frame_id = "camera_link";  // 与图像帧一致
            sensor_msgs::Image msg = *cv_bridge::CvImage(header, "bgr8", binary).toImageMsg();
            debug_img_pub.publish(msg);
        }

        if (best_area == 0) {
            return;
        }

        // 4. 获取相机和map之间的变换
        try {
            camera2map_transform = buffer.lookupTransform("map", "camera_link", ros::Time(0));
        } catch (tf2::TransformException &e) {
            ROS_ERROR("tf2::TransformException %s", e.what());
            return;
        }

        // 5. 根据中心像素和相机参数求取中心点坐标
        cv::Point3d center_vec = camera_model.projectPixelTo3dRay(best_ellipse.center);
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
        circle_center_pub.publish(pose_stamped_res);
        ROS_INFO("camera2map_transform x, y, z, w: %f, %f, %f, %f", camera2map_transform.transform.rotation.x, camera2map_transform.transform.rotation.y, camera2map_transform.transform.rotation.z, camera2map_transform.transform.rotation.w);
        ROS_INFO("base_vec x, y, z: %f %f %f", base_vec.x(), base_vec.y(), base_vec.z());

    }

    cv::Mat drawBinaryResultWithQualityParams(const cv::Mat &mask,
                                              const std::vector<std::vector<cv::Point> > &contours,
                                              const cv::RotatedRect *best_ellipse) {
        // 将二值化图像转换为彩色图像
        cv::Mat binary_color;
        cv::cvtColor(mask, binary_color, cv::COLOR_GRAY2BGR);

        // 在二值化图像上绘制所有轮廓（红色）
        cv::drawContours(binary_color, contours, -1, cv::Scalar(0, 0, 255), 2);

        if (best_ellipse) {
            // 绘制最佳拟合椭圆（绿色）
            cv::ellipse(binary_color, *best_ellipse, cv::Scalar(0, 255, 0), 5);

            // 绘制圆心（蓝色）
            cv::circle(binary_color, best_ellipse->center, 8, cv::Scalar(255, 0, 0), -1);

            // 绘制半径线（黄色）
            double radius = (best_ellipse->size.width + best_ellipse->size.height) / 4.0;
            cv::line(binary_color, best_ellipse->center,
                     cv::Point(best_ellipse->center.x + radius, best_ellipse->center.y),
                     cv::Scalar(0, 255, 255), 2);

                    // 绘制图像中心点（白色十字）
            cv::circle(binary_color, best_ellipse->center, 5, cv::Scalar(255, 255, 255), -1);
            int center_size = 15;
            cv::line(binary_color,
                    cv::Point(best_ellipse->center.x - center_size, best_ellipse->center.y),
                    cv::Point(best_ellipse->center.x + center_size, best_ellipse->center.y),
                    cv::Scalar(255, 255, 255), 2);
            cv::line(binary_color,
                    cv::Point(best_ellipse->center.x, best_ellipse->center.y - center_size),
                    cv::Point(best_ellipse->center.x, best_ellipse->center.y + center_size),
                    cv::Scalar(255, 255, 255), 2);
        } else {
            // 没有检测到有效圆形时显示状态
            cv::putText(binary_color, "No Valid Circle Detected",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            cv::putText(binary_color, "Check HSV parameters and quality thresholds",
                        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        }


        return binary_color;
    }

    void image_callback(const sensor_msgs::ImageConstPtr &image_message) {
        try {
            const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::BGR8);
            this->current_image = cv_ptr->image;
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge failed: %s", e.what());
        }
    }

    void control_callback(const std_msgs::Bool &bool_message) {
        this->control = bool_message;
    }

    void camera_info_callback(const sensor_msgs::CameraInfo &camera_message) {
        this->camera_model.fromCameraInfo(camera_message);
    }

private:
    ros::NodeHandle node;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher debug_img_pub;
    ros::Subscriber control_sub;
    ros::Subscriber camera_info_sub;
    ros::Publisher circle_center_pub;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener transform_listener;

    std_msgs::Bool control;
    cv::Mat current_image;
    image_geometry::PinholeCameraModel camera_model;
    geometry_msgs::TransformStamped camera2map_transform;
    bool debug;

    int blur_kernel_size;
    int h_min, h_max, s_min, s_max, v_min, v_max;
    int morphology_kernel_size;
    int min_contour_points;
    double aspect_ratio_threshold;
    double radius_min, radius_max;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_detector_node");
    const ros::NodeHandle node("~");
    CircleDetection det(node);

    while (ros::ok()) {
        det.loop();
    }

    return 0;
}
