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
#include <std_msgs/builtin_double.h>
#include <std_msgs/Int8.h>

class LandDetection {
public:
    LandDetection(const ros::NodeHandle &nh) : it(nh), transform_listener(buffer) {
        this->node = nh;
        this->image_sub = it.subscribe("/camera/color/image_raw", 1, &LandDetection::image_callback, this);
        this->control_sub = node.subscribe("/detect/landing_control", 1, &LandDetection::control_callback, this);
        this->circle_center_pub = node.advertise<geometry_msgs::PoseStamped>("/detect/land_mark_point", 1);
        this->debug_img_pub = it.advertise("/detect/binary_result_image_landing", 1);
        this->target_height_sub = node.subscribe("/detect/target_height", 1, &LandDetection::target_height_callback, this);

        this->camera_model.fromCameraInfo(generate_CameraInfo());

        // load_params
        // 半径范围参数 (用于过滤)
        node.param("circle_detection/min_radius", radius_min, 10.0);
        node.param("circle_detection/max_radius", radius_max, 300.0);

        // 色彩分割参数 (*** 已根据实际图像进行调整 ***)


        // 质量评估参数 (基于轮廓和椭圆拟合)
        node.param("quality_assessment/min_contour_points", min_contour_points, 15);
        node.param("quality_assessment/aspect_ratio_threshold", aspect_ratio_threshold, 0.85); // 更严格，因为我们期望是圆

        // 图像预处理参数
        node.param("image_preprocessing/blur_kernel_size", blur_kernel_size, 5);
        node.param("image_preprocessing/morphology_kernel_size", morphology_kernel_size, 7);

        morphology_kernel_size = morphology_kernel_size & 1 ? morphology_kernel_size : morphology_kernel_size + 1;
        blur_kernel_size = blur_kernel_size & 1 ? blur_kernel_size : blur_kernel_size + 1;
        // 检测控制参数
        node.param("detection/debug", debug, true);


        this->target_height = 0.0;
    }

    ~LandDetection() = default;

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
        // cv::GaussianBlur(current_image, processed_image, cv::Size(blur_kernel_size, blur_kernel_size), 0);

        // --- 蓝色圆形检测逻辑 ---

        // 1. 色彩分割
        cv::Mat gray_image;
        cv::cvtColor(processed_image, gray_image, cv::COLOR_BGR2GRAY);

        // 1.5 灰度范围归一化到0-255
        gray_image.convertTo(gray_image, CV_8U);
        // cv::Mat log_image;
        // cv::log(gray_image + 1, log_image); // 避免log(0)
        // double minVal, maxVal;
        // cv::minMaxLoc(log_image, &minVal, &maxVal);
        // log_image = 255 * (log_image - minVal) / (maxVal - minVal);
        // log_image.convertTo(gray_image, CV_8U);

        // 1.6 高斯滤波去噪
        cv::GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 0);
    
        // 2. 使用自适应阈值处理阴影
        cv::Mat adaptive_bin;
        cv::adaptiveThreshold(gray_image, adaptive_bin, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 31, 10);

        // cv::cvtColor(processed_image, hsv_image, cv::COLOR_BGR2HSV);
        // cv::inRange(grey_image, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask); 
        

        cv::Mat mask = adaptive_bin;
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
        cv::Point3d center_vec = camera_model.projectPixelTo3dRay(best_ellipse.center * 2);
        // cv::Point3d image_center_vec = camera_model.projectPixelTo3dRay(cv::Point2f(640, 512));
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

        double t = (-camera2map_transform.transform.translation.z - dynamic_calibration) / base_vec.z();
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
        // ROS_INFO("camera2map_transform x, y, z, w: %f, %f, %f, %f", camera2map_transform.transform.rotation.x, camera2map_transform.transform.rotation.y, camera2map_transform.transform.rotation.z, camera2map_transform.transform.rotation.w);
        // ROS_INFO("base_vec x, y, z: %f %f %f", base_vec.x(), base_vec.y(), base_vec.z());

    }

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

    static cv::Mat drawBinaryResultWithQualityParams(const cv::Mat &mask,
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
            cv::resize(cv_ptr->image, current_image, cv::Size(640, 512), 0, 0, cv::INTER_AREA);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge failed: %s", e.what());
        }
    }

    void control_callback(const std_msgs::Bool &bool_message) {
        this->control = bool_message;
    }


    void target_height_callback(const std_msgs::Float64& height) {
        this->target_height = height.data;
    }
private:
    ros::NodeHandle node;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher debug_img_pub;
    ros::Subscriber control_sub;
    ros::Subscriber camera_info_sub;
    ros::Subscriber target_height_sub;
    ros::Subscriber point_class_sub_;
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
    int pointclass;
    double aspect_ratio_threshold;
    double radius_min, radius_max;
    double target_height;
    double dynamic_calibration = 0.0;

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_detector_node");
    const ros::NodeHandle node("~");
    LandDetection det(node);



    while (ros::ok()) {
        det.loop();
    }

    return 0;
}
