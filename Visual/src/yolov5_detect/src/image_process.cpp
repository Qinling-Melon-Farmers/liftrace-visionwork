#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "yolov5_detect/image2center.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Bool.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "image_geometry/pinhole_camera_model.h"
#include "opencv2/core/core.hpp"


image_geometry::PinholeCameraModel camera_model;
static sensor_msgs::CameraInfo generate_CameraInfo() {
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = "camera_link";
    camera_info.binning_x = 0;
    camera_info.binning_y = 0;

    // 图像尺寸
    camera_info.height = 1024;
    camera_info.width = 1280;

    // 畸变模型（plumb_bob对应OpenCV的畸变模型）
    camera_info.distortion_model = "plumb_bob";

    // 内参矩阵K (3x3)：[fx, 0, cx; 0, fy, cy; 0, 0, 1]
    camera_info.K = {
        997.634832, 0, 625.164605,
        0, 998.885312, 509.350168,
        0, 0, 1
    };

    camera_info.D = {
        -0.383598, 0.138970, 0.001680, -0.000277, 0.000000
    };

    // 旋转矩阵R（单目相机默认单位矩阵）
    camera_info.R = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    // 投影矩阵P（3x4）：单目相机通常为[fx, 0, cx, 0; 0, fy, cy, 0; 0, 0, 1, 0]
    camera_info.P = {
        814.495938, 0.0, 617.472576, 0.0,
        0.0, 886.710092, 510.889602, 0.0,
        0, 0, 1, 0
    };
    return camera_info;
}

bool pose_callback(yolov5_detect::image2center::Request& request, yolov5_detect::image2center::Response& response) {
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener transform_listener(buffer);
    static ros::Publisher pixel_offset_pub = ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("/detect/tank_status", 1);
    const cv::Point center = {request.x.data, request.y.data};


    geometry_msgs::TransformStamped camera2map_transform;
    // 4. 获取相机和map之间的变换
    try {
        camera2map_transform = buffer.lookupTransform("map", "camera_link", ros::Time(0));
    } catch (tf2::TransformException &e) {
        ROS_ERROR("Ciallo~(∠・ω< )⌒★ %s", e.what());
        return false;
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
    return true;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "image_process");
    camera_model.fromCameraInfo(generate_CameraInfo());
    ros::ServiceServer pose_server = ros::NodeHandle().advertiseService("/visual/service", pose_callback);

    ros::spin();
    return 0;
}
