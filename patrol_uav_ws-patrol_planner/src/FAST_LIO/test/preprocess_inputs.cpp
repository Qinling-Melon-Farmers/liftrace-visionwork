#include <cmath>
#include <iostream>
#include <stdexcept>
#include "preprocess.h"

void expectEqual(double expected, double actual) {
  if (!std::isfinite(actual) || std::abs(expected - actual) > 1e-6)
    throw std::runtime_error("Expected " + std::to_string(expected) +
                             ", got " + std::to_string(actual));
}

void driver2CustomMessagePreservesPointTiming() {
  Preprocess pre;
  pre.set(false, AVIA, 0.5, 1);
  pre.N_SCANS = 4;
  livox_ros_driver2::CustomMsg::Ptr msg(new livox_ros_driver2::CustomMsg);
  msg->points.resize(5);
  msg->point_num = msg->points.size();
  // Index zero is intentionally skipped by the existing Livox preprocessor.
  for (size_t i = 0; i < msg->points.size(); ++i) {
    auto &p = msg->points[i];
    p.x = 1.0 + i;
    p.reflectivity = 42;
    p.line = 0;
    p.tag = 0x10;
    p.offset_time = i * 1000000;
  }
  msg->points[2].line = 4;  // Invalid scan line.
  msg->points[3].tag = 0x20;  // Rejected return tag.
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  pre.process(msg, cloud);
  expectEqual(2u, cloud->size());
  expectEqual(2.0f, cloud->points[0].x);
  expectEqual(42.0f, cloud->points[0].intensity);
  expectEqual(1.0f, cloud->points[0].curvature);  // ns -> ms.
  expectEqual(5.0f, cloud->points[1].x);
  expectEqual(4.0f, cloud->points[1].curvature);
}

void gazeboXYZPointCloudStillWorksWithoutIntensity() {
  Preprocess pre;
  pre.set(false, MARSIM, 0.5, 1);
  pre.time_unit = NS;
  pcl::PointCloud<pcl::PointXYZ> input;
  input.push_back(pcl::PointXYZ(0.1, 0, 0));
  input.push_back(pcl::PointXYZ(1, 2, 3));
  sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(input, *msg);
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
  pre.process(msg, cloud);
  expectEqual(1u, cloud->size());
  expectEqual(1.0f, cloud->points[0].x);
  expectEqual(2.0f, cloud->points[0].y);
  expectEqual(3.0f, cloud->points[0].z);
  expectEqual(0.0f, cloud->points[0].intensity);
  expectEqual(0.0f, cloud->points[0].curvature);
}

int main() {
  try {
    driver2CustomMessagePreservesPointTiming();
    std::cout << "PASS: driver2 CustomMsg timing and filtering\n";
    gazeboXYZPointCloudStillWorksWithoutIntensity();
    std::cout << "PASS: Gazebo XYZ PointCloud2 and blind filter\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
