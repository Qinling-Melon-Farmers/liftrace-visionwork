#include <cmath>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <new>
#include <type_traits>
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
  input.clear();
  pcl::toROSMsg(input, *msg);
  pre.process(msg, cloud);
  expectEqual(0, cloud->size());
}

// Exercise the production handlers, including their scan-line metadata setup.
PointCloudXYZI::Ptr featureScan(Preprocess &pre, int lidar,
                               const std::vector<pcl::PointXYZ> &points) {
  pre.set(true, lidar, 0.5, 1);
  pre.N_SCANS = 2;  // Leave a second scan line empty as well.
  pre.time_unit = MS;
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  if (lidar == AVIA) {
    livox_ros_driver2::CustomMsg::Ptr msg(new livox_ros_driver2::CustomMsg);
    msg->points.resize(points.size() + 1);  // Livox skips index zero.
    msg->point_num = msg->points.size();
    for (size_t i = 0; i < points.size(); ++i) {
      auto &p = msg->points[i + 1];
      p.x = points[i].x; p.y = points[i].y; p.z = points[i].z;
      p.reflectivity = 42; p.line = 0; p.tag = 0x10;
      p.offset_time = (i + 1) * 1000000;
    }
    pre.process(msg, result);
  } else {
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
    if (lidar == OUST64) {
      pcl::PointCloud<ouster_ros::Point> input;
      for (size_t i = 0; i < points.size(); ++i) {
        ouster_ros::Point p{};
        p.x = points[i].x; p.y = points[i].y; p.z = points[i].z;
        p.intensity = 42; p.ring = 0; p.t = i + 1;
        input.push_back(p);
      }
      pcl::toROSMsg(input, *msg);
    } else {
      pcl::PointCloud<velodyne_ros::Point> input;
      for (size_t i = 0; i < points.size(); ++i) {
        velodyne_ros::Point p{};
        p.x = points[i].x; p.y = points[i].y; p.z = points[i].z;
        p.intensity = 42; p.ring = 0; p.time = i + 1;
        input.push_back(p);
      }
      pcl::toROSMsg(input, *msg);
    }
    pre.process(msg, result);
  }
  return result;
}

std::vector<pcl::PointXYZ> scanLine(size_t count, float x, float step) {
  std::vector<pcl::PointXYZ> points;
  for (size_t i = 0; i < count; ++i)
    points.emplace_back(x, i * step, 0);
  return points;
}

void emptyAndShortFeatureScans() {
  for (int lidar : {AVIA, VELO16, OUST64}) {
    Preprocess pre;
    expectEqual(9, featureScan(pre, lidar, scanLine(9, 1, 0.02))->size());
    for (size_t n = 0; n <= 8; ++n) {
      const auto result = featureScan(pre, lidar, scanLine(n, 1, 0.02));
      // The existing three-point fallback legitimately keeps the middle of
      // seven/eight-point collinear scans even without a full plane group.
      expectEqual(n > 6 ? 3 : 0, result->size());
      expectEqual(0, pre.pl_corn.size());
    }
  }
}

void blindFeatureScans() {
  for (int lidar : {AVIA, VELO16, OUST64}) {
    Preprocess pre;
    for (size_t n : {2u, 6u, 8u, 9u, 32u}) {
      const auto result = featureScan(pre, lidar, scanLine(n, 0.1, 0.001));
      expectEqual(0, result->size());
      expectEqual(0, pre.pl_corn.size());
    }
    // A nonblind suffix too short for a plane must also be safe.
    auto points = scanLine(16, 0.1, 0.001);
    points.emplace_back(1, 0, 0);
    expectEqual(0, featureScan(pre, lidar, points)->size());
  }
}

void denseTailDoesNotBecomeAnOutOfRangePlane() {
  for (int lidar : {AVIA, VELO16, OUST64}) {
    Preprocess pre;
    // Nine collinear points, but even the last is inside the 0.11 m group
    // threshold. Only the five middle points qualify via the existing
    // small-plane fallback; a full plane must not include a tenth point.
    auto result = featureScan(pre, lidar, scanLine(9, 1, 0.001));
    expectEqual(5, result->size());
    for (size_t i = 0; i < result->size(); ++i)
      expectEqual((i + 2) * 0.001, result->points[i].y);
    expectEqual(0, pre.pl_corn.size());
  }
}

void planeThresholdIsDeterministic() {
  for (int lidar : {AVIA, VELO16, OUST64}) {
    // Make forgotten scalar initialization reproducible without accessing
    // private members or replacing the production implementation.
    for (unsigned char pattern : {0x00, 0x55, 0xff}) {
      std::aligned_storage<sizeof(Preprocess), alignof(Preprocess)>::type storage;
      std::memset(&storage, pattern, sizeof(storage));
      Preprocess *pre = new (&storage) Preprocess;
      try {
        // At range 1 m: 0.01 * range + 0.1 = 0.11 m.
        expectEqual(5, featureScan(*pre, lidar, scanLine(9, 1, 0.013125))->size());
        auto result = featureScan(*pre, lidar, scanLine(9, 1, 0.015));
        expectEqual(9, result->size());
        for (size_t i = 0; i < result->size(); ++i) {
          expectEqual(1, result->points[i].x);
          expectEqual(i * 0.015, result->points[i].y);
          expectEqual(42, result->points[i].intensity);
          expectEqual(i + 1, result->points[i].curvature);
        }
        // A second range distinguishes the slope from the intercept.
        expectEqual(5, featureScan(*pre, lidar, scanLine(9, 10, 0.02375))->size());
        expectEqual(9, featureScan(*pre, lidar, scanLine(9, 10, 0.02625))->size());
      } catch (...) {
        pre->~Preprocess();
        throw;
      }
      pre->~Preprocess();
    }
  }
}

int main(int argc, char **argv) {
  const std::string selected = argc > 1 ? argv[1] : "all";
  const std::pair<const char *, void (*)()> tests[] = {
      {"driver2", driver2CustomMessagePreservesPointTiming},
      {"gazebo", gazeboXYZPointCloudStillWorksWithoutIntensity},
      {"short", emptyAndShortFeatureScans},
      {"blind", blindFeatureScans},
      {"tail", denseTailDoesNotBecomeAnOutOfRangePlane},
      {"threshold", planeThresholdIsDeterministic},
  };
  try {
    bool ran = false;
    for (const auto &test : tests) {
      if (selected != "all" && selected != test.first) continue;
      test.second();
      std::cout << "PASS: " << test.first << std::endl;
      ran = true;
    }
    if (!ran) throw std::runtime_error("Unknown test: " + selected);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
