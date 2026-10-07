#include "IMU_Processing.hpp"
#include <iostream>

int main() {
    ImuProcess imu;
    esekfom::esekf<state_ikfom, 12, input_ikfom> filter;
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    cloud->push_back(PointType());  // Previous scan's output must be erased.
    MeasureGroup empty;
    empty.lidar_end_time = 2.;
    if (imu.Process(empty, filter, cloud) || !cloud->empty()) return 1;
    MeasureGroup initial;
    initial.lidar.reset(new PointCloudXYZI());
    initial.lidar_beg_time=1.; initial.lidar_end_time=1.1;
    sensor_msgs::Imu::Ptr sample(new sensor_msgs::Imu());
    sample->header.stamp=ros::Time(1.);
    sample->linear_acceleration.z=1.;
    initial.imu.push_back(sample);
    cloud->push_back(PointType());
    if (imu.Process(initial, filter, cloud) || !cloud->empty()) return 2;
    std::cout << "PASS: empty IMU and initialization cannot publish stale scan state\n";
}
