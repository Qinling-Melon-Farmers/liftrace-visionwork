#pragma once
#include <Eigen/Geometry>
#include <cmath>

namespace patrol_control {
// 安装表是 FC -> 投口的机体系 FLU 杆臂，现有二维测量未包含垂直杆臂。
inline bool outletArmInMission(const Eigen::Quaterniond& orientation,
                              double forward, double left,
                              Eigen::Vector3d* arm) {
    if (!orientation.coeffs().allFinite() || !std::isfinite(forward) ||
        !std::isfinite(left) || std::abs(orientation.norm() - 1.0) > 0.01)
        return false;
    *arm = orientation.normalized() * Eigen::Vector3d(forward, left, 0.0);
    return arm->allFinite();
}
// 每次从原始靶心计算，不能把上次补偿后的 FC 坐标再当作靶心。
inline Eigen::Vector2d compensatedFcXY(const Eigen::Vector2d& center,
                                     const Eigen::Vector3d& arm) {
    return center - arm.head<2>();
}
} // namespace patrol_control
