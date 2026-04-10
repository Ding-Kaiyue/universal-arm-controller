#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

namespace arm_controller::algorithm::cartesian_path_planner {

struct SphereObstacle {
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    double radius{0.5};
};

struct BoxObstacle {
    Eigen::Vector3d min_corner{Eigen::Vector3d(-0.5, -0.5, -0.5)};
    Eigen::Vector3d max_corner{Eigen::Vector3d(0.5, 0.5, 0.5)};
};

inline double distanceToSphere(
    const Eigen::Vector3d& p,
    const SphereObstacle& s) {
    return (p - s.center).norm() - s.radius;
}

inline double distanceToAABB(
    const Eigen::Vector3d& p,
    const BoxObstacle& b) {
    // 计算点p到轴对齐包围盒b的距离
    Eigen::Vector3d q = p.cwiseMax(b.min_corner).cwiseMin(b.max_corner);
    const bool inside = 
        (p.x() >= b.min_corner.x() && p.x() <= b.max_corner.x()) &&
        (p.y() >= b.min_corner.y() && p.y() <= b.max_corner.y()) &&
        (p.z() >= b.min_corner.z() && p.z() <= b.max_corner.z());
    if (!inside) {
        return (p - q).norm();  // 点在外部，返回距离
    } else {
        // 点在内部，计算到最近边界的距离
        double dx = std::min(p.x() - b.min_corner.x(), b.max_corner.x() - p.x());
        double dy = std::min(p.y() - b.min_corner.y(), b.max_corner.y() - p.y());
        double dz = std::min(p.z() - b.min_corner.z(), b.max_corner.z() - p.z());
        return -std::min({dx, dy, dz});  // 返回到最近边界的距离
    }
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
