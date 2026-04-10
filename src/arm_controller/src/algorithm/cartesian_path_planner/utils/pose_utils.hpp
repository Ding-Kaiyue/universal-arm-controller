#pragma once

#include <Eigen/Geometry>

namespace arm_controller::algorithm::cartesian_path_planner::utils {

inline Eigen::Isometry3d makePose(const Eigen::Vector3d& p, const Eigen::Matrix3d& R) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = p;
    return T;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner::utils
