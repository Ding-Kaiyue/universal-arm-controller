#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace arm_controller::controller::reactive_task::planning_detail {

inline double orientationErrorRad(const Eigen::Matrix3d &r_current,
                                  const Eigen::Matrix3d &r_target) {
  const Eigen::Matrix3d r_err = r_current.transpose() * r_target;
  Eigen::AngleAxisd aa(r_err);
  return std::abs(aa.angle());
}

} // namespace arm_controller::controller::reactive_task::planning_detail
