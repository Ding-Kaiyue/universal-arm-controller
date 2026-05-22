#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "controller/reactive_task/local_planner/reactive_task_local_planner.hpp"

namespace arm_controller::controller::reactive_task {

using LocalPlannerPoseList =
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;
using LocalPlannerTwistList =
    std::vector<Eigen::Matrix<double, 6, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>>;

struct LocalPlannerRuntime {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Isometry3d last_target_pose{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 1> last_target_twist{
      Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::VectorXd last_joint_target;
  double last_joint_target_dt_sec{0.0};
  double last_update_time_sec{-std::numeric_limits<double>::infinity()};
  bool last_output_valid{false};

  LocalPlannerPoseList target_poses;
  LocalPlannerTwistList target_twists;
  LocalPlannerJointVectorList joint_targets;
  double start_time_sec{0.0};
  std::size_t start_index{0};
  double dt_sec{0.05};
  bool trajectory_valid{false};

  struct ApplyOutputInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string mapping;
    rclcpp::Logger logger{rclcpp::get_logger("local_planner_runtime")};
    rclcpp::Clock::SharedPtr clock;
    ArmLocalPlanner::Output output;
    Eigen::VectorXd planner_start_state;
    Eigen::VectorXd q_goal;
    Eigen::VectorXd current_qdot_reference;
    bool current_qdot_reference_valid{false};
    int planner_tick{0};
    double now_sec{0.0};
    double request_time_sec{0.0};
    double max_usable_age_sec{0.12};
  };

  struct ApplyOutputResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool accepted{false};
    bool stale_age{false};
    bool stale_handoff{false};
    Eigen::Matrix<double, 6, 1> nominal_twist{
        Eigen::Matrix<double, 6, 1>::Zero()};
    std::string reason;
  };

  void clearTracking();
  void clearCachedTarget();

  ApplyOutputResult applyOutput(ApplyOutputInput input);

  int sampledSteps() const {
    return static_cast<int>(target_poses.size());
  }

  double diagnosticDtSec() const {
    return trajectory_valid ? dt_sec : 0.0;
  }
};

}  // namespace arm_controller::controller::reactive_task
