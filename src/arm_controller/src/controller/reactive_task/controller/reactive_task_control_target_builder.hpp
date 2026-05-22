#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "algorithm/cartesian_path_planner/types.hpp"
#include "controller/reactive_task/local_planner/reactive_task_local_planner_runtime.hpp"
#include "controller/reactive_task/controller/reactive_task_terminal_policy.hpp"
#include "controller/reactive_task/controller/reactive_task_types.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;

class ReactiveTaskControlTargetBuilder {
public:
  struct Input {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string mapping;
    rclcpp::Logger logger{rclcpp::get_logger("reactive_task_control_target")};
    rclcpp::Clock::SharedPtr clock;
    ReactiveTaskTerminalPolicy::PhaseFlags phase_flags;
    bool terminal_goal_tracking{false};
    cp::TimedCartesianSample sample;
    Eigen::Vector3d goal_position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d goal_rotation{Eigen::Matrix3d::Identity()};
    cp::TimedCartesianSample terminal_joint_anchor_sample;
    bool terminal_joint_anchor_sample_valid{false};
    ArmState arm_state;
    double tracked_reference_time_sec{0.0};
    const LocalPlannerRuntime* local_planner{nullptr};
  };

  ControlTarget build(const Input& input) const;

private:
  bool applyLocalPlannerTrajectory(const Input& input,
                                   ControlTarget* target) const;
  bool applyLocalPlannerFallback(const Input& input,
                                 ControlTarget* target) const;
};

}  // namespace arm_controller::controller::reactive_task
