#pragma once

#include "arm_controller/controller_base/trajectory_controller_base.hpp"

std::unique_ptr<TrajectoryControllerBase> createReactiveTaskController(
    const rclcpp::Node::SharedPtr& node);
