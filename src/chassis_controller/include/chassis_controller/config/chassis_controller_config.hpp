#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "chassis_controller/kinematics/chassis_kinematics.hpp"
#include "chassis_controller/model/chassis_model.hpp"

namespace chassis_controller {

struct ChassisControllerConfig {
    ChassisModel model;
    std::string input_topic{"cmd_vel"};
    std::string output_topic{"cmd_vel"};
    std::string joint_command_topic{"chassis_joint_commands"};
    bool publish_twist{true};
    bool publish_joint_commands{false};
    bool warn_on_unsupported_lateral{true};
    ChassisKinematicsConfig kinematics;
};

ChassisControllerConfig loadChassisControllerConfig(rclcpp::Node& node);

}  // namespace chassis_controller
