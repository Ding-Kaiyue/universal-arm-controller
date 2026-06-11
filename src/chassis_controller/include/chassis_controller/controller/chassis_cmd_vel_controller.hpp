#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "chassis_controller/adapter/chassis_velocity_command_adapter.hpp"
#include "chassis_controller/config/chassis_controller_config.hpp"
#include "chassis_controller/kinematics/chassis_kinematics.hpp"
#include "chassis_controller/model/chassis_joint_command.hpp"

namespace chassis_controller {

class ChassisController {
public:
    ChassisController(rclcpp::Node& node, ChassisControllerConfig config);

    void start();

private:
    void handleCommand(const geometry_msgs::msg::Twist& msg);
    void publishTwistCommand(const ChassisVelocityCommand& command);
    void publishJointCommands(const ChassisJointCommandSet& commands);

    rclcpp::Node& node_;
    ChassisControllerConfig config_;
    ChassisVelocityCommandAdapter adapter_;
    ChassisKinematics kinematics_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

}  // namespace chassis_controller
