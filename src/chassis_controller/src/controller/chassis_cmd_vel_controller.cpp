#include "chassis_controller/controller/chassis_cmd_vel_controller.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace chassis_controller {

ChassisController::ChassisController(rclcpp::Node& node, ChassisControllerConfig config)
    : node_(node),
      config_(std::move(config)),
      adapter_(config_.model),
      kinematics_(config_.kinematics) {}

void ChassisController::start() {
    if (config_.publish_twist) {
        twist_publisher_ =
            node_.create_publisher<geometry_msgs::msg::Twist>(config_.output_topic, 10);
    }
    if (config_.publish_joint_commands) {
        joint_command_publisher_ =
            node_.create_publisher<sensor_msgs::msg::JointState>(config_.joint_command_topic, 10);
    }

    subscription_ = node_.create_subscription<geometry_msgs::msg::Twist>(
        config_.input_topic, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { handleCommand(*msg); });

    RCLCPP_INFO(node_.get_logger(), "chassis controller ready: type=%s input=%s twist_output=%s",
                toString(config_.model.type), config_.input_topic.c_str(),
                config_.output_topic.c_str());
    if (config_.publish_joint_commands) {
        RCLCPP_INFO(node_.get_logger(), "chassis joint command output=%s modules=%zu",
                    config_.joint_command_topic.c_str(), config_.kinematics.modules.size());
    }
}

void ChassisController::handleCommand(const geometry_msgs::msg::Twist& msg) {
    if (!config_.model.supportsLateralVelocity() && std::abs(msg.linear.y) > 1e-6 &&
        config_.warn_on_unsupported_lateral) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                             "chassis type %s ignores lateral velocity vy=%.4f",
                             toString(config_.model.type), msg.linear.y);
    }

    const ChassisVelocityCommand command = adapter_.fromTwist(msg);
    const CommandValidation validation = adapter_.validate(command);
    if (!validation.ok) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                             "invalid chassis command: %s", validation.error.c_str());
        return;
    }

    publishTwistCommand(command);

    if (config_.publish_joint_commands) {
        publishJointCommands(kinematics_.computeJointCommands(command));
    }
}

void ChassisController::publishTwistCommand(const ChassisVelocityCommand& command) {
    if (!twist_publisher_) {
        return;
    }

    twist_publisher_->publish(adapter_.toTwist(command));
}

void ChassisController::publishJointCommands(const ChassisJointCommandSet& commands) {
    if (!joint_command_publisher_ || commands.empty()) {
        return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_.now();
    msg.name.reserve(commands.joints.size());

    bool has_any_position = false;
    bool has_any_velocity = false;
    for (const ChassisJointCommand& command : commands.joints) {
        has_any_position = has_any_position || command.has_position;
        has_any_velocity = has_any_velocity || command.has_velocity;
    }

    if (has_any_position) {
        msg.position.reserve(commands.joints.size());
    }
    if (has_any_velocity) {
        msg.velocity.reserve(commands.joints.size());
    }

    for (const ChassisJointCommand& command : commands.joints) {
        msg.name.push_back(command.name);
        if (has_any_position) {
            msg.position.push_back(command.has_position ? command.position
                                                        : std::numeric_limits<double>::quiet_NaN());
        }
        if (has_any_velocity) {
            msg.velocity.push_back(command.has_velocity ? command.velocity
                                                        : std::numeric_limits<double>::quiet_NaN());
        }
    }

    joint_command_publisher_->publish(msg);
}

}  // namespace chassis_controller
