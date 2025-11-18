#include "ros2_action_control_controller.hpp"
#include "controller_interface.hpp"

// This controller represents the state when robot is controlled via ROS2 action server
// It will be automatically activated when trajectory_controller_node receives action goals

ROS2ActionControlController::ROS2ActionControlController(const rclcpp::Node::SharedPtr& node)
    : UtilityControllerBase("ROS2ActionControl", node)
{
    hardware_manager_ = HardwareManager::getInstance();
}

ROS2ActionControlController::~ROS2ActionControlController() {}

void ROS2ActionControlController::start(const std::string& mapping) {
    is_active_ = true;

    // 验证硬件管理器可用性
    if (!hardware_manager_) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ROS2ActionControl: Hardware manager not available", mapping.c_str());
        return;
    }

    if (!hardware_manager_->is_system_healthy(mapping)) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ROS2ActionControl: System health check failed", mapping.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "ROS2ActionControl: Ready to execute trajectories via action server");
}

bool ROS2ActionControlController::stop(const std::string& mapping) {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] ROS2ActionControlController deactivated", mapping.c_str());
    return true;
}
