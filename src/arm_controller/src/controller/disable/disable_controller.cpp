#include "disable_controller.hpp"
#include "controller_interface.hpp"
#include <stdexcept>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'Disable', mapping: 'single_arm'}"

DisableController::DisableController(const rclcpp::Node::SharedPtr & node)
    : UtilityControllerBase("Disable", node)
{
    hardware_manager_ = HardwareManager::getInstance();
}

DisableController::~DisableController() { }

void DisableController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] Disable: not found in hardware configuration. Skip disable operation."
        );
    }

    // 重置系统安全状态
    if (hardware_manager_) {
        hardware_manager_->clear_emergency_stops(mapping);
        hardware_manager_->reset_system_health(mapping);
    }

    if (!disable_motors(mapping)) {
        throw std::runtime_error(
            "❎ [" + mapping + "] Disable: Failed to disable motors");
    }

    is_active_ = true;
}

bool DisableController::stop(const std::string& mapping) {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] DisableController deactivated", mapping.c_str());
    return true; 
}

bool DisableController::disable_motors(const std::string& mapping) {
    if (!hardware_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "Hardware manager not initialized");
        return false;
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "Hardware driver not initialized");
        return false;
    }

    try {
        // 失能指定mapping的电机
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);

        // 检查mapping是否有效
        if (interface.empty() || motor_ids.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Mapping '%s' not found in configuration", mapping.c_str());
            return false;
        }

        for (const auto& motor_id : motor_ids) {
            hardware_driver->disable_motor(interface, motor_id);
            RCLCPP_DEBUG(node_->get_logger(), "[%s] Disabled motor %u on interface %s", 
                        mapping.c_str(), motor_id, interface.c_str());
        }
        RCLCPP_INFO(node_->get_logger(), "[%s] All motors disabled successfully", mapping.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to disable motors: %s", mapping.c_str(), e.what());
        return false;
    }
}


