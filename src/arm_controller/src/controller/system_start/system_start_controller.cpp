#include "system_start_controller.hpp"
#include "controller_interface.hpp"

// This controller represents the state when the system is started
// It will be automatically activated when the robot system starts up

SystemStartController::SystemStartController(const rclcpp::Node::SharedPtr& node)
    : UtilityControllerBase("SystemStart", node)
{
    hardware_manager_ = HardwareManager::getInstance();
}

SystemStartController::~SystemStartController() {}

void SystemStartController::start(const std::string& mapping) {
    is_active_ = true;

    // 验证硬件管理器可用性
    if (!hardware_manager_) {
        RCLCPP_WARN(node_->get_logger(), "[%s] SystemStart: Hardware manager not available", mapping.c_str());
        return;
    }

    if (!hardware_manager_->is_system_healthy(mapping)) {
        RCLCPP_WARN(node_->get_logger(), "[%s] SystemStart: System health check failed", mapping.c_str());
        return;
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return;
    }

    try {
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);

        // 使能所有电机的MIT模式
        hardware_driver->enable_motors(interface, motor_ids, static_cast<uint8_t>(MotorControlMode::MIT_MODE));
        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Enabled %zu motors in MIT mode",
                   mapping.c_str(), motor_ids.size());

        // 获取当前关节位置并发送保持命令
        auto current_positions = hardware_manager_->get_current_joint_positions(mapping);
        if (!current_positions.empty()) {
            hardware_manager_->send_hold_state_command(mapping, current_positions);
            RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Holding current position with %zu joints",
                       mapping.c_str(), current_positions.size());
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] ❎ No current positions available yet", mapping.c_str());
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Error during SystemStart initialization: %s",
                    mapping.c_str(), e.what());
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ SystemStart: System initialized and ready", mapping.c_str());
}

bool SystemStartController::stop(const std::string& mapping) {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] SystemStartController deactivated", mapping.c_str());
    return true;
}
