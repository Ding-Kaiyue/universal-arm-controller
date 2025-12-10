#include "joint_velocity_controller.hpp"
#include "controller_interface.hpp"
#include <stdexcept>
#include <algorithm>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'JointVelocity', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/joint_velocity_action/single_arm sensor_msgs/msg/JointState "{velocity: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"

JointVelocityController::JointVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<sensor_msgs::msg::JointState>("JointVelocity", node)
{
    // 获取HardwareManager实例
    hardware_manager_ = HardwareManager::getInstance();

    // 注意：话题订阅在 init_subscriptions() 中创建，当 controller 被激活时调用
    RCLCPP_INFO(node_->get_logger(), "JointVelocityController initialized");
}

JointVelocityController::~JointVelocityController() { }

void JointVelocityController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "Mapping '" + mapping + "' not found in hardware configuration. Skip disable operation."
        );
    }

    // 保存当前激活的 mapping
    active_mapping_ = mapping;
    is_active_ = true;

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] JointVelocityController activated", mapping.c_str());
}

bool JointVelocityController::stop(const std::string& mapping) {
    is_active_ = false;

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] JointVelocityController deactivated", mapping.c_str());
    return true;  // 需要钩子状态来安全停止
}

void JointVelocityController::velocity_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!is_active_) return;

    // 检查长度匹配
    const auto& joint_names = hardware_manager_->get_joint_names(active_mapping_);
    if (msg->velocity.size() != joint_names.size()) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] Velocity vector size mismatch: expected %zu, got %zu",
            active_mapping_.c_str(), joint_names.size(), msg->velocity.size());
        return;
    }

    // 发送关节速度命令 (实时安全检查已在HardwareManager中实现)
    send_joint_velocities(active_mapping_, msg->velocity);
}
    

bool JointVelocityController::send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities) {
    if (!hardware_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware manager not initialized");
        return false;
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return false;
    }

    try {
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);
        const std::vector<std::string>& joint_names = hardware_manager_->get_joint_names(mapping);

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            const auto& joint_name = joint_names[i];
            uint32_t motor_id = motor_ids[i];
            double vel = joint_velocities[i] * 180.0 / M_PI;  // 转为度/秒

            // 检查急停状态
            if (hardware_manager_->is_joint_emergency_stopped(joint_name)) {
                int violation_dir = hardware_manager_->get_joint_violation_direction(joint_name);

                // 允许反方向运动或离开限位
                if ((violation_dir < 0 && vel >= 0.0) || (violation_dir > 0 && vel <= 0.0)) {
                    hardware_driver->control_motor_in_velocity_mode(interface, motor_id, vel);
                    RCLCPP_INFO(node_->get_logger(),
                        "[%s] Joint '%s' emergency stopped but moving in safe direction (%.3f deg/s).",
                        mapping.c_str(), joint_name.c_str(), vel);
                } else {
                    // 不允许继续违规，发送零速度
                    hardware_driver->control_motor_in_velocity_mode(interface, motor_id, 0.0);
                    auto clock = node_->get_clock();
                    RCLCPP_WARN_THROTTLE(
                        node_->get_logger(),
                        *clock,
                        2000,
                        "[%s] Joint '%s' emergency stopped (dir=%d), unsafe velocity %.3f -> skipping.",
                        mapping.c_str(), joint_name.c_str(), violation_dir, vel);
                }
            } else {
                // 未急停，直接发送速度
                hardware_driver->control_motor_in_velocity_mode(interface, motor_id, vel);
            }
        }

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to send joint velocities: %s", mapping.c_str(), e.what());
        return false;
    }
}

