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

    // ✅ 创建 10ms 控制定时器（实时循环）
    control_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&JointVelocityController::control_loop, this));

    RCLCPP_INFO(node_->get_logger(), "[%s] ✓ JointVelocityController activated with 10ms control loop", mapping.c_str());
}

bool JointVelocityController::stop(const std::string& mapping) {
    is_active_ = false;

    // ✅ 销毁 10ms 控制定时器
    if (control_timer_) {
        control_timer_.reset();
        RCLCPP_INFO(node_->get_logger(), "[%s] Control loop stopped", mapping.c_str());
    }

    // 发送零速度命令停止机械臂
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    if (!joint_names.empty()) {
        std::vector<double> zero_velocities(joint_names.size(), 0.0);
        send_joint_velocities(mapping, zero_velocities);
    }

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] JointVelocityController deactivated", mapping.c_str());
    return true;
}

void JointVelocityController::velocity_callback(const std::string& mapping, const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!is_active_) return;

    // ✅ Velocity latch 模式：仅缓存最新命令
    // 实际的速度控制在 control_loop 中以 10ms 频率进行

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_cmd_ = *msg;
        // ✅ 使用 steady_clock（单调时间）确保工业级时间源一致
        last_cmd_time_ = steady_clock_.now();
    }

    RCLCPP_INFO(node_->get_logger(),
        "[%s] ✓ Joint velocity command received: %zu joints",
        active_mapping_.c_str(), msg->velocity.size());
}

void JointVelocityController::control_loop() {
    /* ========================================
     * 10ms 实时控制循环
     * 关键特性：
     * - 每 10ms 执行一次速度命令
     * - 命令 latch（缓存最新速度）
     * - 100ms 超时保护
     * ======================================== */

    if (!is_active_ || active_mapping_.empty()) {
        return;
    }

    // 获取最新的速度命令（带超时检查）
    sensor_msgs::msg::JointState cmd;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);

        // ✅ 使用 steady_clock 进行超时检查（单调时间，不受 use_sim_time 影响）
        if ((steady_clock_.now() - last_cmd_time_).count() > 100000000) {  // 100ms in nanoseconds
            // 命令超时 → 紧急停止
            auto joint_names = hardware_manager_->get_joint_names(active_mapping_);
            std::vector<double> zero(joint_names.size(), 0.0);
            send_joint_velocities(active_mapping_, zero);
            return;
        }
        cmd = last_cmd_;
    }

    // 检查长度匹配
    const auto& joint_names = hardware_manager_->get_joint_names(active_mapping_);
    if (cmd.velocity.size() != joint_names.size()) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] Velocity vector size mismatch: expected %zu, got %zu",
            active_mapping_.c_str(), joint_names.size(), cmd.velocity.size());
        return;
    }

    // 发送关节速度命令 (实时安全检查已在HardwareManager中实现)
    send_joint_velocities(active_mapping_, cmd.velocity);
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

        // MIT模式速度控制参数
        const double kp_velocity = 0.0;      // 速度模式：kp=0.0
        const double kd_velocity = 0.01;     // 速度模式：kd=0.01
        const double position = 0.0;         // 位置在纯速度模式下不使用

        // 获取当前关节位置对应的重力矩
        std::vector<double> gravity_torques = hardware_manager_->compute_gravity_torques(mapping);

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            const auto& joint_name = joint_names[i];
            uint32_t motor_id = motor_ids[i];
            double vel = joint_velocities[i] * 180.0 / M_PI;  // 转为度/秒
            double effort = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;  // 重力补偿力矩

            int violation_dir = hardware_manager_->get_joint_violation_direction(joint_name);
            if (hardware_manager_->is_joint_emergency_stopped(joint_name) &&
                ((violation_dir < 0 && vel < 0.0) || (violation_dir > 0 && vel > 0.0))) {
                // 不允许继续违规，发送零速度
                hardware_driver->control_motor_in_mit_mode(interface, motor_id, position, 0.0, effort, kp_velocity, kd_velocity);
                auto clock = node_->get_clock();
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *clock,
                    2000,
                    "[%s] Joint '%s' emergency stopped (dir=%d), unsafe velocity %.3f -> skipping.",
                    mapping.c_str(), joint_name.c_str(), violation_dir, vel);
            } else {
                hardware_driver->control_motor_in_mit_mode(interface, motor_id, position, vel, effort, kp_velocity, kd_velocity);
            }
        }

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to send joint velocities: %s", mapping.c_str(), e.what());
        return false;
    }
}

