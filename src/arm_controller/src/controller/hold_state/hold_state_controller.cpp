#include "hold_state_controller.hpp"
#include "controller_interface.hpp"
#include <chrono>
#include <thread>

HoldStateController::HoldStateController(const rclcpp::Node::SharedPtr& node)
    : UtilityControllerBase("HoldState", node)
{
    // HoldState控制器不需要topic，直接初始化hardware_manager
    hardware_manager_ = HardwareManager::getInstance();
    RCLCPP_INFO(node_->get_logger(), "HoldStateController initialized");
}

void HoldStateController::start(const std::string& mapping) {
    const std::string normalized_mapping = normalize_mapping(mapping);

    // 如果已经为该 mapping 激活，直接返回（幂等）
    if (mapping_contexts_.count(normalized_mapping)) {
        RCLCPP_WARN(node_->get_logger(), "HoldStateController already active for mapping '%s'", normalized_mapping.c_str());
        return;
    }

    MappingContext ctx;
    ctx.transition_ready = true;
    ctx.system_health_check_paused = false;

    // ========== 关键：检查速度，如果不为0则发送保持命令 ==========
    if (hardware_manager_) {
        // 获取当前关节位置和速度
        ctx.hold_positions = hardware_manager_->get_current_joint_positions(normalized_mapping);
        auto current_velocities = hardware_manager_->get_current_joint_velocities(normalized_mapping);

        if (!ctx.hold_positions.empty()) {
            // 检查是否所有关节速度都接近0（阈值：0.01 rad/s）
            bool all_velocities_zero = true;
            const double VELOCITY_THRESHOLD = 0.01;
            for (const auto& vel : current_velocities) {
                if (std::abs(vel) > VELOCITY_THRESHOLD) {
                    all_velocities_zero = false;
                    break;
                }
            }

            // 如果速度不为0，发送保持命令来停止电机；否则不需要做任何处理
            if (!all_velocities_zero) {
                RCLCPP_INFO(node_->get_logger(),
                           "[%s] Robot is moving, sending hold command to stop at current position",
                           normalized_mapping.c_str());
                hardware_manager_->send_hold_state_command(normalized_mapping, ctx.hold_positions);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(),
                       "[%s] Failed to get current joint positions for hold state",
                       normalized_mapping.c_str());
        }
    }
    // =======================================================

    // 重置 mapping 的系统健康
    if (hardware_manager_) {
        hardware_manager_->reset_system_health(normalized_mapping);
    }

    // 为该 mapping 创建定时器（仅用于安全检查，不发送保持命令）
    ctx.safety_timer = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        [this, normalized_mapping]() { this->safety_check_timer_callback(normalized_mapping); }
    );

    mapping_contexts_.emplace(normalized_mapping, std::move(ctx));
    is_active_ = true;

    RCLCPP_INFO(node_->get_logger(), "HoldStateController activated for mapping '%s'", normalized_mapping.c_str());
}

bool HoldStateController::stop(const std::string& mapping) {
    const std::string normalized_mapping = normalize_mapping(mapping);

    auto it = mapping_contexts_.find(normalized_mapping);
    if (it == mapping_contexts_.end()) {
        RCLCPP_WARN(node_->get_logger(), "HoldStateController not active for mapping '%s'", normalized_mapping.c_str());
        return false;
    }

    // 取消该 mapping 的定时器
    if (it->second.safety_timer) {
        it->second.safety_timer->cancel();
        it->second.safety_timer.reset();
    }

    // 移除上下文
    mapping_contexts_.erase(it);

    RCLCPP_INFO(node_->get_logger(), "HoldStateController stopped for mapping '%s'", normalized_mapping.c_str());

    // 如果没有任何 mapping 在运行，则整体设置 inactive
    if (mapping_contexts_.empty()) {
        is_active_ = false;
        RCLCPP_INFO(node_->get_logger(), "HoldStateController fully deactivated (no active mappings)");
    }

    return true;
}

bool HoldStateController::can_transition_to_target(const std::string& mapping) {
    const std::string normalized_mapping = normalize_mapping(mapping);

    if (!hardware_manager_) {
        RCLCPP_WARN(node_->get_logger(), "[%s] HardwareManager not available for safety check", mapping.c_str());
        return false;
    }

    // 如果目标是快速切换模式，直接允许
    if (target_mode_ == "Disable" || target_mode_ == "EmergencyStop") {
        RCLCPP_INFO(node_->get_logger(), "[%s] Target %s - skipping safety checks", normalized_mapping.c_str(), target_mode_.c_str());
        return true;
    }


    // 持续监控安全状态，所有条件都必须同时满足
    bool is_robot_stopped = hardware_manager_->is_robot_stopped(normalized_mapping);
    bool are_joints_within_limits = hardware_manager_->are_joints_within_limits(normalized_mapping);
    bool is_system_healthy = hardware_manager_->is_system_healthy(normalized_mapping);

    // 详细记录当前状态，帮助调试
    if (!is_robot_stopped) {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Transition blocked: Robot is still moving");
    }
    if (!are_joints_within_limits) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Transition blocked: Joints are outside safety limits");
    }
    if (!is_system_healthy) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "Transition blocked: System is not healthy");
        // 每5秒打印一次详细系统状态
        static auto last_debug_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_debug_time).count() >= 5) {
            hardware_manager_->print_system_status();
            last_debug_time = now;
        }

        // 系统不健康时暂停健康检查，但保留定时器
        auto& ctx = mapping_contexts_[normalized_mapping];
        if (!ctx.system_health_check_paused) {
            ctx.system_health_check_paused = true;
            RCLCPP_WARN(node_->get_logger(), "[%s] System unhealthy - pausing safety check", normalized_mapping.c_str());
        }
        return false;
    }
    auto& ctx = mapping_contexts_[normalized_mapping];
    if (!ctx.transition_ready) {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "[%s] Transition blocked: Basic transition check not ready", normalized_mapping.c_str());
    }

    // 只有当所有安全条件同时满足时才允许转换
    bool all_conditions_met = ctx.transition_ready && is_robot_stopped && are_joints_within_limits && is_system_healthy;

    if (all_conditions_met) {
        RCLCPP_INFO(node_->get_logger(), "All safety conditions met - transition allowed to %s", target_mode_.c_str());
    }

    return all_conditions_met;
}

void HoldStateController::safety_check_timer_callback(const std::string& mapping) {
    const std::string normalized_mapping = normalize_mapping(mapping);

    auto it = mapping_contexts_.find(normalized_mapping);
    if (it == mapping_contexts_.end()) {
        return;
    }

    auto& ctx = it->second;
    // 只有在激活状态下才进行检查
    if (!is_active_) {
        return;
    }

    // ========== 仅进行安全检查，不发送保持命令 ==========
    // 如果健康检查被暂停，检查系统是否恢复健康
    if (ctx.system_health_check_paused) {
        if (hardware_manager_ && hardware_manager_->is_system_healthy(normalized_mapping)) {
            ctx.system_health_check_paused = false;
            RCLCPP_INFO(node_->get_logger(), "[%s] System recovered - resuming safety checks", normalized_mapping.c_str());
        } else {
            // 系统仍未恢复，继续暂停
            return;
        }
    }

    // 只在有目标模式且有回调时才检查转换条件
    if (!target_mode_.empty() && transition_ready_callback_) {
        // 检查是否可以安全转换到目标状态
        if (can_transition_to_target(normalized_mapping)) {
            RCLCPP_INFO(node_->get_logger(), "Safety conditions satisfied, triggering transition to %s", target_mode_.c_str());

            // 关键：在调用回调前停止定时器，避免竞态条件
            if (ctx.safety_timer) {
                ctx.safety_timer->cancel();
                ctx.safety_timer.reset();
            }

            // 调用回调 - 执行实际的转换
            transition_ready_callback_();
        }
        // 如果条件不满足，继续等待下一次检查
    }
    // 如果没有目标模式，说明这是 HoldState 终止状态，保持等待
}
