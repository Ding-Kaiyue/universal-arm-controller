#include "hold_state_controller.hpp"
#include "controller_interface.hpp"

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

    // ========== 关键：根据前一个模式选择保持策略 ==========
    if (hardware_manager_) {
        // 判断前一个模式是否为速度控制模式
        bool is_velocity_mode = (previous_mode_ == "JointVelocity" ||
                                 previous_mode_ == "ServoMode" ||
                                 previous_mode_.find("Velocity") != std::string::npos);

        if (is_velocity_mode) {
            // 速度模式：发送零速度命令保持
            RCLCPP_INFO(node_->get_logger(),
                       "[%s] HoldState using velocity hold strategy (previous mode: %s)",
                       normalized_mapping.c_str(), previous_mode_.c_str());
            hardware_manager_->send_hold_velocity_command(normalized_mapping);
        } else {
            // 位置模式：锁定当前位置
            ctx.hold_positions = hardware_manager_->get_current_joint_positions(normalized_mapping);

            if (!ctx.hold_positions.empty()) {
                RCLCPP_INFO(node_->get_logger(),
                           "[%s] HoldState using position hold strategy with %zu joints (previous mode: %s)",
                           normalized_mapping.c_str(), ctx.hold_positions.size(), previous_mode_.c_str());

                // 立即发送一次保持位置命令
                hardware_manager_->send_hold_position_command(normalized_mapping, ctx.hold_positions);
            } else {
                RCLCPP_WARN(node_->get_logger(),
                           "[%s] Failed to get current joint positions for hold state",
                           normalized_mapping.c_str());
            }
        }
    }
    // =======================================================

    // 重置 mapping 的系统健康
    if (hardware_manager_) {
        hardware_manager_->reset_system_health(normalized_mapping);
    }

    // 为该 mapping 创建定时器（捕获 mapping 的副本）
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

    // 取消该 mapping 的定时器并移除上下文
    if (it->second.safety_timer) {
        it->second.safety_timer->cancel();
        it->second.safety_timer.reset();
    }
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

    // ========== 持续发送保持命令（根据前一个模式选择策略） ==========
    if (hardware_manager_) {
        // 判断前一个模式是否为速度控制模式
        bool is_velocity_mode = (previous_mode_ == "JointVelocity" ||
                                 previous_mode_ == "CartesianVelocity" ||
                                 previous_mode_.find("Velocity") != std::string::npos);

        if (is_velocity_mode) {
            // 速度模式：持续发送零速度命令
            hardware_manager_->send_hold_velocity_command(normalized_mapping);
        } else if (!ctx.hold_positions.empty()) {
            // 位置模式：持续发送锁定的目标位置
            hardware_manager_->send_hold_position_command(normalized_mapping, ctx.hold_positions);
        }
    }
    // ===========================================================

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
            transition_ready_callback_();

            // 转换完成后停止定时器，避免重复触发
            if (ctx.safety_timer) {
                ctx.safety_timer->cancel();
                ctx.safety_timer.reset();
            }
        }
        // 如果条件不满足，继续等待下一次检查
    }
    // 如果没有目标模式，说明这是终止状态（如轨迹执行完成后），只保持位置，不尝试转换
} 
