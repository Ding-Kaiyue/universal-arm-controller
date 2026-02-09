#include "joint_velocity_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <stdexcept>
#include <algorithm>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'JointVelocity', mapping: 'left_arm'}"
// ros2 topic pub --once /controller_api/joint_velocity_action/left_arm sensor_msgs/msg/JointState "{velocity: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"

JointVelocityController::JointVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<sensor_msgs::msg::JointState>("JointVelocity", node)
{
    // 获取HardwareManager实例
    hardware_manager_ = HardwareManager::getInstance();

    // 启动IPC命令队列消费线程（早期启动以接收API发送的命令）
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&JointVelocityController::command_queue_consumer_thread, this);
    }

    // 注意：话题订阅在 init_subscriptions() 中创建，当 controller 被激活时调用
    RCLCPP_INFO(node_->get_logger(), "JointVelocityController initialized");
}

void JointVelocityController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] JointVelocity: not found in hardware configuration."
        );
    }

    // 调用基类 start() 设置 per-mapping 的 active_mappings_[mapping] = true
    VelocityControllerImpl::start(mapping);

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    // 为此 mapping 初始化控制状态
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (mapping_states_.find(mapping) == mapping_states_.end()) {
            mapping_states_[mapping] = MappingControlState();
            mapping_states_[mapping].last_cmd_time = steady_clock_.now();
            // 初始化速度向量为零
            auto joint_names = hardware_manager_->get_joint_names(mapping);
            mapping_states_[mapping].last_cmd.velocity.resize(joint_names.size(), 0.0);
        }
    }

    // 为此 mapping 创建 10ms 控制定时器（支持多臂）
    mapping_states_[mapping].control_timer = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        [this, mapping]() { control_loop(mapping); });

    RCLCPP_INFO(node_->get_logger(), "[%s] ✓ JointVelocityController activated with 10ms control loop", mapping.c_str());
}

bool JointVelocityController::stop(const std::string& mapping) {
    // 调用基类 stop() 设置 per-mapping 的 active_mappings_[mapping] = false
    VelocityControllerImpl::stop(mapping);

    // 销毁此 mapping 的 10ms 控制定时器
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        if (it != mapping_states_.end() && it->second.control_timer) {
            it->second.control_timer.reset();
            RCLCPP_INFO(node_->get_logger(), "[%s] Control loop stopped", mapping.c_str());
        }
    }

    // 发送零速度命令停止机械臂
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    if (!joint_names.empty()) {
        std::vector<double> zero_velocities(joint_names.size(), 0.0);
        send_joint_velocities(mapping, zero_velocities);
    }

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    // 清理该 mapping 的控制状态
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        mapping_states_.erase(mapping);
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] JointVelocityController deactivated", mapping.c_str());
    return true;
}

void JointVelocityController::velocity_callback(const std::string& mapping, const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Velocity latch 模式：仅缓存最新命令
    // 实际的速度控制在 control_loop 中以 10ms 频率进行
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        if (it != mapping_states_.end()) {
            it->second.last_cmd = *msg;
            // 使用 steady_clock（单调时间）确保工业级时间源一致
            it->second.last_cmd_time = steady_clock_.now();
        }
    }

    RCLCPP_INFO(node_->get_logger(),
        "[%s] ✓ Joint velocity command received: %zu joints",
        mapping.c_str(), msg->velocity.size());
}

void JointVelocityController::control_loop(const std::string& mapping) {
    /* ========================================
     * 10ms 实时控制循环
     * 关键特性：
     * - 每 10ms 执行一次速度命令
     * - 命令 latch（缓存最新速度）
     * - 100ms 超时保护
     * ======================================== */

    // 检查该 mapping 是否仍然活跃
    if (!is_active(mapping)) {
        return;
    }

    // 获取此 mapping 的最新速度命令（带超时检查）
    sensor_msgs::msg::JointState cmd;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        if (it == mapping_states_.end()) {
            return;
        }

        // ✅ 使用 steady_clock 进行超时检查（单调时间，不受 use_sim_time 影响）
        auto now = steady_clock_.now();
        auto time_delta = (now - it->second.last_cmd_time).count();
        if (time_delta > 100000000) {  // 100ms in nanoseconds
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ TIMEOUT TRIGGERED at %ld ns", mapping.c_str(), time_delta);
            // 命令超时 → 紧急停止
            auto joint_names = hardware_manager_->get_joint_names(mapping);
            std::vector<double> zero(joint_names.size(), 0.0);
            send_joint_velocities(mapping, zero);
            return;
        }
        cmd = it->second.last_cmd;
    }

    // 检查长度匹配
    const auto& joint_names = hardware_manager_->get_joint_names(mapping);
    if (cmd.velocity.size() != joint_names.size()) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] Velocity vector size mismatch: expected %zu, got %zu",
            mapping.c_str(), joint_names.size(), cmd.velocity.size());
        return;
    }

    // 发送关节速度命令 (实时安全检查已在HardwareManager中实现)
    send_joint_velocities(mapping, cmd.velocity);
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

bool JointVelocityController::send_velocity(const std::string& mapping, const std::vector<double>& velocity) {
    // ⭐ 更新命令时间戳，防止超时误触
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        if (it != mapping_states_.end()) {
            it->second.last_cmd_time = steady_clock_.now();
        }
    }
    return send_joint_velocities(mapping, velocity);
}

void JointVelocityController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;
    std::map<std::string, std::string> current_mode;
    std::map<std::string, arm_controller::ipc::ExecutionState> last_state;  // Track last execution state per mapping

    while (consumer_running_) {
        // 使用带过滤的 pop，只获取 JointVelocity 命令
        // popWithFilter 会阻塞直到有匹配的命令（顺序执行）
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "JointVelocity")) {
            continue;  // 只在异常时继续
        }

        std::string mode = cmd.get_mode();
        std::string mapping = cmd.get_mapping();
        std::string cmd_id = cmd.get_command_id();

        // 获取 per-mapping 的互斥锁，确保同一手臂的命令串行执行
        std::lock_guard<std::mutex> execution_lock(arm_controller::CommandQueueIPC::getMappingExecutionMutex(mapping));

        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        try {
            // 获取状态管理器并更新为执行中
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::EXECUTING);
                last_state[mapping] = arm_controller::ipc::ExecutionState::EXECUTING;
            }

            auto params = cmd.get_parameters();
            bool success = send_velocity(mapping, params);

            if (success) {
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::SUCCESS);
                    last_state[mapping] = arm_controller::ipc::ExecutionState::SUCCESS;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ JointVelocity command execution failed (ID: %s)",
                           mapping.c_str(), cmd_id.c_str());
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                    last_state[mapping] = arm_controller::ipc::ExecutionState::FAILED;
                }
            }

            // 延迟后恢复到 IDLE，给下一条命令足够的时间看到最终状态
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);
                // 命令执行完成后
                arm_controller::ipc::ExecutorControllerState executor_state;
                strncpy(executor_state.current_mode, mode.c_str(), sizeof(executor_state.current_mode) - 1);
                executor_state.current_mode[sizeof(executor_state.current_mode) - 1] = '\0';  // 确保字符串以空字符结尾
                executor_state.execution_state = (int)arm_controller::ipc::ExecutionState::IDLE;
                state_mgr->updateFromExecutor(executor_state);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception in JointVelocity command execution: %s",
                        mapping.c_str(), e.what());
            if (state_mgr) {
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                last_state[mapping] = arm_controller::ipc::ExecutionState::FAILED;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);
            }
        }
        // 通知其他 consumers
        arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
    }
}