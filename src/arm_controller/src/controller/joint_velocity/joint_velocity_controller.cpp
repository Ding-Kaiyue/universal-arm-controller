#include "joint_velocity_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <stdexcept>
#include <algorithm>
#include <cmath>

JointVelocityController::JointVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<sensor_msgs::msg::JointState>("JointVelocity", node)
{
    // 获取HardwareManager实例
    hardware_manager_ = HardwareManager::getInstance();

    RCLCPP_WARN(node_->get_logger(), "[JointVelocityController] ⚠️  CONSTRUCTOR CALLED - new instance created, this=%px",
                static_cast<void*>(this));

    // 启动IPC命令队列消费线程（早期启动以接收API发送的命令）
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&JointVelocityController::command_queue_consumer_thread, this);
    }

    // 注意：话题订阅在 init_subscriptions() 中创建，当 controller 被激活时调用
    RCLCPP_INFO(node_->get_logger(), "JointVelocityController initialized");
}

void JointVelocityController::start(const std::string& mapping) {
    // 检查是否已经初始化（通过检查timer是否存在）
    // ⭐ 使用timer存在性而不是is_active()，因为timer是可靠的per-mapping指标
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        RCLCPP_INFO(node_->get_logger(), "[%s] DEBUG start(): mapping_states_ size=%zu", mapping.c_str(), mapping_states_.size());
        auto it = mapping_states_.find(mapping);

        // 详细的诊断日志
        if (it == mapping_states_.end()) {
            RCLCPP_INFO(node_->get_logger(), "[%s] DEBUG start(): mapping_states_ entry does NOT exist", mapping.c_str());
        } else {
            bool timer_valid = (it->second.control_timer != nullptr);
            RCLCPP_INFO(node_->get_logger(), "[%s] DEBUG start(): mapping_states_ entry exists, timer=%s",
                        mapping.c_str(), timer_valid ? "valid" : "null");
            if (timer_valid) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Timer already exists, skipping re-initialization", mapping.c_str());
                return;
            }
        }
    }

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

    // 为此 mapping 初始化控制状态（只在第一次）
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);

        // ⭐ 只在第一次初始化，不要覆盖已有的状态！
        if (mapping_states_.find(mapping) == mapping_states_.end()) {
            mapping_states_[mapping] = MappingControlState();
            const auto& joint_names = hardware_manager_->get_joint_names(mapping);
            mapping_states_[mapping].last_cmd_velocity.assign(joint_names.size(), 0.0);
            // 不初始化 last_cmd_time，让它保持默认值（0 / epoch）
            RCLCPP_INFO(node_->get_logger(), "[%s] State initialized in start(), mapping_states_ size=%zu",
                        mapping.c_str(), mapping_states_.size());
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] State already exists, skipping initialization", mapping.c_str());
        }
    }

    // ⭐ 为此 mapping 创建 10ms 控制定时器（支持多臂）
    // 重要：先创建，然后在锁内保存
    auto timer = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        [this, mapping]() { control_loop(mapping); }
    );

    // 在锁内保存定时器 - 必须确保 entry 存在且 timer 被正确保存
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        if (it != mapping_states_.end()) {
            // ⭐ 只有在 entry 存在时才保存 timer
            it->second.control_timer = timer;
            RCLCPP_INFO(node_->get_logger(), "[%s] ✓ 10ms control loop started, timer saved successfully",
                        mapping.c_str());
        } else {
            // 如果 entry 消失了，这是严重错误
            RCLCPP_ERROR(node_->get_logger(), "[%s] CRITICAL ERROR: mapping_states_ entry disappeared before timer save!",
                         mapping.c_str());
            // timer 会被自动析构
            return;
        }
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] ✓ JointVelocityController activated", mapping.c_str());
}

bool JointVelocityController::stop(const std::string& mapping) {
    RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  STOP CALLED! mapping_states_ size=%zu", mapping.c_str(), mapping_states_.size());

    // 清理 timer
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        if (it != mapping_states_.end()) {
            if (it->second.control_timer) {
                RCLCPP_INFO(node_->get_logger(), "[%s] DEBUG stop(): Resetting timer", mapping.c_str());
                it->second.control_timer.reset();
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] DEBUG stop(): mapping state not found!", mapping.c_str());
        }
    }

    // 调用基类 stop() 设置 active_mappings_[mapping] = false
    VelocityControllerImpl::stop(mapping);

    // 发送零速度确保立即停止
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    if (!joint_names.empty()) {
        std::vector<double> zero_velocities(joint_names.size(), 0.0);
        send_joint_velocities(mapping, zero_velocities);
    }

    cleanup_subscriptions(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] JointVelocityController deactivated, mapping_states_ size=%zu",
                mapping.c_str(), mapping_states_.size());
    return true;
}

void JointVelocityController::velocity_callback(const std::string& mapping, const sensor_msgs::msg::JointState::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        if (it == mapping_states_.end()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] JointVelocity: mapping state not initialized", mapping.c_str());
            return;
        }

        // 仅更新缓存：速度值 + 时间戳
        it->second.last_cmd_velocity = msg->velocity;
        it->second.last_cmd_time = steady_clock_.now();
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] ✓ Joint velocity command received: %zu joints",
                mapping.c_str(), msg->velocity.size());
}

void JointVelocityController::control_loop(const std::string& mapping) {
    // 读取当前状态
    std::vector<double> velocity_to_send;
    std::chrono::steady_clock::time_point last_cmd_time;

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        if (it == mapping_states_.end()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] DEBUG control_loop: mapping state not found!", mapping.c_str());
            return;
        }

        velocity_to_send = it->second.last_cmd_velocity;
        last_cmd_time = it->second.last_cmd_time;
    }

    // ⭐ 如果 last_cmd_time 还是默认值（0），说明还没有命令到达
    // 检查是否为 epoch 时刻（等于 0）
    if (last_cmd_time == std::chrono::steady_clock::time_point()) {
        // 还没有收到任何命令，发送零速度等待
        const auto& joint_names = hardware_manager_->get_joint_names(mapping);
        std::vector<double> zero_velocities(joint_names.size(), 0.0);
        send_joint_velocities(mapping, zero_velocities);
        return;
    }

    auto now = steady_clock_.now();
    auto time_since_last_cmd = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_time).count();

    // ⭐ 超时检查：如果距离上一次命令 >100ms，发送零速度
    if (time_since_last_cmd > 100) {
        // 发送零速度作为紧急停止
        const auto& joint_names = hardware_manager_->get_joint_names(mapping);
        std::vector<double> zero_velocities(joint_names.size(), 0.0);
        send_joint_velocities(mapping, zero_velocities);

        RCLCPP_WARN(node_->get_logger(),
            "[%s] JointVelocity timeout (>100ms, delta=%ldms), sending zero velocity",
            mapping.c_str(), time_since_last_cmd);
        return;
    }

    // 检查速度向量有效性
    const auto& joint_names = hardware_manager_->get_joint_names(mapping);
    if (velocity_to_send.size() != joint_names.size()) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Velocity vector size mismatch: expected %zu, got %zu",
                    mapping.c_str(), joint_names.size(), velocity_to_send.size());
        return;
    }

    // 正常情况：发送当前缓存的速度
    send_joint_velocities(mapping, velocity_to_send);
}

bool JointVelocityController::send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities) {
    if (!hardware_manager_) return false;
    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) return false;

    try {
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const auto& motor_ids = hardware_manager_->get_motors_id(mapping);
        const auto& joint_names = hardware_manager_->get_joint_names(mapping);

        const double kp_velocity = 0.0;
        const double kd_velocity = 0.01;
        const double position = 0.0;

        auto gravity_torques = hardware_manager_->compute_gravity_torques(mapping);

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            double vel = joint_velocities[i] * 180.0 / M_PI;
            double effort = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;
            int violation_dir = hardware_manager_->get_joint_violation_direction(joint_names[i]);

            if (hardware_manager_->is_joint_emergency_stopped(joint_names[i]) &&
                ((violation_dir < 0 && vel < 0.0) || (violation_dir > 0 && vel > 0.0))) {
                hardware_driver->control_motor_in_mit_mode(interface, motor_ids[i], position, 0.0, effort, kp_velocity, kd_velocity);
            } else {
                hardware_driver->control_motor_in_mit_mode(interface, motor_ids[i], position, vel, effort, kp_velocity, kd_velocity);
            }
        }

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to send joint velocities: %s", mapping.c_str(), e.what());
        return false;
    }
}

bool JointVelocityController::send_velocity(const std::string& mapping, const std::vector<double>& velocity) {
    // 检查timer是否存在
    bool timer_exists = false;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        timer_exists = (it != mapping_states_.end() && it->second.control_timer);
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] DEBUG send_velocity: timer_exists=%s",
                mapping.c_str(), timer_exists ? "true" : "false");

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto it = mapping_states_.find(mapping);
        if (it == mapping_states_.end()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] send_velocity: mapping state not found!", mapping.c_str());
            return false;
        }

        const auto& joint_names = hardware_manager_->get_joint_names(mapping);
        if (velocity.size() != joint_names.size()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] send_velocity: size mismatch", mapping.c_str());
            return false;
        }

        // 只更新缓存，10ms 定时器会读取并发送
        it->second.last_cmd_velocity = velocity;
        it->second.last_cmd_time = steady_clock_.now();
        RCLCPP_INFO(node_->get_logger(), "[%s] send_velocity: updated last_cmd_time", mapping.c_str());
    }

    return true;
}


void JointVelocityController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;

    while (consumer_running_) {
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "JointVelocity")) continue;

        std::string mode = cmd.get_mode();
        std::string mapping = cmd.get_mapping();
        std::string cmd_id = cmd.get_command_id();

        RCLCPP_INFO(node_->get_logger(),
            "[JointVelocity IPC] Received command: mapping=%s, mode=%s, id=%s",
            mapping.c_str(), mode.c_str(), cmd_id.c_str());

        std::lock_guard<std::mutex> execution_lock(arm_controller::CommandQueueIPC::getMappingExecutionMutex(mapping));

        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        // ⭐ 只在第一次时调用 start()，检查timer是否存在而不是is_active()
        bool timer_exists = false;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            auto it = mapping_states_.find(mapping);
            timer_exists = (it != mapping_states_.end() && it->second.control_timer);
        }
        RCLCPP_INFO(node_->get_logger(), "[%s] DEBUG consumer: timer_exists=%s before start check",
                    mapping.c_str(), timer_exists ? "true" : "false");

        if (!timer_exists) {
            RCLCPP_INFO(node_->get_logger(), "[%s] IPC: calling start()", mapping.c_str());
            try {
                start(mapping);
                RCLCPP_INFO(node_->get_logger(), "[%s] IPC: start() succeeded", mapping.c_str());
            } catch (...) {
                RCLCPP_WARN(node_->get_logger(), "[%s] IPC: start() failed", mapping.c_str());
                continue;
            }
        }

        auto params = cmd.get_parameters();
        RCLCPP_INFO(node_->get_logger(), "[%s] IPC: calling send_velocity with %zu params", mapping.c_str(), params.size());
        bool success = send_velocity(mapping, params);
        RCLCPP_INFO(node_->get_logger(), "[%s] IPC: send_velocity returned %s", mapping.c_str(), success ? "true" : "false");

        if (state_mgr) {
            state_mgr->setExecutionState(success ? arm_controller::ipc::ExecutionState::SUCCESS
                                                : arm_controller::ipc::ExecutionState::FAILED);
            state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);

            arm_controller::ipc::ExecutorControllerState executor_state;
            strncpy(executor_state.current_mode, mode.c_str(), sizeof(executor_state.current_mode) - 1);
            executor_state.current_mode[sizeof(executor_state.current_mode) - 1] = '\0';
            executor_state.execution_state = (int)arm_controller::ipc::ExecutionState::IDLE;
            state_mgr->updateFromExecutor(executor_state);
        }

        arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
    }
}

JointVelocityController::~JointVelocityController() {
    RCLCPP_WARN(node_->get_logger(), "[JointVelocityController] ⚠️  DESTRUCTOR CALLED - instance destroyed, this=%px",
                static_cast<void*>(this));
    consumer_running_ = false;
    if (queue_consumer_ && queue_consumer_->joinable()) {
        queue_consumer_->join();
    }
}
