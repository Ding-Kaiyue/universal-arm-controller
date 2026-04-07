#include "command_streaming_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <algorithm>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'CommandStreaming', mapping: 'left_arm'}"
// ros2 topic pub --once /controller_api/command_streaming_action/left_arm sensor_msgs/msg/JointState "{velocity: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"

CommandStreamingController::CommandStreamingController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<sensor_msgs::msg::JointState>("CommandStreaming", node)
{
    // 获取HardwareManager实例
    hardware_manager_ = HardwareManager::getInstance();

    consumer_running_ = true;
    queue_consumer_ = std::make_unique<std::thread>(
        &CommandStreamingController::command_queue_consumer_thread, this);
}

CommandStreamingController::~CommandStreamingController() {
    consumer_running_ = false;

    for (auto& [mapping, rt_running] : rt_running_per_mapping_) {
        rt_running->store(false, std::memory_order_release);
    }

    arm_controller::CommandQueueIPC::getInstance().shutdown();
    if (queue_consumer_ && queue_consumer_->joinable()) {
        queue_consumer_->join();
    }

    for (auto& [m, th] : rt_threads_) {
        if (th.joinable()) th.join();
    }
}

CommandStreamingController::JointCommandData
CommandStreamingController::normalize_joint_command(
    const std::string& mapping,
    const JointCommandData& input) const {
    JointCommandData normalized = input;
    const size_t joint_count = hardware_manager_ ? hardware_manager_->get_joint_count(mapping) : 0U;

    auto resize_with_default = [joint_count](std::vector<double>& values, double fill_value) {
        values.resize(joint_count, fill_value);
    };

    if (joint_count == 0) {
        normalized.positions.clear();
        normalized.velocities.clear();
        normalized.efforts.clear();
        return normalized;
    }

    if (normalized.positions.empty() && hardware_manager_) {
        normalized.positions = hardware_manager_->get_current_joint_positions_lockfree(mapping);
    }
    resize_with_default(normalized.positions, 0.0);
    resize_with_default(normalized.velocities, 0.0);

    if (normalized.efforts.empty() && hardware_manager_) {
        normalized.efforts = hardware_manager_->compute_gravity_torques(mapping, normalized.positions);
    }
    resize_with_default(normalized.efforts, 0.0);

    return normalized;
}

CommandStreamingController::JointCommandData
CommandStreamingController::decode_packed_command(
    const std::string& mapping,
    const std::vector<double>& packed_command) const {
    JointCommandData decoded;
    const size_t joint_count = hardware_manager_ ? hardware_manager_->get_joint_count(mapping) : 0U;

    if (joint_count == 0) {
        return decoded;
    }

    if (packed_command.size() >= joint_count * 3) {
        decoded.positions.assign(
            packed_command.begin(),
            packed_command.begin() + joint_count);
        decoded.velocities.assign(
            packed_command.begin() + joint_count,
            packed_command.begin() + joint_count * 2);
        decoded.efforts.assign(
            packed_command.begin() + joint_count * 2,
            packed_command.begin() + joint_count * 3);
    } else {
        decoded.positions = packed_command;
    }

    return normalize_joint_command(mapping, decoded);
}

void CommandStreamingController::start(const std::string& mapping) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

    // 幂等启动：避免与 ControllerManager 并发调用导致重复创建资源
    if (rt_threads_.count(mapping) > 0 || rt_running_per_mapping_.count(mapping) > 0) {
        return;
    }

    auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
    if (state_mgr) {
        state_mgr->initializeCurrentMode("CommandStreaming");
    }

    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] CommandStreaming: not found in hardware configuration."
        );
    }

    VelocityControllerImpl::start(mapping);

    // 在激活时创建话题订阅（如果还没创建的话）
    bool need_subscription = false;
    {
        std::lock_guard<std::mutex> sub_lock(subscriptions_mutex_);
        need_subscription = (subscriptions_.find(mapping) == subscriptions_.end());
    }
    if (need_subscription) {
        init_subscriptions(mapping);
    }

    RtState state;
    state.last_update = steady_clock_.now();

    auto joint_names = hardware_manager_->get_joint_names(mapping);

    rt_states_[mapping] = state;

    // create buffer
    rt_buffers_[mapping] = std::make_unique<SPSCQueue<RtCommand, 128>>();

    // 创建per-mapping的RT线程运行标志（避免stop()关闭所有线程）
    auto rt_running = std::make_shared<std::atomic<bool>>(true);
    rt_running_per_mapping_[mapping] = rt_running;

    // create RT thread
    rt_threads_[mapping] = std::thread([this, mapping, rt_running]() {
        auto next = std::chrono::steady_clock::now();

        while (rt_running->load(std::memory_order_acquire)) {
            next += std::chrono::milliseconds(5); // 200Hz

            control_loop_rt(mapping);

            std::this_thread::sleep_until(next);
        }
    });
}

bool CommandStreamingController::stop(const std::string& mapping) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

    VelocityControllerImpl::stop(mapping);

    // 只关闭这个mapping的RT线程（使用per-mapping标志）
    auto it_running = rt_running_per_mapping_.find(mapping);
    if (it_running != rt_running_per_mapping_.end()) {
        it_running->second->store(false, std::memory_order_release);
        rt_running_per_mapping_.erase(it_running);
    }

    auto it = rt_threads_.find(mapping);
    if (it != rt_threads_.end()) {
        if (it->second.joinable()) {
            it->second.join();
        }
        rt_threads_.erase(it);
    }

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    // 清除RT状态和缓冲区（需要加锁保护，因为velocity_callback可能并发访问）
    {
        std::lock_guard<std::mutex> lock(rt_buffers_mutex_);
        rt_buffers_.erase(mapping);
        rt_states_.erase(mapping);
    }

    return true;
}

void CommandStreamingController::velocity_callback(
    const std::string& mapping,
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(rt_buffers_mutex_);  // 保护 rt_buffers_ map 的并发访问

    auto it = rt_buffers_.find(mapping);
    if (it == rt_buffers_.end()) return;

    RtCommand c;
    c.data.positions = msg->position;
    c.data.velocities = msg->velocity;
    c.data.efforts = msg->effort;
    c.data = normalize_joint_command(mapping, c.data);

    c.stamp = steady_clock_.now();

    it->second->push(c);  // 队列的 push 本身是 lock-free 的
}

void CommandStreamingController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;
    auto last_command_time = steady_clock_.now();  // 记录上一次处理完命令的时间
    constexpr std::chrono::milliseconds BATCH_TIMEOUT{100};  // 批处理超时：100ms 内无新命令则认为批次结束

    while (consumer_running_) {
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "CommandStreaming", 10)) {
            // 超时未获得新命令，检查是否已超过批处理超时时间
            auto now = steady_clock_.now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time) >= BATCH_TIMEOUT) {
                arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
                last_command_time = now;  // 重置计时器
            }
            continue;
        }

        std::string mapping = cmd.get_mapping();

        // 第一步：检查当前mode，如果不是CommandStreaming则请求完整的mode转换
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        if (state_mgr) {
            std::string current_mode = state_mgr->getCurrentMode();

            // 如果当前不是CommandStreaming，需要请求完整的mode转换来停止旧mode
            if (!current_mode.empty() && current_mode != "CommandStreaming") {
                if (hook_request_callback_) {
                    hook_request_callback_(mapping, "CommandStreaming");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;  // 等待mode转换完成后重新进入
            }

            bool transition_ok = state_mgr->transitionToMode("CommandStreaming");
            if (!transition_ok) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  Mode transition failed", mapping.c_str());
                continue;
            }

            // 如果仍在 hook 状态，请求启动 HoldState
            if (state_mgr->isInHookState()) {
                std::string target_mode = state_mgr->getTargetMode();
                if (hook_request_callback_) {
                    hook_request_callback_(mapping, target_mode);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }

        // 第二步：确保控制器已启动（start() 已做并发保护和幂等）
        start(mapping);

        // 第四步：在锁内执行命令
        {
            std::lock_guard<std::mutex> lock(rt_buffers_mutex_);

            auto it = rt_buffers_.find(mapping);
            if (it != rt_buffers_.end()) {
                RtCommand c;
                c.data = decode_packed_command(mapping, cmd.get_parameters());
                c.stamp = steady_clock_.now();

                it->second->push(c);
            }
        }

        // 更新最后处理时间戳，用于判断批处理是否结束
        last_command_time = steady_clock_.now();
    }
}

void CommandStreamingController::control_loop_rt(const std::string& mapping) {
    if (!is_active(mapping)) {
        return;
    }

    auto it_state = rt_states_.find(mapping);
    auto it_buf = rt_buffers_.find(mapping);
    if (it_state == rt_states_.end() || it_buf == rt_buffers_.end()) {
        return;
    }

    auto& state = it_state->second;
    auto& buffer = it_buf->second;

    RtCommand cmd;
    while (buffer->pop(cmd)) {
        state.target = cmd.data;
        state.last_update = cmd.stamp;
    }

    auto now = steady_clock_.now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - state.last_update).count();
    JointCommandData target = state.target;

    if (dt > 100.0) {
        std::fill(target.velocities.begin(), target.velocities.end(), 0.0);
        std::fill(target.efforts.begin(), target.efforts.end(), 0.0);
    }

    send_joint_status(mapping, target.positions, target.velocities, target.efforts);
}

bool CommandStreamingController::send_joint_status(
    const std::string& mapping,
    const std::vector<double>& joint_positions,
    const std::vector<double>& joint_velocities,
    const std::vector<double>& joint_efforts) {
    if (!hardware_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware manager not initialized");
        return false;
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    
    try {
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);
        const std::vector<std::string>& joint_names = hardware_manager_->get_joint_names(mapping);

        JointCommandData target;
        target.positions = joint_positions;
        target.velocities = joint_velocities;
        target.efforts = joint_efforts;
        target = normalize_joint_command(mapping, target);

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            const auto& joint_name = joint_names[i];
            uint32_t motor_id = motor_ids[i];
            double pos = target.positions[i] * 180.0 / M_PI;
            double vel = target.velocities[i] * 180.0 / M_PI;
            double effort = target.efforts[i];
            const double kp_velocity = 0.05;
            const double kd_velocity = 0.005;

            int violation_dir = hardware_manager_->get_joint_violation_direction(joint_name);
            if (hardware_manager_->is_joint_emergency_stopped(joint_name) &&
                ((violation_dir < 0 && vel < 0.0) || (violation_dir > 0 && vel > 0.0))) {
                // 不允许继续违规，发送零速度
                hardware_driver->control_motor_in_mit_mode(interface, motor_id, pos, 0.0, effort, kp_velocity, kd_velocity);
                auto clock = node_->get_clock();
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *clock,
                    2000,
                    "[%s] Joint '%s' emergency stopped (dir=%d), unsafe velocity %.3f -> skipping.",
                    mapping.c_str(), joint_name.c_str(), violation_dir, vel);
            } else {
                hardware_driver->control_motor_in_mit_mode(interface, motor_id, pos, vel, effort, kp_velocity, kd_velocity);
            }
        }

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to send joint velocities: %s", mapping.c_str(), e.what());
        return false;
    }
}

bool CommandStreamingController::send_velocity(const std::string& mapping, const std::vector<double>& velocity) {
    auto target = decode_packed_command(mapping, velocity);
    return send_joint_status(mapping, target.positions, target.velocities, target.efforts);
}
