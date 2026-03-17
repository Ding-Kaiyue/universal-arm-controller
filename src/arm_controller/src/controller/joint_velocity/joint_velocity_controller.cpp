#include "joint_velocity_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <algorithm>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'JointVelocity', mapping: 'left_arm'}"
// ros2 topic pub --once /controller_api/joint_velocity_action/left_arm sensor_msgs/msg/JointState "{velocity: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"

JointVelocityController::JointVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<sensor_msgs::msg::JointState>("JointVelocity", node)
{
    // 获取HardwareManager实例
    hardware_manager_ = HardwareManager::getInstance();

    consumer_running_ = true;
    queue_consumer_ = std::make_unique<std::thread>(
        &JointVelocityController::command_queue_consumer_thread, this);
}

JointVelocityController::~JointVelocityController() {
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

void JointVelocityController::start(const std::string& mapping) {
    // 检查是否已初始化（仅用于防止重复创建线程）
    if (rt_threads_.count(mapping) > 0) {
        return;
    }

    auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
    if (state_mgr) {
        state_mgr->initializeCurrentMode("JointVelocity");
    }

    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] JointVelocity: not found in hardware configuration."
        );
    }

    VelocityControllerImpl::start(mapping);

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    RtState state;
    state.last_update = steady_clock_.now();

    auto joint_names = hardware_manager_->get_joint_names(mapping);
    state.target.resize(joint_names.size(), 0.0);

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

bool JointVelocityController::stop(const std::string& mapping) {
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

void JointVelocityController::velocity_callback(
    const std::string& mapping,
    const sensor_msgs::msg::JointState::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(rt_buffers_mutex_);  // 保护 rt_buffers_ map 的并发访问

    auto it = rt_buffers_.find(mapping);
    if (it == rt_buffers_.end()) return;

    RtCommand c;
    c.velocity = msg->velocity;
    c.stamp = steady_clock_.now();

    it->second->push(c);  // 队列的 push 本身是 lock-free 的
}

void JointVelocityController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;
    auto last_command_time = steady_clock_.now();  // 记录上一次处理完命令的时间
    constexpr std::chrono::milliseconds BATCH_TIMEOUT{100};  // 批处理超时：100ms 内无新命令则认为批次结束

    while (consumer_running_) {
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "JointVelocity", 10)) {
            // 超时未获得新命令，检查是否已超过批处理超时时间
            auto now = steady_clock_.now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time) >= BATCH_TIMEOUT) {
                arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
                last_command_time = now;  // 重置计时器
            }
            continue;
        }

        std::string mapping = cmd.get_mapping();

        // 第一步：检查当前mode，如果不是JointVelocity则请求完整的mode转换
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        if (state_mgr) {
            std::string current_mode = state_mgr->getCurrentMode();

            // 如果当前不是JointVelocity，需要请求完整的mode转换来停止旧mode
            if (!current_mode.empty() && current_mode != "JointVelocity") {
                if (hook_request_callback_) {
                    hook_request_callback_(mapping, "JointVelocity");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;  // 等待mode转换完成后重新进入
            }

            bool transition_ok = state_mgr->transitionToMode("JointVelocity");
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

        // 第二步：检查是否已经初始化（仅用于启动 RT 线程）
        bool need_init = (rt_threads_.count(mapping) == 0);

        // 第三步：启动控制器（仅在第一次）
        if (need_init) {
            start(mapping);
        }

        // 第四步：在锁内执行命令
        {
            std::lock_guard<std::mutex> lock(rt_buffers_mutex_);

            auto it = rt_buffers_.find(mapping);
            if (it != rt_buffers_.end()) {
                RtCommand c;
                c.velocity = cmd.get_parameters();
                c.stamp = steady_clock_.now();

                it->second->push(c);
            }
        }

        // 更新最后处理时间戳，用于判断批处理是否结束
        last_command_time = steady_clock_.now();
    }
}

void JointVelocityController::control_loop_rt(const std::string& mapping) {
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
        state.target = cmd.velocity;
        state.last_update = cmd.stamp;
    }

    auto now = steady_clock_.now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - state.last_update).count();
    std::vector<double> target = state.target;

    if (dt > 100.0) {
        std::fill(target.begin(), target.end(), 0.0);
    }

    send_joint_velocities(mapping, target);
}

bool JointVelocityController::send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities) {
    if (!hardware_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware manager not initialized");
        return false;
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    
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
    return send_joint_velocities(mapping, velocity);
}

