#include "cartesian_velocity_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/ipc_context.hpp"
#include <stdexcept>
#include <algorithm>
#include <set>
#include <sstream>
#include <iomanip>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'CartesianVelocity', mapping: 'single_arm'}"
// ros2 topic pub /controller_api/cartesian_velocity_action/single_arm geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"

CartesianVelocityController::CartesianVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerImpl<geometry_msgs::msg::TwistStamped>("CartesianVelocity", node)
{
    // 获取HardwareManager实例
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化TF2缓冲和监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    consumer_running_ = true;
    queue_consumer_ = std::make_unique<std::thread>(
        &CartesianVelocityController::command_queue_consumer_thread, this);
}

CartesianVelocityController::~CartesianVelocityController() {
    // Step 1: 停止所有已激活的 mapping（调用 stop() 会完整清理线程和资源）
    std::vector<std::string> mappings_to_stop;
    {
        // 从 rt_running_per_mapping_ 中收集所有已初始化的 mapping
        for (const auto& [mapping, _] : rt_running_per_mapping_) {
            mappings_to_stop.push_back(mapping);
        }
    }

    for (const auto& mapping : mappings_to_stop) {
        stop(mapping);
    }

    // Step 2: 停止 IPC 消费线程（此时 RT/计算线程已全部退出）
    consumer_running_ = false;
    arm_controller::CommandQueueIPC::getInstance().shutdown();
    if (queue_consumer_ && queue_consumer_->joinable()) {
        queue_consumer_->join();
    }
}


void CartesianVelocityController::start(const std::string& mapping) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

    // 幂等启动：避免与 ControllerManager 并发调用导致重复创建资源
    if (rt_threads_.count(mapping) > 0 ||
        rt_running_per_mapping_.count(mapping) > 0 ||
        computation_threads_.count(mapping) > 0 ||
        computation_running_per_mapping_.count(mapping) > 0) {
        return;
    }

    auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);
    if (state_mgr) {
        state_mgr->initializeCurrentMode("CartesianVelocity");
    }

    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] CartesianVelocity: not found in hardware configuration."
        );
    }

    VelocityControllerImpl::start(mapping);

    initialize_moveit_adapter(mapping);
    initialize_jacobian_provider(mapping);

    std::string base_frame = hardware_manager_->get_frame_id(mapping);
    if (base_frame.empty()) {
        base_frame = "base_link";  // 默认降级到 "base_link"
    }
    mapping_base_frames_[mapping] = base_frame;
    RCLCPP_INFO(node_->get_logger(), "[%s] CartesianVelocity base frame: %s",
                mapping.c_str(), base_frame.c_str());

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
    state.target = geometry_msgs::msg::TwistStamped();
    state.target.header.frame_id = base_frame;

    rt_states_[mapping] = state;

    // 初始化RT状态的mutex（保护RT线程和计算线程的并发访问）
    auto rt_state_mutex = std::make_unique<std::mutex>();
    rt_states_mutexes_[mapping] = std::move(rt_state_mutex);

    // create buffer
    rt_buffers_[mapping] = std::make_unique<SPSCQueue<TwistCommand, 128>>();

    // 创建per-mapping的RT线程运行标志（避免stop()关闭所有线程）
    auto rt_running = std::make_shared<std::atomic<bool>>(true);
    rt_running_per_mapping_[mapping] = rt_running;

    // create RT thread (5ms control loop)
    rt_threads_[mapping] = std::thread([this, mapping, rt_running]() {
        auto next = std::chrono::steady_clock::now();

        while (rt_running->load(std::memory_order_acquire)) {
            next += std::chrono::milliseconds(5);  // 5ms

            control_loop_rt(mapping);

            auto now = std::chrono::steady_clock::now();
            if (now < next) {
                std::this_thread::sleep_for(next - now);
            }
        }
    });

    // 创建计算线程 - 处理所有耗时操作（Jacobian、SVD、QP求解等）
    auto computation_running = std::make_shared<std::atomic<bool>>(true);
    computation_running_per_mapping_[mapping] = computation_running;

    // 初始化计算结果结构体
    {
        auto result_with_mtx = std::make_unique<ComputationResultWithMutex>();
        result_with_mtx->result.valid = false;
        const int dof = hardware_manager_->get_joint_names(mapping).size();
        result_with_mtx->result.qd = Eigen::VectorXd::Zero(dof);
        result_with_mtx->result.qd_last = Eigen::VectorXd::Zero(dof);  // 初始化缓存结果为零速度
        result_with_mtx->result.timestamp = steady_clock_.now();
        computation_results_[mapping] = std::move(result_with_mtx);
    }

    computation_threads_[mapping] = std::thread(
        &CartesianVelocityController::cartesian_computation_thread,
        this,
        mapping);
}


bool CartesianVelocityController::stop(const std::string& mapping) {
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

    // 关闭计算线程
    auto it_comp_running = computation_running_per_mapping_.find(mapping);
    if (it_comp_running != computation_running_per_mapping_.end()) {
        it_comp_running->second->store(false, std::memory_order_release);
        computation_running_per_mapping_.erase(it_comp_running);
    }

    auto it_comp = computation_threads_.find(mapping);
    if (it_comp != computation_threads_.end()) {
        if (it_comp->second.joinable()) {
            it_comp->second.join();
        }
        computation_threads_.erase(it_comp);
    }
    computation_results_.erase(mapping);

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);
    moveit_adapters_.erase(mapping);
    jacobian_providers_.erase(mapping);
    mapping_base_frames_.erase(mapping);

    // 清除状态、锁、buffer等所有与这个 mapping 相关的资源（需要加锁保护）
    {
        std::lock_guard<std::mutex> lock(rt_buffers_mutex_);
        rt_buffers_.erase(mapping);
    }

    {
        auto it_mtx = rt_states_mutexes_.find(mapping);
        if (it_mtx != rt_states_mutexes_.end()) {
            {
                std::lock_guard<std::mutex> lock(*it_mtx->second);
                rt_states_.erase(mapping);
            }
            rt_states_mutexes_.erase(mapping);
        }
    }

    return true;
}

void CartesianVelocityController::initialize_moveit_adapter(const std::string& mapping) {
    try {
        if (hardware_manager_->get_motors_id(mapping).empty()) {
            return;
        }

        if (moveit_adapters_.find(mapping) != moveit_adapters_.end()) {
            RCLCPP_INFO(node_->get_logger(), "[%s] MoveIt adapter already initialized", mapping.c_str());
            return;
        }

        std::string planning_group = hardware_manager_->get_planning_group(mapping);
        if (planning_group.empty()) {
            RCLCPP_WARN(node_->get_logger(), "[%s] ❎ No planning group configured", mapping.c_str());
            return;
        }

        auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
            node_, planning_group);

        if (!moveit_adapter) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Failed to create MoveItAdapter", mapping.c_str());
            return;
        }

        moveit_adapters_[mapping] = moveit_adapter;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Exception: %s", mapping.c_str(), e.what());
    }
}

void CartesianVelocityController::initialize_jacobian_provider(const std::string& mapping) {
    try {
        if (jacobian_providers_.find(mapping) != jacobian_providers_.end()) {
            return;
        }

        auto it_moveit = moveit_adapters_.find(mapping);
        if (it_moveit == moveit_adapters_.end() || !it_moveit->second) {
            RCLCPP_WARN(node_->get_logger(),
                        "[%s] ❎ MoveIt adapter not initialized, skip JacobianProvider init",
                        mapping.c_str());
            return;
        }

        auto jacobian_provider =
            std::make_shared<arm_controller::kinematics::MoveItJacobianProvider>(node_, it_moveit->second);
        if (!jacobian_provider->initialize()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Failed to initialize MoveItJacobianProvider",
                         mapping.c_str());
            return;
        }

        jacobian_providers_[mapping] = jacobian_provider;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ Jacobian provider init exception: %s",
                     mapping.c_str(), e.what());
    }
}

void CartesianVelocityController::velocity_callback(
    const std::string& mapping,
    const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(rt_buffers_mutex_);

    auto it = rt_buffers_.find(mapping);
    if (it == rt_buffers_.end()) return;

    // 使用 mapping 对应的基座坐标系
    auto it_frame = mapping_base_frames_.find(mapping);
    std::string base_frame = (it_frame != mapping_base_frames_.end()) ?
        it_frame->second : "base_link";

    TwistCommand c;
    c.twist = *msg;
    c.twist.header.frame_id = base_frame;
    c.stamp = steady_clock_.now();

    it->second->push(c);
}

void CartesianVelocityController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;
    auto last_command_time = steady_clock_.now();
    constexpr std::chrono::milliseconds BATCH_TIMEOUT{100};  // 批处理超时：100ms 内无新命令则认为批次结束

    while (consumer_running_) {
        if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "CartesianVelocity", 10)) {
            auto now = steady_clock_.now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time) >= BATCH_TIMEOUT) {
                arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
                last_command_time = now;  // 重置计时器
            }
            continue;
        }

        std::string mapping = cmd.get_mapping();

        // 第一步：检查当前mode，如果不是CartesianVelocity则请求完整的mode转换
        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(mapping);

        if (state_mgr) {
            std::string current_mode = state_mgr->getCurrentMode();

            // 如果当前不是CartesianVelocity，需要请求完整的mode转换来停止旧mode
            if (!current_mode.empty() && current_mode != "CartesianVelocity") {
                if (hook_request_callback_) {
                    hook_request_callback_(mapping, "CartesianVelocity");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;  // 等待mode转换完成后重新进入
            }

            bool transition_ok = state_mgr->transitionToMode("CartesianVelocity");
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

        // 第三步：在锁内执行命令
        {
            std::lock_guard<std::mutex> lock(rt_buffers_mutex_);

            auto it = rt_buffers_.find(mapping);
            if (it != rt_buffers_.end()) {
                TwistCommand c;
                auto params = cmd.get_parameters();

                // 使用 mapping 对应的基座坐标系
                auto it_frame = mapping_base_frames_.find(mapping);
                std::string base_frame = (it_frame != mapping_base_frames_.end()) ?
                    it_frame->second : "base_link";

                // 构造 TwistStamped 命令
                c.twist.header.frame_id = base_frame;
                c.twist.header.stamp = node_->now();
                if (params.size() == 6) {
                    c.twist.twist.linear.x = params[0];
                    c.twist.twist.linear.y = params[1];
                    c.twist.twist.linear.z = params[2];
                    c.twist.twist.angular.x = params[3];
                    c.twist.twist.angular.y = params[4];
                    c.twist.twist.angular.z = params[5];
                }
                c.stamp = steady_clock_.now();

                it->second->push(c);
            }
        }

        last_command_time = steady_clock_.now();
    }
}

void CartesianVelocityController::control_loop_rt(const std::string& mapping) {
    if (!is_active(mapping)) {
        return;
    }

    // 检查 RT 线程是否被要求停止（支持可中断退出）
    auto it_running = rt_running_per_mapping_.find(mapping);
    if (it_running == rt_running_per_mapping_.end() ||
        !it_running->second->load(std::memory_order_acquire)) {
        return;
    }

    auto it_state = rt_states_.find(mapping);
    if (it_state == rt_states_.end()) {
        return;
    }

    auto& state = it_state->second;

    // 第一步：在锁内从IPC命令队列pop最新命令
    std::vector<TwistCommand> new_commands;
    {
        std::lock_guard<std::mutex> lock(rt_buffers_mutex_);
        auto it_buf = rt_buffers_.find(mapping);
        if (it_buf == rt_buffers_.end()) {
            return;
        }

        TwistCommand cmd;
        while (it_buf->second->pop(cmd)) {
            new_commands.push_back(cmd);
        }
    }  // ← 释放 rt_buffers_mutex_

    // 现在在锁外处理命令（避免锁嵌套导致死锁）
    if (!new_commands.empty()) {
        auto it_mtx = rt_states_mutexes_.find(mapping);
        if (it_mtx != rt_states_mutexes_.end() && it_mtx->second) {
            std::lock_guard<std::mutex> lock2(*it_mtx->second);
            // 取最后一条命令（最新的）
            const auto& cmd = new_commands.back();
            state.target = cmd.twist;
            state.last_update = cmd.stamp;
            state.first_command_received = true;
        }
    }

    auto now = steady_clock_.now();

    // 第二步：检查是否超时（分离锁依赖，避免循环死锁）只在状态锁内检查超时标志和重置状态，不进行任何清理操作
    bool should_reset_on_timeout = false;
    {
        auto it_mtx = rt_states_mutexes_.find(mapping);
        if (it_mtx != rt_states_mutexes_.end() && it_mtx->second) {
            std::lock_guard<std::mutex> lock(*it_mtx->second);

            // 只有在已收到过命令的情况下才进行超时检测
            if (state.first_command_received) {
                auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - state.last_update).count();
                if (dt_ms > 100) {
                    // 标记超时，准备在锁外执行清理
                    should_reset_on_timeout = true;
                    state.first_command_received = false;
                }
            }
        }
    }  // ← 释放 rt_states_mutexes_

    // 第三步：在锁外执行超时清理（避免嵌套锁导致的循环依赖）
    if (should_reset_on_timeout) {
        // 清空 rt_buffers 中的旧命令
        {
            std::lock_guard<std::mutex> lock_buf(rt_buffers_mutex_);
            auto it_buf = rt_buffers_.find(mapping);
            if (it_buf != rt_buffers_.end()) {
                TwistCommand dummy;
                while (it_buf->second->pop(dummy)) { /*继续 pop 直到队列为空*/ }
            }
        }

        // 清空计算线程的缓存
        {
            auto it_result = computation_results_.find(mapping);
            if (it_result != computation_results_.end() && it_result->second) {
                std::lock_guard<std::mutex> lock2(it_result->second->mtx);
                const int dof = it_result->second->result.qd_last.size();
                it_result->second->result.valid = false;
                it_result->second->result.qd_last = Eigen::VectorXd::Zero(dof);
            }
        }

        // 发送零速度停止
        send_joint_velocities(mapping, std::vector<double>(hardware_manager_->get_joint_names(mapping).size(), 0.0));
        return;
    }

    // 从受保护的计算结果中读取关节速度
    {
        auto it_result = computation_results_.find(mapping);
        if (it_result != computation_results_.end() && it_result->second) {
            std::lock_guard<std::mutex> lock(it_result->second->mtx);
            const auto& result = it_result->second->result;

            // 优先使用 qd，如果无效则使用上一次的有效结果 qd_last
            const Eigen::VectorXd& qd_to_send = result.valid ? result.qd : result.qd_last;

            // ===== 发送速度（当前结果或缓存的上一次有效结果） =====
            std::vector<double> velocities(qd_to_send.data(), qd_to_send.data() + qd_to_send.size());
            send_joint_velocities(mapping, velocities);
        }
    }
}


bool CartesianVelocityController::send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities) {
    if (!hardware_manager_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware manager not initialized");
        return false;
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();

    try {
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);
        const std::vector<std::string>& joint_names = hardware_manager_->get_joint_names(mapping);

        // 获取当前关节位置对应的重力矩
        std::vector<double> gravity_torques = hardware_manager_->compute_gravity_torques(mapping);

        std::array<double, 6> batch_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, 6> batch_velocities = {};
        std::array<double, 6> batch_efforts = {};
        // MIT模式速度控制参数
        std::array<double, 6> batch_kps = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, 6> batch_kds = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

        for (size_t i = 0; i < motor_ids.size(); ++i) {
            const auto& joint_name = joint_names[i];
            double vel = joint_velocities[i] * 180.0 / M_PI;  // 转为度/秒

            // 检查紧急停止
            int violation_dir = hardware_manager_->get_joint_violation_direction(joint_name);
            bool is_emergency_stopped = hardware_manager_->is_joint_emergency_stopped(joint_name);

            if (is_emergency_stopped && ((violation_dir < 0 && vel < 0.0) || (violation_dir > 0 && vel > 0.0))) {
                // 紧急停止：设置零速度
                batch_velocities[i] = 0.0;
                auto clock = node_->get_clock();
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(),
                    *clock,
                    2000,
                    "[%s] Joint '%s' (index %zu) emergency stopped (dir=%d), unsafe velocity %.3f -> forced to 0",
                    mapping.c_str(), joint_name.c_str(), i, violation_dir, vel);
            } else {
                batch_velocities[i] = vel;
            }

            // MIT 模式参数
            batch_efforts[i] = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;
        }

        // 一次性批量发送所有电机命令（完全同步）
        bool success = hardware_driver->send_realtime_mit_command(
            interface,
            batch_positions,
            batch_velocities,
            batch_efforts,
            batch_kps,
            batch_kds
        );

        return success;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to send joint velocities: %s", mapping.c_str(), e.what());
        return false;
    }
}

bool CartesianVelocityController::send_velocity(const std::string& mapping, const std::vector<double>& velocity) {
    // 注意：velocity 参数是 Cartesian 速度 [vx, vy, vz, wx, wy, wz]

    if (velocity.size() < 6) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] ❎ Invalid Cartesian velocity dimensions: %zu (expected 6)",
            mapping.c_str(), velocity.size());
        return false;
    }

    // 使用 mapping 对应的基座坐标系
    auto it_frame = mapping_base_frames_.find(mapping);
    std::string base_frame = (it_frame != mapping_base_frames_.end()) ?
        it_frame->second : "base_link";

    // 构造一个 TwistStamped 消息来存储 Cartesian 速度命令
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = base_frame;
    cmd.header.stamp = node_->now();
    cmd.twist.linear.x = velocity[0];
    cmd.twist.linear.y = velocity[1];
    cmd.twist.linear.z = velocity[2];
    cmd.twist.angular.x = velocity[3];
    cmd.twist.angular.y = velocity[4];
    cmd.twist.angular.z = velocity[5];

    // 存储命令状态，让 control_loop_rt() 处理 IK 转换
    {
        std::lock_guard<std::mutex> lock(rt_buffers_mutex_);

        auto it = rt_buffers_.find(mapping);
        if (it != rt_buffers_.end()) {
            TwistCommand c;
            c.twist = cmd;
            c.stamp = steady_clock_.now();

            it->second->push(c);
        }
    }

    return true;
}

void CartesianVelocityController::cartesian_computation_thread(const std::string& mapping) {
    auto it_running = computation_running_per_mapping_.find(mapping);
    if (it_running == computation_running_per_mapping_.end()) {
        return;
    }
    auto computation_running = it_running->second;

    // ===== Lambda 辅助函数：标记结果无效 =====
    auto mark_invalid = [this, &mapping]() {
        auto it_result = computation_results_.find(mapping);
        if (it_result != computation_results_.end() && it_result->second) {
            std::lock_guard<std::mutex> lock(it_result->second->mtx);
            it_result->second->result.valid = false;
        }
    };

    // ===== Lambda 辅助函数：睡眠到下个周期 =====
    auto sleep_to_next_cycle = [](const auto& cycle_start) {
        auto elapsed = std::chrono::steady_clock::now() - cycle_start;
        auto remaining = std::chrono::milliseconds(10) - elapsed;
        if (remaining.count() > 0) {
            std::this_thread::sleep_for(remaining);
        }
    };

    while (computation_running->load(std::memory_order_acquire)) {
        auto cycle_start = std::chrono::steady_clock::now();

        if (!is_active(mapping)) {
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        // ===== 获取状态和目标命令 =====
        auto it_state = rt_states_.find(mapping);
        if (it_state == rt_states_.end()) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        // 安全复制state
        RtState state_copy;
        {
            auto it_mtx = rt_states_mutexes_.find(mapping);
            if (it_mtx != rt_states_mutexes_.end() && it_mtx->second) {
                std::lock_guard<std::mutex> lock(*it_mtx->second);
                state_copy = it_state->second;
            } else {
                state_copy = it_state->second;
            }
        }

        auto& target_cmd = state_copy.target;

        // ===== 获取关节位置和MoveIt适配器 =====
        // 使用 lock-free 版本，避免竞争 hardware_manager 的 joint_state_mutex_
        // 这确保硬件回调线程永远不会被计算线程阻塞
        auto joint_positions = hardware_manager_->get_current_joint_positions_lockfree(mapping);
        auto joint_names = hardware_manager_->get_joint_names(mapping);

        if (joint_positions.empty() || joint_names.empty()) {
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        auto it_jacobian = jacobian_providers_.find(mapping);
        if (it_jacobian == jacobian_providers_.end() || !it_jacobian->second) {
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        // ===== Jacobian 计算 =====
        // 计算前再次检查，避免在 computeJacobian（可能阻塞）之前已被要求退出
        if (!computation_running->load(std::memory_order_acquire)) break;

        Eigen::Map<const Eigen::VectorXd> q_eig(joint_positions.data(), static_cast<int>(joint_positions.size()));
        Eigen::MatrixXd J = it_jacobian->second->computeJacobian(
            q_eig, "", Eigen::Vector3d::Zero());
        if (!computation_running->load(std::memory_order_acquire)) break;
        if (J.rows() == 0 || J.cols() == 0 || J.hasNaN()) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        // ===== 提取笛卡尔速度 =====
        Eigen::Vector3d v_linear(target_cmd.twist.linear.x,
                                  target_cmd.twist.linear.y,
                                  target_cmd.twist.linear.z);
        Eigen::Vector3d v_angular(target_cmd.twist.angular.x,
                                   target_cmd.twist.angular.y,
                                   target_cmd.twist.angular.z);

        // ===== TF2 坐标系变换 =====
        auto it_frame = mapping_base_frames_.find(mapping);
        std::string base_frame = (it_frame != mapping_base_frames_.end()) ?
            it_frame->second : "base_link";

        std::string user_frame = target_cmd.header.frame_id.empty() ?
            base_frame : target_cmd.header.frame_id;

        if (user_frame != base_frame) {
            try {
                auto tf = tf_buffer_->lookupTransform(
                    base_frame, user_frame,
                    tf2::TimePointZero,
                    std::chrono::milliseconds(50));

                Eigen::Quaterniond q(tf.transform.rotation.w,
                                      tf.transform.rotation.x,
                                      tf.transform.rotation.y,
                                      tf.transform.rotation.z);
                Eigen::Matrix3d R = q.toRotationMatrix();
                v_linear = R * v_linear;
                v_angular = R * v_angular;

            } catch (const std::exception& e) {
                mark_invalid();
                sleep_to_next_cycle(cycle_start);
                continue;
            }
        }

        // ===== 构造 6D 任务速度向量 =====
        Eigen::MatrixXd J_task = J;
        Eigen::VectorXd v_task(6);
        v_task << v_linear(0), v_linear(1), v_linear(2),
                  v_angular(0), v_angular(1), v_angular(2);

        if (v_task.norm() < 1e-8) {
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        // ===== 奇异性检测和缩速 =====
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_task);
        double sigma_min = svd.singularValues().minCoeff();

        double scale = 1.0;
        if (sigma_min < 0.05) {
            scale = (sigma_min <= 0.01) ? 0.0 : (sigma_min - 0.01) / 0.04;
        }

        v_task *= scale;

        // ===== 获取关节限制 =====
        int dof = J.cols();
        Eigen::VectorXd q_current = Eigen::Map<Eigen::VectorXd>(
            joint_positions.data(), joint_positions.size());

        Eigen::VectorXd qd_max(dof), q_min_pos(dof), q_max_pos(dof);
        for (int i = 0; i < dof; ++i) {
            JointLimits limits;
            hardware_manager_->get_joint_limits(joint_names[i], limits);
            qd_max(i) = limits.has_velocity_limits ? limits.max_velocity : 1.0;
            q_min_pos(i) = limits.min_position;
            q_max_pos(i) = limits.max_position;
        }

        // ===== QP 求解 =====
        Eigen::VectorXd qd(dof);
        bool ok = solver_.solve(J_task, v_task, q_current, q_min_pos, q_max_pos,
                                qd_max, qd, node_->get_logger());

        if (!ok) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        // ===== 方向一致性验证 =====
        Eigen::VectorXd v_reconstructed = J_task * qd;

        if (v_reconstructed.norm() < 1e-6 ||
            v_reconstructed.normalized().dot(v_task.normalized()) < 0.98) {
            mark_invalid();
            sleep_to_next_cycle(cycle_start);
            continue;
        }

        // ===== 存储结果 =====
        {
            auto it_result = computation_results_.find(mapping);
            if (it_result != computation_results_.end() && it_result->second) {
                std::lock_guard<std::mutex> lock(it_result->second->mtx);
                it_result->second->result.qd = qd;
                it_result->second->result.qd_last = qd;
                it_result->second->result.valid = true;
                it_result->second->result.timestamp = steady_clock_.now();
            }
        }

        sleep_to_next_cycle(cycle_start);
    }
}
