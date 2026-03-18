#include "trajectory_record_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <numeric>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'TrajectoryRecord', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/trajectory_record_action/single_arm std_msgs/msg/String 'data: "little_ding"'
// ros2 topic pub --once /controller_api/trajectory_record_control/single_arm std_msgs/msg/String 'data: "complete"'

TrajectoryRecordController::TrajectoryRecordController(const rclcpp::Node::SharedPtr & node)
    : TeachControllerBase("TrajectoryRecord", node)
{
    // 获取硬件管理器实例
    hardware_manager_ = HardwareManager::getInstance();

    // ✅ 初始化轨迹平滑处理器
    trajectory_smoother_ = std::make_unique<TrajectorySmoother>(node);

    // 参数由 TeachControllerBase::init_subscriptions() 自动处理
    // input_topic0: 文件名输入话题（teach_callback）
    // input_topic1: 录制控制话题（on_teaching_control）

    /* ---------- trajectory directory ---------- */
    try {
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("arm_controller");

        std::filesystem::path workspace_root =
            std::filesystem::path(pkg_dir).parent_path().parent_path().parent_path();

        record_dir_ = (workspace_root / "trajectories").string();

        std::filesystem::create_directories(record_dir_);
    } catch (const std::exception& e) {
        record_dir_ = "/tmp/arm_recording_trajectories";
        std::filesystem::create_directories(record_dir_);
        fprintf(stderr, "⚠️  Fallback to: %s\n", record_dir_.c_str());
    }

    // 启动IPC命令队列消费线程
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&TeachControllerBase::command_queue_consumer_thread, this);
    }

    RCLCPP_INFO(node_->get_logger(), "TrajectoryRecordController initialized. Output dir: %s", record_dir_.c_str());
}


void TrajectoryRecordController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] TrajectoryRecord: not found in hardware configuration."
        );
    }

    // 调用基类 start() 设置 per-mapping 的 active_mappings_[mapping] = true
    ModeControllerBase::start(mapping);

    // 启用示教模式 - 防止安全限位检查触发急停
    enable_teaching_mode();

    RCLCPP_INFO(node_->get_logger(), "[%s] TrajectoryRecordController activated",
                mapping.c_str());

    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return;
    }

    // ✅ 启用高频反馈（示教模式需要高频反馈）
    hardware_driver->force_high_freq_feedback();

    // ✅ 为该 mapping 启动独立的重力补偿线程
    start_gravity_compensation_thread(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Gravity compensation thread started", mapping.c_str());
}

bool TrajectoryRecordController::stop(const std::string& mapping) {
    // 调用基类 stop() 设置 per-mapping 的 active_mappings_[mapping] = false
    ModeControllerBase::stop(mapping);

    // ✅ 取消强制高频反馈
    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (hardware_driver) {
        hardware_driver->cancel_force_high_freq();
    }

    // ✅ 停止该 mapping 的重力补偿线程
    stop_gravity_compensation_thread(mapping);

    // 禁用示教模式，恢复正常安全检查
    disable_teaching_mode();

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] TrajectoryRecordController deactivated",
                mapping.c_str());
    return true;
}


void TrajectoryRecordController::teach_callback(const controller_interfaces::msg::TeachingControl::SharedPtr msg) {
    // ROS 侧示教控制：直接操作 RecorderManager，不调用 IPC 侧的 execute()

    std::string action = msg->action;
    std::string mapping = msg->mapping;
    std::string filename = msg->filename;

    RCLCPP_INFO(node_->get_logger(), "📨 TeachingControl: action=%s, mapping=%s",
                action.c_str(), mapping.c_str());

    if (action == "Start") {
        // ✅ 记录本次录制会话的目标 mappings（支持单臂）
        std::vector<std::string> target_mappings;
        if (!mapping.empty() && mapping != "*") {
            target_mappings.push_back(mapping);
        } else {
            target_mappings = hardware_manager_->get_all_mappings();
        }
        {
            std::lock_guard<std::mutex> lock(recording_mappings_mutex_);
            recording_mappings_ = target_mappings;
        }

        // ✅ 直接操作 RecorderManager，不调用 execute()
        std::string file_path = record_dir_ + "/" + filename + ".csv";

        if (!recorder_manager_.startRecording(file_path)) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Failed to start recording: %s", file_path.c_str());
            return;
        }

        auto recorder = recorder_manager_.getRecorder();
        if (!recorder) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Failed to get recorder");
            recorder_manager_.cancelRecording();
            return;
        }

        // ✅ 仅注册本次会话目标 mappings 对应的 interface
        std::set<std::string> registered_interfaces;
        for (const auto& target_mapping : target_mappings) {
            const std::string& interface = hardware_manager_->get_interface(target_mapping);
            const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(target_mapping);
            if (interface.empty() || motor_ids.empty()) {
                continue;
            }
            if (registered_interfaces.insert(interface).second) {
                recorder->register_interface(interface, motor_ids);
                RCLCPP_INFO(node_->get_logger(), "✅ Registered interface %s with %zu motors (mapping: %s)",
                            interface.c_str(), motor_ids.size(), target_mapping.c_str());
            }
        }

        if (!hardware_manager_->register_motor_recorder(recorder)) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Failed to register motor recorder observer");
            recorder_manager_.cancelRecording();
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "✅ Started recording: %s", file_path.c_str());

    } else if (action == "Pause") {
        pause();

    } else if (action == "Resume") {
        resume();

    } else if (action == "Cancel") {
        cancel();

    } else if (action == "Complete") {
        complete();

    } else {
        RCLCPP_WARN(node_->get_logger(), "❎ Unknown action: %s", action.c_str());
    }
}

// 虚方法实现 - 来自 TeachControllerBase 接口
void TrajectoryRecordController::pause() {
    if (!recorder_manager_.pauseRecording()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to pause recording");
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "✅ Paused recording");
}

void TrajectoryRecordController::resume() {
    if (!recorder_manager_.resumeRecording()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to resume recording");
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "✅ Resumed recording");
}

void TrajectoryRecordController::cancel() {
    // ✅ 取消录制并删除文件
    std::string file_path = recorder_manager_.getFilePath();

    if (!recorder_manager_.cancelRecording()) {
        RCLCPP_WARN(node_->get_logger(), "No recording to cancel");
        return;
    }

    // 尝试删除文件
    if (!file_path.empty()) {
        try {
            std::filesystem::remove(file_path);
            RCLCPP_INFO(node_->get_logger(), "✅ Trajectory file deleted: %s", file_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Failed to delete file: %s", e.what());
        }
    }

    // ✅ 注销观察者
    hardware_manager_->unregister_motor_recorder();
    {
        std::lock_guard<std::mutex> lock(recording_mappings_mutex_);
        recording_mappings_.clear();
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Cancelled recording");
}

void TrajectoryRecordController::complete() {
    // ✅ 完成并保存录制
    std::string file_path = recorder_manager_.getFilePath();

    if (!recorder_manager_.stopRecording()) {
        RCLCPP_WARN(node_->get_logger(), "No recording to complete");
        return;
    }

    // ✅ 注销观察者
    hardware_manager_->unregister_motor_recorder();
    {
        std::lock_guard<std::mutex> lock(recording_mappings_mutex_);
        recording_mappings_.clear();
    }

    if (!file_path.empty()) {
        RCLCPP_INFO(node_->get_logger(), "✅ Recording completed and saved: %s", file_path.c_str());

        // 可选：平滑处理已保存的轨迹
        smooth_recorded_trajectory(file_path);
    }
}

// ============ 持续重力补偿线程实现（per-mapping） ============

void TrajectoryRecordController::start_gravity_compensation_thread(const std::string& mapping) {
    std::lock_guard<std::mutex> lock(gravity_compensation_threads_mutex_);

    // 检查该 mapping 的线程是否已在运行
    if (gravity_compensation_running_.find(mapping) != gravity_compensation_running_.end() &&
        gravity_compensation_running_[mapping].load(std::memory_order_acquire)) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️  Gravity compensation thread already running", mapping.c_str());
        return;
    }

    // 创建该 mapping 的运行标志
    gravity_compensation_running_[mapping].store(true, std::memory_order_release);

    // 为该 mapping 启动独立的线程
    gravity_compensation_threads_[mapping] = std::make_unique<std::thread>(
        &TrajectoryRecordController::gravity_compensation_thread_func, this, mapping);

    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Gravity compensation thread created", mapping.c_str());
}

void TrajectoryRecordController::stop_gravity_compensation_thread(const std::string& mapping) {
    std::lock_guard<std::mutex> lock(gravity_compensation_threads_mutex_);

    auto it = gravity_compensation_running_.find(mapping);
    if (it == gravity_compensation_running_.end() || !it->second.load(std::memory_order_acquire)) {
        return;
    }

    // 设置停止标志
    gravity_compensation_running_[mapping].store(false, std::memory_order_release);

    // 等待线程完成
    auto thread_it = gravity_compensation_threads_.find(mapping);
    if (thread_it != gravity_compensation_threads_.end() && thread_it->second && thread_it->second->joinable()) {
        thread_it->second->join();
        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Gravity compensation thread stopped", mapping.c_str());
    }

    // 清理线程和标志
    gravity_compensation_threads_.erase(mapping);
    gravity_compensation_running_.erase(mapping);
}

void TrajectoryRecordController::gravity_compensation_thread_func(const std::string& mapping) {
    // ✅ 简化：atomic<bool> 读操作本身线程安全，不需要 mutex
    while (gravity_compensation_running_[mapping].load(std::memory_order_acquire))
    {
        try {
            auto hardware_driver = hardware_manager_->get_hardware_driver();
            if (!hardware_driver) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(GRAVITY_COMPENSATION_INTERVAL_MS)));
                continue;
            }

            // ✅ 只为该 mapping 计算重力补偿
            try {
                const std::string& interface = hardware_manager_->get_interface(mapping);
                const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);

                // 计算重力补偿力矩（实时计算）
                std::vector<double> gravity_torques = hardware_manager_->compute_gravity_torques(mapping);

                // ✅ 使用批量接口发送补偿力矩（一次性发送给所有电机，避免单个电机逐个发送的延迟）
                // 准备数组格式的数据 - send_realtime_mit_command支持最多6个电机
                std::array<double, 6> positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                std::array<double, 6> velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                std::array<double, 6> efforts = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                std::array<double, 6> kps = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                std::array<double, 6> kds = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                // 填充effort数组
                for (size_t i = 0; i < std::min(motor_ids.size(), size_t(6)); ++i) {
                    efforts[i] = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;
                }

                // 使用批量命令发送重力补偿力矩给所有电机（一次CAN报文）
                hardware_driver->send_realtime_mit_command(
                    interface,
                    positions,
                    velocities,
                    efforts,
                    kps,
                    kds
                );
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(),
                            "[%s] ⚠️  Exception in gravity compensation: %s",
                            mapping.c_str(), e.what());
            }

            // 等待指定的时间间隔后再次执行
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(GRAVITY_COMPENSATION_INTERVAL_MS)));

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                        "[%s] ❎ Exception in gravity compensation thread: %s", mapping.c_str(), e.what());

            // 发生异常后继续运行，等待下一次循环
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(GRAVITY_COMPENSATION_INTERVAL_MS)));
        }
    }
}

void TrajectoryRecordController::smooth_recorded_trajectory(const std::string& file_path) {
    if (!trajectory_smoother_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ TrajectorySmoother not initialized");
        return;
    }

    try {
        // 按interface分组加载轨迹数据（can0和can1分开）
        std::map<std::string, std::vector<double>> interface_times;
        std::map<std::string, std::vector<std::vector<double>>> interface_positions;
        std::map<std::string, std::vector<std::vector<double>>> interface_velocities;
        std::map<std::string, std::vector<std::vector<double>>> interface_efforts;

        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open trajectory file: %s", file_path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line);  // 跳过表头

        while (std::getline(file, line)) {
            // 去除行尾的空白字符
            while (!line.empty() && (line.back() == '\r' || line.back() == '\n' ||
                   line.back() == ' ' || line.back() == '\t')) {
                line.pop_back();
            }

            if (line.empty()) continue;

            // CSV解析
            std::vector<std::string> tokens;
            std::stringstream ss(line);
            std::string token;

            while (std::getline(ss, token, ',')) {
                token.erase(0, token.find_first_not_of(" \t"));
                token.erase(token.find_last_not_of(" \t") + 1);
                tokens.push_back(token);
            }

            if (tokens.size() < 20) continue;  // timestamp + interface + 6*pos + 6*vel + 6*eff

            try {
                std::string interface = tokens[1];  // can0 或 can1
                double timestamp = std::stod(tokens[0]);

                interface_times[interface].push_back(timestamp);

                std::vector<double> pos(6), vel(6), eff(6);
                for (int i = 0; i < 6; i++) {
                    pos[i] = std::stod(tokens[2 + i]);
                    vel[i] = std::stod(tokens[8 + i]);
                    eff[i] = std::stod(tokens[14 + i]);
                }
                interface_positions[interface].push_back(pos);
                interface_velocities[interface].push_back(vel);
                interface_efforts[interface].push_back(eff);
            } catch (...) {
                continue;
            }
        }
        file.close();

        if (interface_positions.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ No valid trajectory data loaded from: %s", file_path.c_str());
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "✅ Loaded trajectory data for %zu interfaces", interface_positions.size());

        // 为每个interface分别进行平滑处理
        std::map<std::string, std::vector<std::vector<double>>> smoothed_positions;
        std::map<std::string, std::vector<std::vector<double>>> smoothed_velocities;
        std::map<std::string, std::vector<double>> smoothed_times;

        for (const auto& [interface, positions] : interface_positions) {
            auto& times = interface_times[interface];
            auto& velocities = interface_velocities[interface];
            auto& efforts = interface_efforts[interface];

            // Ensure each interface stream is strictly time-ordered before smoothing.
            // Out-of-order samples can inject spline artifacts that look like trajectory jumps.
            std::vector<size_t> indices(times.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(),
                      [&times](size_t a, size_t b) { return times[a] < times[b]; });

            std::vector<double> sorted_times(times.size());
            std::vector<std::vector<double>> sorted_positions(positions.size());
            std::vector<std::vector<double>> sorted_velocities(velocities.size());
            std::vector<std::vector<double>> sorted_efforts(efforts.size());
            for (size_t i = 0; i < indices.size(); ++i) {
                sorted_times[i] = times[indices[i]];
                sorted_positions[i] = positions[indices[i]];
                sorted_velocities[i] = velocities[indices[i]];
                sorted_efforts[i] = efforts[indices[i]];
            }

            // 根据interface确定mapping（从 hardware_manager 获取，而不是硬编码）
            std::string mapping = hardware_manager_->get_mapping_by_interface(interface);
            if (mapping.empty()) {
                RCLCPP_WARN(node_->get_logger(), "⚠️ Failed to get mapping for interface %s", interface.c_str());
                continue;
            }
            auto joint_names = hardware_manager_->get_joint_names(mapping);
            RCLCPP_INFO(node_->get_logger(), "   joint_names size: %zu", joint_names.size());

            // 使用 TrajectorySmoother 进行 CSAPS 平滑
            auto smooth_traj = trajectory_smoother_->smooth(sorted_times, sorted_positions, sorted_velocities, sorted_efforts,
                                                           joint_names, true, true);

            if (smooth_traj.points.empty()) {
                RCLCPP_WARN(node_->get_logger(), "⚠️  smooth() returned empty trajectory for interface %s - skipping this interface", interface.c_str());
                continue;
            }

            // 提取平滑后的数据
            std::vector<double> smooth_times;
            std::vector<std::vector<double>> smooth_pos, smooth_vel;
            for (const auto& point : smooth_traj.points) {
                smooth_times.push_back(point.time_from_start);
                smooth_pos.push_back(point.positions);
                smooth_vel.push_back(point.velocities);
            }

            smoothed_times[interface] = smooth_times;
            smoothed_positions[interface] = smooth_pos;
            smoothed_velocities[interface] = smooth_vel;
        }

        // 将平滑后的轨迹写入新文件，按时间戳排序（交错can0和can1）
        std::string smooth_file_path = file_path.substr(0, file_path.rfind(".csv")) + "_smooth.csv";
        std::ofstream out_file(smooth_file_path);
        if (!out_file.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open output file: %s", smooth_file_path.c_str());
            return;
        }

        // 写入表头
        out_file << "timestamp,interface";
        for (int i = 1; i <= 6; i++) out_file << ",position" << i;
        for (int i = 1; i <= 6; i++) out_file << ",velocity" << i;
        for (int i = 1; i <= 6; i++) out_file << ",effort" << i;
        out_file << "\n";

        // 合并所有interface的数据并按时间戳排序
        struct TrajectoryPoint {
            double timestamp;
            std::string interface;
            std::vector<double> position;
            std::vector<double> velocity;
        };
        std::vector<TrajectoryPoint> all_points;

        for (const auto& [interface, times] : smoothed_times) {
            const auto& positions = smoothed_positions[interface];
            const auto& velocities = smoothed_velocities[interface];

            for (size_t i = 0; i < times.size(); i++) {
                all_points.push_back({
                    times[i],
                    interface,
                    positions[i],
                    velocities[i]
                });
            }
        }

        // 按时间戳排序
        std::sort(all_points.begin(), all_points.end(),
                  [](const TrajectoryPoint& a, const TrajectoryPoint& b) {
                      return a.timestamp < b.timestamp;
                  });

        // 按时间顺序写入平滑数据
        for (const auto& point : all_points) {
            out_file << point.timestamp << "," << point.interface;
            for (double pos : point.position) out_file << "," << pos;
            for (double vel : point.velocity) out_file << "," << vel;
            // 加速度不写入，用0填充
            for (int j = 0; j < 6; j++) {
                out_file << ",0.0";
            }
            out_file << "\n";
        }
        out_file.close();

        RCLCPP_INFO(node_->get_logger(), "✅ Smoothed trajectory saved: %s (sorted by timestamp)", smooth_file_path.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Error smoothing trajectory: %s", e.what());
    }
}

// ============ IPC 接口实现 ============

bool TrajectoryRecordController::execute(const std::string& mapping, const std::string& command, const std::string& filename) {
    // mapping: 具体的映射名（仅在 start 时使用，用于初始化）
    // command: start、pause、resume、cancel、complete
    // filename: 要操作的录制文件名

    if (command == "start") {
        // ✅ 检查 mapping 是否有效
        if (mapping.empty()) {
            RCLCPP_ERROR(node_->get_logger(),
                "Cannot start recording: mapping is empty");
            return false;
        }

        // 1. 确保前一个录制已完成
        if (recorder_manager_.getState() != RecorderManager::State::IDLE) {
            RCLCPP_ERROR(node_->get_logger(),
                "Previous recording not completed");
            return false;
        }

        // 2. 初始化控制器（如果尚未初始化）
        bool need_init = false;
        {
            std::lock_guard<std::mutex> lock(active_mappings_mutex_);
            need_init = (active_mappings_.find(mapping) == active_mappings_.end() ||
                         !active_mappings_[mapping]);
        }

        if (need_init) {
            RCLCPP_ERROR(node_->get_logger(),
                "[%s] Controller not initialized before execute", mapping.c_str());
            return false;
        }

        // 3. 启动录制
        std::string file_path = record_dir_ + "/" + filename + ".csv";
        if (!recorder_manager_.startRecording(file_path)) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to start recording: %s", file_path.c_str());
            return false;
        }

        // 4. 获取 recorder 并注册硬件观察者（仅一次）
        auto recorder = recorder_manager_.getRecorder();
        if (!recorder) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to get recorder");
            recorder_manager_.cancelRecording();
            return false;
        }

        // ✅ 仅注册本次录制会话目标 mappings 的 interface（支持单臂）
        std::vector<std::string> session_mappings;
        {
            std::lock_guard<std::mutex> lock(recording_mappings_mutex_);
            session_mappings = recording_mappings_;
        }
        if (session_mappings.empty()) {
            // 兜底：若未设置会话目标，退化为当前命令 mapping
            session_mappings.push_back(mapping);
        }

        std::set<std::string> registered_interfaces;
        for (const auto& mapping_str : session_mappings) {
            const std::string& interface = hardware_manager_->get_interface(mapping_str);
            const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping_str);
            if (interface.empty() || motor_ids.empty()) {
                continue;
            }
            if (registered_interfaces.insert(interface).second) {
                recorder->register_interface(interface, motor_ids);
                RCLCPP_INFO(node_->get_logger(),
                            "✅ Registered interface %s with %zu motors (mapping: %s)",
                            interface.c_str(), motor_ids.size(), mapping_str.c_str());
            }
        }

        if (!hardware_manager_->register_motor_recorder(recorder)) {
            RCLCPP_ERROR(node_->get_logger(),
                "Failed to register motor recorder observer");
            recorder_manager_.cancelRecording();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
            "[%s] Recording started: %s", mapping.c_str(), file_path.c_str());

        return true;
    }

    else if (command == "pause") {
        if (!recorder_manager_.pauseRecording()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to pause recording");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "Recording paused");
        return true;
    }

    else if (command == "resume") {
        if (!recorder_manager_.resumeRecording()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to resume recording");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "Recording resumed");
        return true;
    }

    else if (command == "complete") {
        // ✅ 检查是否有活动的录制
        if (recorder_manager_.getState() == RecorderManager::State::IDLE) {
            RCLCPP_WARN(node_->get_logger(), "No active recording to complete");
            return false;
        }

        // ✅ 关键：在 stopRecording() 之前保存文件路径，否则 stopRecording() 会清空它
        std::string file_to_smooth = recorder_manager_.getFilePath();

        if (!recorder_manager_.stopRecording()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to complete recording");
            return false;
        }
        hardware_manager_->unregister_motor_recorder();
        {
            std::lock_guard<std::mutex> lock(recording_mappings_mutex_);
            recording_mappings_.clear();
        }

        // ✅ 现在用保存的路径进行平滑处理
        if (!file_to_smooth.empty()) {
            smooth_recorded_trajectory(file_to_smooth);
        } else {
            RCLCPP_WARN(node_->get_logger(), "⚠️ No file path from recorder, skipping smooth");
        }
        RCLCPP_INFO(node_->get_logger(), "Recording completed");
        return true;
    }

    else if (command == "cancel") {
        // ✅ 检查是否有活动的录制
        if (recorder_manager_.getState() == RecorderManager::State::IDLE) {
            RCLCPP_WARN(node_->get_logger(), "No active recording to cancel");
            return false;
        }

        if (!recorder_manager_.cancelRecording()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to cancel recording");
            return false;
        }
        hardware_manager_->unregister_motor_recorder();
        {
            std::lock_guard<std::mutex> lock(recording_mappings_mutex_);
            recording_mappings_.clear();
        }
        RCLCPP_INFO(node_->get_logger(), "Recording cancelled");
        return true;
    }

    RCLCPP_WARN(node_->get_logger(),
        "Unknown command: %s", command.c_str());

    return false;
}

void TrajectoryRecordController::command_queue_consumer_thread() {
    arm_controller::CommandIPC cmd;
    bool has_pending_start = false;
    std::string pending_mapping;
    std::string pending_filename;
    std::vector<std::string> pending_target_mappings;

    while (consumer_running_) {
        std::string mapping;               // 获取映射（可能为空）
        std::string filename;              // 记录的轨迹文件名
        std::string action;                // start、pause、resume、cancel、complete
        std::vector<std::string> target_mappings;

        if (has_pending_start) {
            // 继续处理上一条挂起的 start 命令（切模式完成后落地执行）
            action = "start";
            mapping = pending_mapping;
            filename = pending_filename;
            target_mappings = pending_target_mappings;
        } else {
            // ✅ 使用带过滤的 pop，只获取 TrajectoryRecord 命令
            if (!arm_controller::CommandQueueIPC::getInstance().popWithFilter(cmd, "TrajectoryRecord", 10)) {
                continue;
            }

            mapping = cmd.get_mapping();
            filename = cmd.get_filename();
            action = cmd.get_action();

            // 计算本次命令作用的 mappings：
            // - start：按命令mapping决定（空/"*" => 所有）
            // - 其他动作：默认作用于当前录制会话（避免把未参与录制的手臂拉入TrajectoryRecord）
            if (action == "start") {
                if (mapping.empty() || mapping == "*") {
                    if (hardware_manager_) {
                        for (const auto& m : hardware_manager_->get_all_mappings()) {
                            // 仅对有电机反馈的 arm 映射启用录制，跳过纯软件映射（如 gripper）
                            if (!hardware_manager_->get_motors_id(m).empty()) {
                                target_mappings.push_back(m);
                            }
                        }
                    }
                } else {
                    target_mappings.push_back(mapping);
                }
            } else {
                if (!mapping.empty() && mapping != "*") {
                    target_mappings.push_back(mapping);
                } else {
                    std::lock_guard<std::mutex> lock(recording_mappings_mutex_);
                    target_mappings = recording_mappings_;
                }
            }
        }

        if (target_mappings.empty()) {
            if (action == "start") {
                RCLCPP_WARN(node_->get_logger(), "❎ TrajectoryRecord: No mappings to process for start");
            } else {
                RCLCPP_WARN(node_->get_logger(), "❎ TrajectoryRecord: No active recording session");
            }
            has_pending_start = false;
            continue;
        }

        try {
            // ✅ start 两阶段：先触发切模式，待模式就绪后再落地执行 startRecording
            if (action == "start") {
                bool mode_ready = true;
                for (const auto& target_mapping : target_mappings) {
                    std::lock_guard<std::mutex> execution_lock(arm_controller::CommandQueueIPC::getMappingExecutionMutex(target_mapping));
                    auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(target_mapping);

                    if (!state_mgr) {
                        mode_ready = false;
                        continue;
                    }

                    // 若当前并非目标模式，先请求 ControllerManager 做真实模式切换（先不消费 start）
                    std::string current_mode = state_mgr->getCurrentMode();
                    if (current_mode != "TrajectoryRecord") {
                        if (hook_request_callback_) {
                            hook_request_callback_(target_mapping, "TrajectoryRecord");
                        }
                        mode_ready = false;
                    }
                }

                if (!mode_ready) {
                    has_pending_start = true;
                    pending_mapping = mapping;
                    pending_filename = filename;
                    pending_target_mappings = target_mappings;

                    for (const auto& target_mapping : target_mappings) {
                        std::lock_guard<std::mutex> execution_lock(
                            arm_controller::CommandQueueIPC::getMappingExecutionMutex(target_mapping));
                        auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(target_mapping);
                        if (state_mgr) {
                            state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::PENDING);
                        }
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
                    continue;
                }

                // 模式就绪，清理 pending_start
                has_pending_start = false;
                pending_mapping.clear();
                pending_filename.clear();
                pending_target_mappings.clear();
            }

            // 更新状态为执行中
            for (const auto& target_mapping : target_mappings) {
                std::lock_guard<std::mutex> execution_lock(arm_controller::CommandQueueIPC::getMappingExecutionMutex(target_mapping));
                auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(target_mapping);
                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::EXECUTING);
                }
            }

            // ✅ 执行命令
            // Recorder 是全局单例，所有操作都只需执行一次
            // Controller 初始化（如需要）在 execute() 内部按 mapping 处理
            bool success = true;

            // ✅ 重置 complete 标志（新的录制会话开始）
            if (action == "start") {
                complete_executed_in_session_.store(false, std::memory_order_release);
                {
                    std::lock_guard<std::mutex> lock(recording_mappings_mutex_);
                    recording_mappings_ = target_mappings;
                }
                for (const auto& m : target_mappings)
                {
                    try
                    {
                        // 模式切换已由 ControllerManager 完成，避免在这里重复 start()
                        if (!is_active(m)) {
                            start(m);
                        }
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_ERROR(node_->get_logger(),
                            "[%s] Failed to initialize: %s",
                            m.c_str(), e.what());
                        success = false;
                    }
                }
            }

            // ✅ 防止 complete 在同一会话中执行多次
            if (action == "complete") {
                if (complete_executed_in_session_.exchange(true, std::memory_order_acq_rel)) {
                    RCLCPP_WARN(node_->get_logger(), "⚠️ Complete already executed in this session, skipping");
                    success = true;  // 视为成功（已完成过）
                } else {
                    if (!execute(target_mappings[0], action, filename)) {
                        success = false;
                    }
                }
            } else {
                if (!execute(target_mappings[0], action, filename)) {
                    success = false;
                }
            }

            if (success) {
                RCLCPP_INFO(node_->get_logger(), "✅ TrajectoryRecord command executed successfully (action: %s)",
                           action.c_str());
            } else {
                RCLCPP_ERROR(node_->get_logger(), "❎ TrajectoryRecord command execution failed (action: %s)",
                           action.c_str());
                if (action == "start") {
                    has_pending_start = false;
                    pending_mapping.clear();
                    pending_filename.clear();
                    pending_target_mappings.clear();
                }
            }

            // ✅ 延迟后为每个 target mapping 更新状态
            // 只在 cancel 或 complete 时恢复到 IDLE，其他命令保持当前状态
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            bool should_idle = (action == "cancel" || action == "complete");
            for (const auto& target_mapping : target_mappings) {
                std::lock_guard<std::mutex> execution_lock(arm_controller::CommandQueueIPC::getMappingExecutionMutex(target_mapping));
                auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(target_mapping);

                if (state_mgr) {
                    if (success) {
                        state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::SUCCESS);
                    } else {
                        state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                    }

                    if (should_idle) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);

                        // ✅ 命令执行完成后更新状态
                        arm_controller::ipc::ExecutorControllerState executor_state;
                        strncpy(executor_state.current_mode, "TrajectoryRecord", sizeof(executor_state.current_mode) - 1);
                        executor_state.current_mode[sizeof(executor_state.current_mode) - 1] = '\0';
                        executor_state.execution_state = (int)arm_controller::ipc::ExecutionState::IDLE;
                        state_mgr->updateFromExecutor(executor_state);
                    }
                }
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Exception in TrajectoryRecord command execution: %s",
                        e.what());

            // ✅ 为所有 target mappings 设置失败状态
            for (const auto& target_mapping : target_mappings) {
                std::lock_guard<std::mutex> execution_lock(arm_controller::CommandQueueIPC::getMappingExecutionMutex(target_mapping));
                auto state_mgr = arm_controller::ipc::IPCContext::getInstance().getStateManager(target_mapping);

                if (state_mgr) {
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::FAILED);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    state_mgr->setExecutionState(arm_controller::ipc::ExecutionState::IDLE);
                }
            }
        }

        // ✅ 通知其他 consumers
        arm_controller::CommandQueueIPC::getInstance().notifyConsumers();
    }
}
