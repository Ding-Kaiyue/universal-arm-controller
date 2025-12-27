#include "trajectory_record_controller.hpp"
#include "controller_interface.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
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

    // 保存当前激活的 mapping
    active_mapping_ = mapping.empty() ? "single_arm" : mapping;
    is_active_ = true;

    recording_ = false;
    paused_ = false;

    // 启用示教模式 - 防止安全限位检查触发急停
    enable_teaching_mode();

    RCLCPP_INFO(node_->get_logger(), "[%s] TrajectoryRecordController activated",
                active_mapping_.c_str());

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return;
    }

    // ✅ 启动持续重力补偿线程
    start_gravity_compensation_thread();

    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Gravity compensation thread started", mapping.c_str());
}

bool TrajectoryRecordController::stop(const std::string& mapping) {
    is_active_ = false;

    // ✅ 停止重力补偿线程
    stop_gravity_compensation_thread();

    // 禁用示教模式，恢复正常安全检查
    disable_teaching_mode();

    // ✅ 清理电机数据记录器
    if (motor_recorder_) {
        motor_recorder_->sync_to_disk();

        // ✅ 先从 HardwareManager 中注销观察者，防止悬垂指针
        hardware_manager_->unregister_motor_recorder();

        motor_recorder_.reset();
        RCLCPP_INFO(node_->get_logger(), "Motor data recording stopped");
    }

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    // 清理资源
    recording_ = false;
    paused_ = false;
    active_mapping_.clear();
    current_recording_file_path_.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] TrajectoryRecordController deactivated",
                mapping.c_str());
    return true;
}


void TrajectoryRecordController::teach_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_ || msg->data.empty()) return;

    /* -------- already recording -------- */
    if (recording_) {
        RCLCPP_WARN(node_->get_logger(), "❎ Already recording, ignoring new command");
        return;
    }

    /* -------- start new recording -------- */
    current_recording_file_path_ = record_dir_ + "/" + msg->data + ".csv";

    // 创建新的 MotorDataRecorder 实例
    motor_recorder_ = std::make_shared<MotorDataRecorder>(current_recording_file_path_, active_mapping_);

    if (!motor_recorder_->is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open trajectory file: %s", current_recording_file_path_.c_str());
        motor_recorder_.reset();
        return;
    }

    // 禁用单数据模式 - TrajectoryRecord 需要持续记录所有数据
    motor_recorder_->set_single_record_mode(false);

    // 将 MotorDataRecorder 作为观察者注册到 HardwareManager
    // HardwareManager 本身实现了 MotorStatusObserver，我们让它转发给 motor_recorder_
    if (!hardware_manager_->register_motor_recorder(motor_recorder_)) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to register motor recorder observer");
        motor_recorder_.reset();
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "✅ Motor data recorder registered as observer");

    recording_ = true;
    paused_ = false;

    RCLCPP_INFO(node_->get_logger(), "✅ Started trajectory recording: %s", current_recording_file_path_.c_str());
}

void TrajectoryRecordController::on_teaching_control(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_ || msg->data.empty()) return;

    if (msg->data == "pause") {
        pause(active_mapping_);
    } else if (msg->data == "resume") {
        resume(active_mapping_);
    } else if (msg->data == "cancel") {
        cancel(active_mapping_);
    } else if (msg->data == "complete") {
        complete(active_mapping_);
    } else {
        RCLCPP_WARN(node_->get_logger(), "❎ Unknown teaching control command: %s", msg->data.c_str());
    }
}

// 虚方法实现 - 来自 TeachControllerBase 接口
void TrajectoryRecordController::pause(const std::string& mapping) {
    if (!recording_ || paused_) return;

    paused_ = true;
    motor_recorder_->pause(mapping);
    RCLCPP_INFO(node_->get_logger(), "✅ Trajectory recording paused (mapping: %s)", mapping.c_str());
}

void TrajectoryRecordController::resume(const std::string& mapping) {
    if (!recording_ || !paused_) return;

    paused_ = false;
    motor_recorder_->resume(mapping);

    RCLCPP_INFO(node_->get_logger(), "✅ Trajectory recording resumed (mapping: %s)", mapping.c_str());
}

void TrajectoryRecordController::cancel(const std::string& mapping) {
    if (!recording_) return;

    // ✅ 先从 HardwareManager 中注销观察者，防止悬垂指针
    hardware_manager_->unregister_motor_recorder();

    motor_recorder_.reset();  // 观察者会自动停止接收更新
    recording_ = false;
    paused_ = false;

    // 删除已录制的文件
    if (!current_recording_file_path_.empty()) {
        try {
            std::filesystem::remove(current_recording_file_path_);
            RCLCPP_INFO(node_->get_logger(), "✅ Trajectory recording cancelled and file deleted: %s", mapping.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "⚠️  Failed to delete file: %s", e.what());
        }
        current_recording_file_path_.clear();
    }
}

void TrajectoryRecordController::complete(const std::string& mapping) {
    if (!recording_) return;

    motor_recorder_->sync_to_disk();

    // ✅ 先从 HardwareManager 中注销观察者，防止悬垂指针
    hardware_manager_->unregister_motor_recorder();

    std::string file_path = current_recording_file_path_;

    motor_recorder_.reset();  // 销毁本地的记录器对象
    recording_ = false;
    paused_ = false;
    if (!current_recording_file_path_.empty()) {
        current_recording_file_path_.clear();
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Trajectory recording completed (mapping: %s)", mapping.c_str());

    // ✅ 在后台线程中进行 CSAPS 平滑处理，不阻塞主线程
    std::thread smoothing_thread([this, file_path]() {
        this->smooth_recorded_trajectory(file_path);
    });
    smoothing_thread.detach();
}

// ============ 持续重力补偿线程实现 ============

void TrajectoryRecordController::start_gravity_compensation_thread() {
    if (gravity_compensation_running_) {
        RCLCPP_WARN(node_->get_logger(), "⚠️  Gravity compensation thread already running");
        return;
    }

    gravity_compensation_running_ = true;
    gravity_compensation_thread_ = std::make_unique<std::thread>(
        &TrajectoryRecordController::gravity_compensation_thread_func, this);

    RCLCPP_INFO(node_->get_logger(), "✅ Gravity compensation thread created");
}

void TrajectoryRecordController::stop_gravity_compensation_thread() {
    if (!gravity_compensation_running_) {
        return;
    }

    gravity_compensation_running_ = false;

    if (gravity_compensation_thread_ && gravity_compensation_thread_->joinable()) {
        gravity_compensation_thread_->join();
        RCLCPP_INFO(node_->get_logger(), "✅ Gravity compensation thread stopped");
    }

    gravity_compensation_thread_.reset();
}

void TrajectoryRecordController::gravity_compensation_thread_func() {
    while (gravity_compensation_running_) {
        try {
            // 每 GRAVITY_COMPENSATION_INTERVAL_MS 毫秒执行一次重力补偿
            auto hardware_driver = hardware_manager_->get_hardware_driver();
            if (!hardware_driver) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(GRAVITY_COMPENSATION_INTERVAL_MS)));
                continue;
            }

            // 获取当前映射信息
            if (active_mapping_.empty()) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(GRAVITY_COMPENSATION_INTERVAL_MS)));
                continue;
            }

            const std::string& interface = hardware_manager_->get_interface(active_mapping_);
            const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(active_mapping_);

            // 计算重力补偿力矩（实时计算）
            std::vector<double> gravity_torques = hardware_manager_->compute_gravity_torques(active_mapping_);

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

            // 等待指定的时间间隔后再次执行
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(GRAVITY_COMPENSATION_INTERVAL_MS)));

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                        "❎ Exception in gravity compensation thread: %s", e.what());

            // 发生异常后继续运行，等待下一次循环
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(GRAVITY_COMPENSATION_INTERVAL_MS)));
        }
    }

    RCLCPP_INFO(node_->get_logger(), "ℹ️  Gravity compensation thread exited");
}

void TrajectoryRecordController::smooth_recorded_trajectory(const std::string& file_path) {
    if (!trajectory_smoother_) {
        RCLCPP_ERROR(node_->get_logger(), "❎ TrajectorySmoother not initialized");
        return;
    }

    try {
        // 加载原始轨迹数据
        std::vector<double> times;
        std::vector<std::vector<double>> positions, velocities, efforts;

        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open trajectory file: %s", file_path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line);  // 跳过表头

        int line_count = 0;
        while (std::getline(file, line)) {
            line_count++;

            // 去除行尾的空白字符
            while (!line.empty() && (line.back() == '\r' || line.back() == '\n' ||
                   line.back() == ' ' || line.back() == '\t')) {
                line.pop_back();
            }

            if (line.empty()) continue;

            // 简单的CSV解析
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
                times.push_back(std::stod(tokens[0]));

                std::vector<double> pos(6), vel(6), eff(6);
                for (int i = 0; i < 6; i++) {
                    pos[i] = std::stod(tokens[2 + i]);
                    vel[i] = std::stod(tokens[8 + i]);
                    eff[i] = std::stod(tokens[14 + i]);
                }
                positions.push_back(pos);
                velocities.push_back(vel);
                efforts.push_back(eff);
            } catch (...) {
                continue;
            }
        }
        file.close();

        if (positions.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ No valid trajectory data loaded");
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "✅ Loaded %zu trajectory points from: %s", positions.size(), file_path.c_str());

        // 获取joint names
        auto joint_names = hardware_manager_->get_joint_names(active_mapping_);

        // 使用 TrajectorySmoother 进行 CSAPS 平滑
        auto smooth_traj = trajectory_smoother_->smooth(times, positions, velocities, efforts, 
                                                       joint_names, true, true);

        // 将平滑后的轨迹写入新文件
        std::string smooth_file_path = file_path.substr(0, file_path.rfind(".csv")) + "_smooth.csv";
        std::ofstream out_file(smooth_file_path);
        if (!out_file.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open output file: %s", smooth_file_path.c_str());
            return;
        }

        // 写入表头（与原始CSV格式保持一致）
        out_file << "timestamp,interface";
        for (int i = 1; i <= 6; i++) out_file << ",position" << i;
        for (int i = 1; i <= 6; i++) out_file << ",velocity" << i;
        for (int i = 1; i <= 6; i++) out_file << ",effort" << i;
        out_file << "\n";

        // 写入平滑后的数据
        for (const auto& point : smooth_traj.points) {
            out_file << point.time_from_start << ",canfd";
            for (double pos : point.positions) out_file << "," << pos;
            for (double vel : point.velocities) out_file << "," << vel;
            // 加速度不写入，用0填充
            for (size_t j = 0; j < 6; j++) {
                out_file << ",0.0";
            }
            out_file << "\n";
        }
        out_file.close();

        RCLCPP_INFO(node_->get_logger(), "✅ Smoothed trajectory saved: %s", smooth_file_path.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Error smoothing trajectory: %s", e.what());
    }
}
