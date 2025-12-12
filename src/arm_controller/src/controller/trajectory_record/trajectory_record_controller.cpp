#include "trajectory_record_controller.hpp"
#include "controller_interface.hpp"
#include <filesystem>
#include <thread>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

TrajectoryRecordController::TrajectoryRecordController(const rclcpp::Node::SharedPtr & node)
    : TeachControllerBase("TrajectoryRecord", node)
{
    // 获取硬件管理器实例（用于重力补偿）
    hardware_manager_ = HardwareManager::getInstance();

    // 创建录制器，录制频率为 100Hz
    recorder_ = std::make_unique<JointRecorder>(100.0);

    std::string input_topic, output_topic;
    node_->get_parameter("controllers.TrajectoryRecord.input_topic", input_topic);
    node_->get_parameter("controllers.TrajectoryRecord.output_topic", output_topic);

    // 获取工作空间中的轨迹存储目录
    try {
        // 获取 arm_controller 包的共享目录路径
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("arm_controller");

        // 从包目录获取工作空间根目录 (install/arm_controller -> install -> workspace_root)
        std::filesystem::path pkg_path(package_share_dir);
        std::filesystem::path workspace_root = pkg_path.parent_path().parent_path();

        // 构造轨迹目录路径: workspace_root/trajectories
        record_output_dir_ = (workspace_root / "trajectories").string();

        RCLCPP_INFO(node_->get_logger(), "Trajectory directory: %s", record_output_dir_.c_str());

        // 检查目录是否存在，如果不存在则创建
        if (!std::filesystem::exists(record_output_dir_)) {
            try {
                std::filesystem::create_directories(record_output_dir_);
                RCLCPP_INFO(node_->get_logger(), "✅ Created trajectory directory: %s", record_output_dir_.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "❎ Failed to create directory %s: %s",
                            record_output_dir_.c_str(), e.what());
                record_output_dir_ = "/tmp/arm_recording_trajectories";
                RCLCPP_WARN(node_->get_logger(), "Fallback to: %s", record_output_dir_.c_str());
            }
        }
    } catch (const std::exception& e) {
        record_output_dir_ = "/tmp/arm_recording_trajectories";
        RCLCPP_WARN(node_->get_logger(), "❎ Using fallback trajectory directory: %s", record_output_dir_.c_str());
    }

    // 订阅轨迹录制命令
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        input_topic, rclcpp::QoS(10).reliable(),
        std::bind(&TrajectoryRecordController::teach_callback, this, std::placeholders::_1)
    );

    // 订阅关节状态用于录制
    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&TrajectoryRecordController::joint_states_callback, this, std::placeholders::_1)
    );

    // 进入示教模式时，创建重力补偿订阅
    // 只要控制器激活就持续接收重力补偿力矩
    gravity_torque_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/gravity_torque", 10,
        std::bind(&TrajectoryRecordController::gravity_torque_callback, this, std::placeholders::_1)
    );

    // 发布状态反馈（可选）
    pub_ = node_->create_publisher<std_msgs::msg::String>(output_topic, 10);

    RCLCPP_INFO(node_->get_logger(), "TrajectoryRecordController initialized. Output dir: %s", record_output_dir_.c_str());
}

void TrajectoryRecordController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] TeajectoryRecord: not found in hardware configuration."
        );
    }

    // 保存当前激活的 mapping
    active_mapping_ = mapping.empty() ? "single_arm" : mapping;
    is_active_ = true;

    RCLCPP_INFO(node_->get_logger(), "[%s] TrajectoryRecordController activated with gravity compensation",
                active_mapping_.c_str());
}

bool TrajectoryRecordController::stop(const std::string& mapping) {
    is_active_ = false;

    // 停止任何正在进行的录制
    if (recorder_->isRecording()) {
        recorder_->stop();
        RCLCPP_INFO(node_->get_logger(), "Stopped ongoing recording");
    }

    // 退出示教模式时，销毁重力补偿订阅，停止重力补偿
    gravity_torque_sub_.reset();
    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    // 清理资源
    active_mapping_.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] TrajectoryRecordController deactivated, gravity compensation stopped",
                active_mapping_.c_str());
    return true;
}

void TrajectoryRecordController::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 验证关节状态数据
    if (msg->effort.size() < 6 || msg->position.size() < 6) {
        if (is_active_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "JointState data incomplete! Expected 6, got effort: %zu, position: %zu",
                msg->effort.size(), msg->position.size());
        }
        return;
    }

    // 始终缓存最新的关节状态（不管控制器是否激活）
    // 这样在控制器激活时就已经有数据可用，避免等待超时
    latest_joint_states_ = msg;

    // 只有在控制器激活且正在录制时，才将消息加入队列
    if (is_active_ && recorder_->isRecording()) {
        recorder_->enqueue(latest_joint_states_);
    }
}

void TrajectoryRecordController::teach_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_) return;
    if (msg->data.empty()) return;

    // 处理停止命令
    if (msg->data == "stop") {
        if (recorder_->isRecording()) {
            recorder_->stop();
            RCLCPP_INFO(node_->get_logger(), "Recording stopped");

            // 发布状态
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "stopped";
            pub_->publish(status_msg);
        } else {
            RCLCPP_WARN(node_->get_logger(), "No active recording to stop");
        }
        return;
    }

    // 使用给定的文件名开始录制
    // 如果还没有关节状态数据，等待最多2秒
    if (!latest_joint_states_) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for joint states data...");
        for (int i = 0; i < 20 && !latest_joint_states_ && is_active_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (!recorder_->isRecording() && latest_joint_states_) {
        std::string file_path = record_output_dir_ + "/" + msg->data + ".txt";

        if (recorder_->start(file_path, latest_joint_states_)) {
            RCLCPP_INFO(node_->get_logger(), "Started recording trajectory: '%s'", msg->data.c_str());

            // 发布状态
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "recording:" + msg->data;
            pub_->publish(status_msg);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to start recording: '%s'", msg->data.c_str());
        }
    } else if (recorder_->isRecording()) {
        RCLCPP_WARN(node_->get_logger(), "Already recording! Stop current recording first.");
    } else {
        RCLCPP_WARN(node_->get_logger(), "No joint states available after waiting. Check /joint_states topic.");
    }
}

void TrajectoryRecordController::publish_trajectory_record_name(const std::string &name) {
    auto msg = std_msgs::msg::String();
    msg.data = name;
    pub_->publish(msg);
}

void TrajectoryRecordController::gravity_torque_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 只在控制器激活时处理重力补偿
    if (!is_active_) return;

    // 验证力矩数据
    if (msg->effort.empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "Received empty gravity torque message");
        return;
    }

    // 发送重力补偿力矩到电机
    send_gravity_compensation(active_mapping_, msg->effort);
}

bool TrajectoryRecordController::send_gravity_compensation(const std::string& mapping, const std::vector<double>& efforts) {
    if (!hardware_manager_) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "Hardware manager not initialized");
        return false;
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "Hardware driver not initialized");
        return false;
    }

    try {
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);

        // 重力补偿模式：使用 MIT 模式，kp=0, kd=0，只发送力矩
        const double kp = 0.0;
        const double kd = 0.0;
        const double position = 0.0;   // 位置不使用
        const double velocity = 0.0;   // 速度不使用

        for (size_t i = 0; i < motor_ids.size() && i < efforts.size(); ++i) {
            uint32_t motor_id = motor_ids[i];
            double effort = efforts[i];

            // 发送重力补偿力矩
            hardware_driver->control_motor_in_mit_mode(
                interface, motor_id, position, velocity, effort, kp, kd);
        }

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[%s] Failed to send gravity compensation: %s", mapping.c_str(), e.what());
        return false;
    }
}

