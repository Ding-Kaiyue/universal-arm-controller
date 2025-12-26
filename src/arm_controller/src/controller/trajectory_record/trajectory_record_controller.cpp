#include "trajectory_record_controller.hpp"
#include "controller_interface.hpp"
#include <filesystem>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'TrajectoryRecord', mapping: 'single_arm'}"
// ros2 topic pub --once /controller_api/trajectory_record_action/single_arm std_msgs/msg/String 'data: "little_nie"'
// ros2 topic pub --once /controller_api/trajectory_record_control/single_arm std_msgs/msg/String 'data: "complete"'

TrajectoryRecordController::TrajectoryRecordController(const rclcpp::Node::SharedPtr & node)
    : TeachControllerBase("TrajectoryRecord", node)
{
    // 获取硬件管理器实例
    hardware_manager_ = HardwareManager::getInstance();

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

    try {
        const std::string& interface = hardware_manager_->get_interface(mapping);
        const std::vector<uint32_t>& motor_ids = hardware_manager_->get_motors_id(mapping);

        // MIT模式速度控制参数
        const double kp_velocity = 0.0;      // 速度模式：kp=0.0
        const double kd_velocity = 0.0;     // 速度模式：kd=0.0

        std::vector<double> gravity_torques = hardware_manager_->compute_gravity_torques(mapping);
        
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            uint32_t motor_id = motor_ids[i]; 
            double effort = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;        // 重力补偿力矩
            hardware_driver->control_motor_in_mit_mode(interface, motor_id, 0.0, 0.0, effort, kp_velocity, kd_velocity);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to send joint velocities: %s", mapping.c_str(), e.what());
        return;
    }
}

bool TrajectoryRecordController::stop(const std::string& mapping) {
    is_active_ = false;

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

    motor_recorder_.reset();  // 销毁本地的记录器对象
    recording_ = false;
    paused_ = false;
    if (!current_recording_file_path_.empty()) {
        current_recording_file_path_.clear();
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Trajectory recording completed (mapping: %s)", mapping.c_str());
}
