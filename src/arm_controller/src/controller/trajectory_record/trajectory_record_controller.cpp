#include "trajectory_record_controller.hpp"
#include "controller_interface.hpp"
#include <filesystem>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

TrajectoryRecordController::TrajectoryRecordController(const rclcpp::Node::SharedPtr & node)
    : TeachControllerBase("TrajectoryRecord", node)
{
    // 获取硬件管理器实例
    hardware_manager_ = HardwareManager::getInstance();

    // ✅ 参数由 TeachControllerBase::init_subscriptions() 自动处理
    // input_topic0: 文件名输入话题（teach_callback）
    // input_topic1: 录制控制话题（on_teaching_control）

    /* ---------- trajectory directory ---------- */
    try {
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("arm_controller");
        fprintf(stderr, "📦 Package directory: %s\n", pkg_dir.c_str());

        // pkg_dir = /path/to/install/arm_controller/share/arm_controller
        // parent_path() = /path/to/install/arm_controller/share
        // parent_path() = /path/to/install/arm_controller
        // parent_path() = /path/to/install
        std::filesystem::path workspace_root =
            std::filesystem::path(pkg_dir).parent_path().parent_path().parent_path();

        fprintf(stderr, "🏠 Workspace root: %s\n", workspace_root.c_str());

        record_output_dir_ = (workspace_root / "trajectories").string();
        fprintf(stderr, "📁 Output directory: %s\n", record_output_dir_.c_str());

        std::filesystem::create_directories(record_output_dir_);
        fprintf(stderr, "✅ Directory created/exists: %s\n", record_output_dir_.c_str());
    } catch (const std::exception& e) {
        fprintf(stderr, "❌ Exception creating directory: %s\n", e.what());
        record_output_dir_ = "/tmp/arm_recording_trajectories";
        std::filesystem::create_directories(record_output_dir_);
        fprintf(stderr, "⚠️  Fallback to: %s\n", record_output_dir_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "TrajectoryRecordController initialized. Output dir: %s", record_output_dir_.c_str());
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
}

bool TrajectoryRecordController::stop(const std::string& mapping) {
    is_active_ = false;

    // ✅ 清理电机数据记录器
    if (motor_recorder_) {
        motor_recorder_->sync_to_disk();
        motor_recorder_.reset();  // 观察者会自动停止接收更新
        RCLCPP_INFO(node_->get_logger(), "Motor data recording stopped");
    }

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    // 清理资源
    recording_ = false;
    paused_ = false;
    active_mapping_.clear();

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
    std::string file_path = record_output_dir_ + "/" + msg->data + ".csv";

    // ✅ 创建新的 MotorDataRecorder 实例
    motor_recorder_ = std::make_shared<MotorDataRecorder>(file_path, active_mapping_);

    if (!motor_recorder_->is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open trajectory file: %s", file_path.c_str());
        motor_recorder_.reset();
        return;
    }

    // ✅ 简化方案：将 MotorDataRecorder 作为观察者注册到 HardwareManager
    // HardwareManager 本身实现了 MotorStatusObserver，我们让它转发给 motor_recorder_
    if (!hardware_manager_->register_motor_recorder(motor_recorder_)) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to register motor recorder observer");
        motor_recorder_.reset();
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "✅ Motor data recorder registered as observer");

    recording_ = true;
    paused_ = false;

    RCLCPP_INFO(node_->get_logger(), "✅ Started trajectory recording: %s", file_path.c_str());
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

// ✅ 虚方法实现 - 来自 TeachControllerBase 接口
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

    motor_recorder_.reset();  // 观察者会自动停止接收更新
    recording_ = false;
    paused_ = false;

    RCLCPP_INFO(node_->get_logger(), "✅ Trajectory recording cancelled (mapping: %s)", mapping.c_str());
}

void TrajectoryRecordController::complete(const std::string& mapping) {
    if (!recording_) return;

    motor_recorder_->sync_to_disk();
    motor_recorder_.reset();  // 观察者会自动停止接收更新
    recording_ = false;
    paused_ = false;

    RCLCPP_INFO(node_->get_logger(), "✅ Trajectory recording completed (mapping: %s)", mapping.c_str());
}
