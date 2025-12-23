#include "point_record_controller.hpp"
#include "controller_interface.hpp"
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>

PointRecordController::PointRecordController(const rclcpp::Node::SharedPtr & node)
    : TeachControllerBase("PointRecord", node)
{
    // 获取硬件管理器实例
    hardware_manager_ = HardwareManager::getInstance();

    /* ---------- record directory ---------- */
    try {
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("arm_controller");
        std::filesystem::path workspace_root =
            std::filesystem::path(pkg_dir).parent_path().parent_path().parent_path();

        record_output_dir_ = (workspace_root / "points").string();

        std::filesystem::create_directories(record_output_dir_);
    } catch (const std::exception& e) {
        record_output_dir_ = "/tmp/arm_recording_points";
        std::filesystem::create_directories(record_output_dir_);
        fprintf(stderr, "⚠️  Fallback to: %s\n", record_output_dir_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "PointRecordController initialized. Output dir: %s", record_output_dir_.c_str());
}


void PointRecordController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] PointRecord: not found in hardware configuration."
        );
    }

    // 保存当前激活的 mapping
    active_mapping_ = mapping.empty() ? "single_arm" : mapping;
    is_active_ = true;

    RCLCPP_INFO(node_->get_logger(), "[%s] PointRecordController activated",
                active_mapping_.c_str());

    // 在激活时创建话题订阅（如果还没创建的话）
    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }
}

bool PointRecordController::stop(const std::string& mapping) {
    is_active_ = false;

    // ✅ 清理电机数据记录器
    if (motor_recorder_) {
        motor_recorder_->sync_to_disk();
        motor_recorder_.reset();
    }

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    // 清理资源
    active_mapping_.clear();
    current_point_file_path_.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] PointRecordController deactivated",
                mapping.c_str());
    return true;
}


void PointRecordController::teach_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_ || msg->data.empty()) return;

    /* -------- record current point -------- */
    current_point_file_path_ = record_output_dir_ + "/" + msg->data + ".csv";

    // 获取该mapping的所有电机ID
    const auto& motor_ids = hardware_manager_->get_motors_id(active_mapping_);
    if (motor_ids.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ No motors found for mapping: %s", active_mapping_.c_str());
        return;
    }

    // 创建新的 MotorDataRecorder 实例，传递电机ID列表（用于单数据模式）
    motor_recorder_ = std::make_shared<MotorDataRecorder>(
        current_point_file_path_, active_mapping_, motor_ids);

    if (!motor_recorder_->is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open point file: %s", current_point_file_path_.c_str());
        motor_recorder_.reset();
        return;
    }

    // 启用单数据模式：记录所有电机各一条数据后自动暂停
    motor_recorder_->set_single_record_mode(true);

    // 将 MotorDataRecorder 作为观察者注册到 HardwareManager
    if (!hardware_manager_->register_motor_recorder(motor_recorder_)) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to register motor recorder observer");
        motor_recorder_.reset();
        return;
    }

    // 由于启用了单数据模式，记录器会在所有电机各记录一条数据后自动暂停
    // 稍微等待一下确保数据被写入和暂停
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 记录完成，同步到磁盘并清理
    if (motor_recorder_) {
        motor_recorder_->sync_to_disk();
        motor_recorder_.reset();
    }

    current_point_file_path_.clear();
    RCLCPP_INFO(node_->get_logger(), "✅ Point recorded: %s", msg->data.c_str());
}

void PointRecordController::on_teaching_control(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_ || msg->data.empty()) return;

    const std::string command = msg->data;

    if (command == "pause") {
        pause(active_mapping_);
    } else if (command == "resume") {
        resume(active_mapping_);
    } else if (command == "cancel") {
        cancel(active_mapping_);
    } else if (command == "complete") {
        complete(active_mapping_);
    } else {
        RCLCPP_WARN(node_->get_logger(), "❎ Unknown teaching control command: %s", command.c_str());
    }
}

// 虚方法实现
void PointRecordController::pause(const std::string& mapping) {
    // Point Record 不支持暂停，因为每个点都是独立完成的
    (void) mapping;
}

void PointRecordController::resume(const std::string& mapping) {
    // Point Record 不支持恢复，因为每个点都是独立完成的
    (void) mapping;
}

void PointRecordController::cancel(const std::string& mapping) {
    motor_recorder_.reset();

    // 删除已记录的点文件
    if (!current_point_file_path_.empty()) {
        try {
            std::filesystem::remove(current_point_file_path_);
            RCLCPP_INFO(node_->get_logger(), "✅ Point recording cancelled and file deleted: %s", mapping.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "⚠️  Failed to delete point file: %s", e.what());
        }
        current_point_file_path_.clear();
    }
}

void PointRecordController::complete(const std::string& mapping) {
    // Point Record 不需要显式完成，因为 teach_callback 已经立即完成了记录
    (void) mapping;

    // 如果还有未清理的文件路径，清理它
    if (!current_point_file_path_.empty()) {
        current_point_file_path_.clear();
    }
}
