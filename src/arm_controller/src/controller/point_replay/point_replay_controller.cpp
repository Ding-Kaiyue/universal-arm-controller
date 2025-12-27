#include "point_replay_controller.hpp"
#include "controller_interface.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>

PointReplayController::PointReplayController(const rclcpp::Node::SharedPtr& node)
    : TeachControllerBase("PointReplay", node)
{
    // 获取硬件管理器实例
    hardware_manager_ = HardwareManager::getInstance();

    // 初始化轨迹插值器
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

    // ✅ 加载插值器配置
    load_interpolator_config(*trajectory_interpolator_);

    // 初始化轨迹规划服务
    initialize_planning_services();

    /* ---------- replay directory ---------- */
    try {
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("arm_controller");
        std::filesystem::path workspace_root =
            std::filesystem::path(pkg_dir).parent_path().parent_path().parent_path();

        replay_dir_ = (workspace_root / "points").string();

    } catch (const std::exception& e) {
        replay_dir_ = "/tmp/arm_recording_points";
        fprintf(stderr, "⚠️  Fallback to: %s\n", replay_dir_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "PointReplayController initialized. Output dir: %s", replay_dir_.c_str());
}

void PointReplayController::start(const std::string& mapping) {
    // 检查 mapping 是否存在于配置中
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] PointReplay: not found in hardware configuration."
        );
    }

    // 保存当前激活的 mapping
    active_mapping_ = mapping.empty() ? "single_arm" : mapping;
    is_active_ = true;

    // 启用示教模式 - 防止安全限位检查触发急停
    enable_teaching_mode();

    RCLCPP_INFO(node_->get_logger(), "[%s] PointReplayController activated",
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
}

bool PointReplayController::stop(const std::string& mapping) {
    is_active_ = false;

    // 取消当前执行的轨迹
    if (!current_execution_id_.empty()) {
        hardware_manager_->cancel_trajectory(mapping);
        current_execution_id_.clear();
    }

    disable_teaching_mode();

    cleanup_subscriptions(mapping);
    active_mapping_.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] PointReplayController deactivated", mapping.c_str());
    return true;
}

void PointReplayController::initialize_planning_services() {
    try {
        // 获取所有mapping
        auto all_mappings = hardware_manager_->get_all_mappings();
        if (all_mappings.empty()) {
            RCLCPP_WARN(node_->get_logger(), "❎ PointReplay: No mappings configured");
            return;
        }

        // 为每个mapping初始化规划服务
        for (const auto& mapping : all_mappings) {
            std::string planning_group = hardware_manager_->get_planning_group(mapping);

            if (planning_group.empty()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ❎ PointReplay: No planning group configured, skipping...", mapping.c_str());
                continue;
            }

            try {
                // 创建 MoveItAdapter
                auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, planning_group);

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ PointReplay: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;

                // 创建轨迹规划服务
                auto motion_planning_service = std::make_shared<trajectory_planning::application::services::MotionPlanningService>(
                    moveit_adapter,
                    node_);

                if (!motion_planning_service) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ PointReplay: Failed to create MotionPlanningService", mapping.c_str());
                    continue;
                }

                motion_planning_service->registerMoveJStrategy();

                motion_planning_services_[mapping] = motion_planning_service;
                mapping_to_planning_group_[mapping] = planning_group;

                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ PointReplay: Planning service initialized", mapping.c_str());

            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ PointReplay: Failed to initialize planning services: %s", mapping.c_str(), e.what());
                continue;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ PointReplay: Failed to initialize planning services: %s", e.what());
    }
}

void PointReplayController::teach_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_ || msg->data.empty()) return;

    std::string file_path = replay_dir_ + "/" + msg->data + ".csv";

    if (!std::filesystem::exists(file_path)) {
        RCLCPP_ERROR(node_->get_logger(), "❎ File not found: %s", file_path.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Loading point: %s", file_path.c_str());

    // 直接在当前线程中执行（点回放不需要后台线程，因为只有一个点）
    replay_point(file_path);
}

void PointReplayController::replay_point(const std::string& file_path) {
    // 打开CSV文件并读取第一个数据行（只回放一个点）
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open file: %s", file_path.c_str());
        return;
    }

    std::string line;
    std::vector<double> point_position;

    // 跳过CSV头部（如果有）
    if (std::getline(file, line)) {
        // 检查第一行是否是头部
        if (line.find("time") != std::string::npos || line.find("joint") != std::string::npos) {
            // 这是头部，继续读取数据行
            if (!std::getline(file, line)) {
                RCLCPP_ERROR(node_->get_logger(), "❎ No data in file");
                file.close();
                return;
            }
        }
        // 否则第一行就是数据

        // 解析CSV行 - 格式: time,joint1,joint2,...,jointN
        std::istringstream iss(line);
        std::string value;
        std::vector<std::string> values;

        while (std::getline(iss, value, ',')) {
            values.push_back(value);
        }

        if (values.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Invalid CSV format");
            file.close();
            return;
        }

        // 第一列是时间，跳过；后面是关节位置
        for (size_t i = 1; i < values.size(); ++i) {
            try {
                point_position.push_back(std::stod(values[i]));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "❎ Failed to parse joint position: %s", e.what());
                file.close();
                return;
            }
        }
    }

    file.close();

    if (point_position.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ No joint positions found in file");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Loaded point with %zu joints", point_position.size());

    // 检查关节数是否匹配
    size_t expected_joints = hardware_manager_->get_joint_count(active_mapping_);
    if (point_position.size() != expected_joints) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Joint count mismatch: expected %zu, got %zu",
                    expected_joints, point_position.size());
        return;
    }

    // 移动到目标点
    move_to_point(point_position, active_mapping_);
}

void PointReplayController::move_to_point(const std::vector<double>& target_position, const std::string& mapping) {
    // 查找mapping对应的规划服务
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() ||
        !motion_planning_services_[mapping]) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ❎ PointReplay: Planning service not found", mapping.c_str());
        return;
    }

    // 创建目标关节状态
    sensor_msgs::msg::JointState target_state;

    // ✅ CSV 数据是度数，需要转换为弧度供 MoveIt 使用
    target_state.position.reserve(target_position.size());
    for (double deg : target_position) {
        target_state.position.push_back(deg * M_PI / 180.0);  // deg to rad
    }

    // 进行轨迹规划
    auto planning_result = motion_planning_services_[mapping]->planJointMotion(target_state);
    if (!planning_result.success) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ PointReplay: Failed to plan trajectory to target point", mapping.c_str());
        return;
    }

    // 检查轨迹点数
    if (planning_result.trajectory.size() < 3) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ PointReplay: Already at target position, no movement needed",
                    mapping.c_str());
        return;
    }

    // 转换轨迹格式
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    trajectory_interpolator::Trajectory interpolator_trajectory =
        arm_controller::utils::TrajectoryConverter::convertPlanningToInterpolator(
            planning_result.trajectory, joint_names);

    // 分析轨迹动力学参数
    auto dynamics = arm_controller::utils::TrajectoryConverter::analyzeTrajectoryDynamics(planning_result.trajectory);
    auto safe_params = arm_controller::utils::TrajectoryConverter::calculateSafeInterpolationParams(dynamics);

    // 插值轨迹
    trajectory_interpolator::Trajectory final_trajectory = interpolate_trajectory(
        interpolator_trajectory,
        safe_params.max_velocity,
        safe_params.max_acceleration,
        safe_params.max_jerk,
        mapping
    );

    // 执行轨迹
    execute_trajectory(final_trajectory, mapping);
}

void PointReplayController::execute_trajectory(
    const trajectory_interpolator::Trajectory& trajectory,
    const std::string& mapping) {

    // ✅ 转换为hardware_driver格式
    Trajectory hw_trajectory = arm_controller::utils::TrajectoryConverter::convertInterpolatorToHardwareDriver(trajectory);
    // ✅ 执行轨迹
    try {
        current_execution_id_ = hardware_manager_->execute_trajectory_async(
            mapping, hw_trajectory, true);

        if (current_execution_id_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ PointReplay: Failed to execute trajectory", mapping.c_str());
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ PointReplay: Trajectory execution started (ID: %s)",
                    mapping.c_str(), current_execution_id_.c_str());

        // 等待执行完成
        hardware_manager_->wait_for_trajectory_completion(mapping);
        current_execution_id_.clear();

        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ PointReplay: Point reached successfully", mapping.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ PointReplay: Exception during trajectory execution: %s", mapping.c_str(), e.what());
        return;
    }
}

trajectory_interpolator::Trajectory PointReplayController::interpolate_trajectory(
    const trajectory_interpolator::Trajectory& interpolator_trajectory,
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    const std::string& mapping) {

    if (!trajectory_interpolator_) {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] ⚠️ PointReplay: No interpolator available, using original trajectory", mapping.c_str());
        return interpolator_trajectory;
    }

    try {
        // 使用提供的动力学参数加载轨迹
        if (trajectory_interpolator_->loadTrajectoryWithDynamicConfig(interpolator_trajectory,
                                                                     max_velocity,
                                                                     max_acceleration,
                                                                     max_jerk)) {
            return trajectory_interpolator_->interpolate();
        } else {
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ PointReplay: Interpolation failed, using original trajectory", mapping.c_str());
            return interpolator_trajectory;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ PointReplay: Interpolation exception: %s, using original trajectory",
                   mapping.c_str(), e.what());
        return interpolator_trajectory;
    }
}

void PointReplayController::on_teaching_control(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_ || msg->data.empty()) return;

    if (msg->data == "cancel") {
        cancel(active_mapping_);
    }
}

void PointReplayController::pause(const std::string& mapping) {
    if (current_execution_id_.empty()) return;
    hardware_manager_->pause_trajectory(mapping);
    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Paused", mapping.c_str());
}

void PointReplayController::resume(const std::string& mapping) {
    if (current_execution_id_.empty()) return;
    hardware_manager_->resume_trajectory(mapping);
    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Resumed", mapping.c_str());
}

void PointReplayController::cancel(const std::string& mapping) {
    if (current_execution_id_.empty()) return;
    hardware_manager_->cancel_trajectory(mapping);
    current_execution_id_.clear();
    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Cancelled", mapping.c_str());
}

void PointReplayController::complete(const std::string& mapping) {
    (void) mapping;
    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Completed", mapping.c_str());
}
