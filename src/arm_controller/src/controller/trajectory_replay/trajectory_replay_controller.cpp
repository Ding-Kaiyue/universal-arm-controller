#include "trajectory_replay_controller.hpp"
#include "controller_interface.hpp"
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"
#include "arm_controller/hardware/motor_data_reloader.hpp"
#include "trajectory_segmenter.hpp"
#include <filesystem>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>

TrajectoryReplayController::TrajectoryReplayController(const rclcpp::Node::SharedPtr& node)
    : TeachControllerBase("TrajectoryReplay", node) {

    hardware_manager_ = HardwareManager::getInstance();

    motor_data_reloader_ = std::make_unique<MotorDataReloader>(node);
    trajectory_segmenter_ = std::make_unique<TrajectorySegmenter>(100000);  // 100k点每段

    // 初始化轨迹插值器（仅用于规划到起点）
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();

    // ✅ 加载插值器配置
    load_interpolator_config(*trajectory_interpolator_);

    // 初始化轨迹规划服务
    initialize_planning_services();

    // 获取轨迹存储目录
    try {
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("arm_controller");
        std::filesystem::path workspace_root =
            std::filesystem::path(pkg_dir).parent_path().parent_path().parent_path();
        replay_dir_ = (workspace_root / "trajectories").string();
    } catch (const std::exception& e) {
        replay_dir_ = "/tmp/arm_recording_trajectories";
    }

    RCLCPP_INFO(node_->get_logger(), "TrajectoryReplayController initialized");
}

void TrajectoryReplayController::start(const std::string& mapping) {
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error("TrajectoryReplay: mapping not found");
    }

    active_mapping_ = mapping.empty() ? "single_arm" : mapping;
    is_active_ = true;

    replaying_ = false;
    paused_ = false;

    // 启用示教模式 - 防止安全限位检查触发急停
    // enable_teaching_mode();

    if (subscriptions_.find(mapping) == subscriptions_.end()) {
        init_subscriptions(mapping);
    }

    auto hardware_driver = hardware_manager_->get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Hardware driver not initialized");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] TrajectoryReplayController activated", active_mapping_.c_str());
}

bool TrajectoryReplayController::stop(const std::string& mapping) {
    is_active_ = false;
    replaying_ = false;
    paused_ = false;

    // 取消当前执行的轨迹
    if (!current_execution_id_.empty()) {
        hardware_manager_->cancel_trajectory(mapping);
        current_execution_id_.clear();
    }

    // 等待回放线程完成
    if (replay_thread_ && replay_thread_->joinable()) {
        replay_thread_->join();
    }

    // disable_teaching_mode();
    
    cleanup_subscriptions(mapping);
    active_mapping_.clear();

    RCLCPP_INFO(node_->get_logger(), "[%s] TrajectoryReplayController deactivated", mapping.c_str());
    return true;
}

void TrajectoryReplayController::initialize_planning_services() {
    try {
        // 获取所有mapping
        auto all_mappings = hardware_manager_->get_all_mappings();
        if (all_mappings.empty()) {
            RCLCPP_WARN(node_->get_logger(), "❎ TrajectoryReplay: No mappings configured");
            return;
        }

        // 为每个mapping初始化规划服务
        for (const auto& mapping : all_mappings) {
            std::string planning_group = hardware_manager_->get_planning_group(mapping);

            if (planning_group.empty()) {
                RCLCPP_WARN(node_->get_logger(), "[%s] ❎ TrajectoryReplay: No planning group configured, skipping...", mapping.c_str());
                continue;
            }

            try {
                // 创建 MoveItAdapter
                auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, planning_group);

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;

                // 创建轨迹规划服务
                auto motion_planning_service = std::make_shared<trajectory_planning::application::services::MotionPlanningService>(
                    moveit_adapter,
                    node_);

                if (!motion_planning_service) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Failed to create MotionPlanningService", mapping.c_str());
                    continue;
                }

                motion_planning_service->registerMoveJStrategy();

                motion_planning_services_[mapping] = motion_planning_service;
                mapping_to_planning_group_[mapping] = planning_group;

                RCLCPP_INFO(node_->get_logger(), "[%s] ✅ TrajectoryReplay: Planning service initialized", mapping.c_str());

            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Failed to initialize planning services: %s", mapping.c_str(), e.what());
                continue;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❎ TrajectoryReplay: Failed to initialize planning services: %s", e.what());
    }
}

void TrajectoryReplayController::teach_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_ || msg->data.empty()) return;

    std::string file_path = replay_dir_ + "/" + msg->data + ".csv";

    if (!std::filesystem::exists(file_path)) {
        RCLCPP_ERROR(node_->get_logger(), "❎ File not found: %s", file_path.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Loading trajectory: %s", file_path.c_str());

    // 启动后台线程来加载、平滑和执行轨迹
    if (replay_thread_ && replay_thread_->joinable()) {
        replaying_ = false;
        replay_thread_->join();
    }

    replaying_ = true;
    paused_ = false;
    replay_thread_ = std::make_unique<std::thread>([this, file_path]() {
        this->replay_thread_func(file_path);
    });
}

void TrajectoryReplayController::replay_thread_func(const std::string& file_path) {
    // ✅ 使用新的模块化组件进行分段加载和执行
    std::vector<double> all_times;
    std::vector<std::vector<double>> all_positions;
    std::vector<std::vector<double>> all_velocities;
    std::vector<std::vector<double>> all_efforts;

    // 使用 MotorDataReloader 加载 CSV 文件
    if (!motor_data_reloader_->load_trajectory_from_csv(file_path, all_times, all_positions,
                                                        all_velocities, all_efforts)) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to load trajectory from CSV");
        replaying_ = false;
        return;
    }

    auto joint_names = hardware_manager_->get_joint_names(active_mapping_);
    size_t total_points = all_positions.size();

    // 使用 TrajectorySegmenter 计算分段信息
    auto segments = trajectory_segmenter_->compute_segments(total_points);
    RCLCPP_INFO(node_->get_logger(), "Will execute in %zu segments", segments.size());

    for (size_t seg = 0; seg < segments.size() && replaying_; ++seg) {
        const auto& segment = segments[seg];

        RCLCPP_INFO(node_->get_logger(), "Processing segment %zu/%zu (%zu-%zu, %zu points)",
                    seg + 1, segments.size(), segment.start_idx, segment.end_idx - 1, segment.point_count);

        // 提取当前分段的数据
        std::vector<double> segment_times(all_times.begin() + segment.start_idx,
                                         all_times.begin() + segment.end_idx);
        std::vector<std::vector<double>> segment_positions(all_positions.begin() + segment.start_idx,
                                                           all_positions.begin() + segment.end_idx);
        std::vector<std::vector<double>> segment_velocities(all_velocities.begin() + segment.start_idx,
                                                            all_velocities.begin() + segment.end_idx);
        std::vector<std::vector<double>> segment_efforts(all_efforts.begin() + segment.start_idx,
                                                         all_efforts.begin() + segment.end_idx);

        // ✅ 转换为固定时间间隔的轨迹
        
        std::vector<double> fixed_times;
        std::vector<std::vector<double>> resampled_positions;
        std::vector<std::vector<double>> resampled_velocities;
        std::vector<std::vector<double>> resampled_efforts;

        for (size_t i = 0; i < segment_positions.size(); ++i) {
            fixed_times.push_back(i * TIME_STEP);
            resampled_positions.push_back(segment_positions[i]);
            resampled_velocities.push_back(segment_velocities[i]);
            resampled_efforts.push_back(segment_efforts[i]);
        }

        // 跳过第一个点（除了第一个分段）
        if (segment.is_first) {
            // 第一个分段：先移动到起点
            move_to_start_point(all_positions[0], active_mapping_);
            RCLCPP_INFO(node_->get_logger(), "✅ Waiting for reach start point...");
            hardware_manager_->wait_for_trajectory_completion(active_mapping_);
        } else {
            // 后续分段：删除第一个点避免重复
            if (!resampled_positions.empty()) {
                fixed_times.erase(fixed_times.begin());
                resampled_positions.erase(resampled_positions.begin());
                resampled_velocities.erase(resampled_velocities.begin());
                resampled_efforts.erase(resampled_efforts.begin());
            }
        }

        if (resampled_positions.empty()) {
            continue;
        }

        // ✅ 直接使用已记录的轨迹数据（在TrajectoryRecord时已通过CSAPS平滑处理）
        // 不需要额外的平滑和插值，直接转换为轨迹对象执行

        // 执行当前分段
        RCLCPP_INFO(node_->get_logger(), "▶️  Executing segment %zu/%zu", seg + 1, segments.size());

        // 构建轨迹对象
        trajectory_interpolator::Trajectory traj;
        traj.joint_names = joint_names;
        traj.points.reserve(resampled_positions.size());

        for (size_t i = 0; i < resampled_positions.size(); i++) {
            trajectory_interpolator::TrajectoryPoint point;
            point.time_from_start = fixed_times[i];
            point.positions = resampled_positions[i];
            point.velocities = resampled_velocities[i];
            point.accelerations.resize(resampled_positions[i].size(), 0.0);
            traj.points.push_back(point);
        }

        execute_trajectory(traj, active_mapping_);
        hardware_manager_->wait_for_trajectory_completion(active_mapping_);

        if (!replaying_) {
            RCLCPP_INFO(node_->get_logger(), "⏸️  Replay cancelled by user");
            break;
        }
    }

    replaying_ = false;
    current_execution_id_.clear();
    RCLCPP_INFO(node_->get_logger(), "✅ Replay completed");
}


void TrajectoryReplayController::execute_trajectory(
    const trajectory_interpolator::Trajectory& trajectory,
    const std::string& mapping) {

    // ✅ 转换为hardware_driver格式
    Trajectory hw_trajectory = arm_controller::utils::TrajectoryConverter::convertInterpolatorToHardwareDriver(trajectory);
    // ✅ 执行轨迹
    try {
        current_execution_id_ = hardware_manager_->execute_trajectory_async(
            mapping, hw_trajectory, true);

        if (current_execution_id_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Failed to execute trajectory", mapping.c_str());
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ TrajectoryReplay: Trajectory execution started (ID: %s)", 
                    mapping.c_str(), current_execution_id_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Exception during trajectory execution: %s", mapping.c_str(), e.what());
        return;
    }
}

void TrajectoryReplayController::on_teaching_control(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_active_ || msg->data.empty()) return;

    if (msg->data == "pause") {
        pause(active_mapping_);
    } else if (msg->data == "resume") {
        resume(active_mapping_);
    } else if (msg->data == "cancel") {
        cancel(active_mapping_);
    }
}

void TrajectoryReplayController::pause(const std::string& mapping) {
    if (!replaying_) return;
    paused_ = true;
    hardware_manager_->pause_trajectory(mapping);
    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Paused", mapping.c_str());
}

void TrajectoryReplayController::resume(const std::string& mapping) {
    if (!replaying_ || !paused_) return;
    paused_ = false;
    hardware_manager_->resume_trajectory(mapping);
    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Resumed", mapping.c_str());
}

void TrajectoryReplayController::cancel(const std::string& mapping) {
    replaying_ = false;
    hardware_manager_->cancel_trajectory(mapping);
    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Cancelled", mapping.c_str());
}

void TrajectoryReplayController::complete(const std::string& mapping) {
    (void) mapping;
    RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Completed", mapping.c_str());
}

void TrajectoryReplayController::move_to_start_point(const std::vector<double>& start_position, const std::string& mapping) {
    // 查找mapping
    if (motion_planning_services_.find(mapping) == motion_planning_services_.end() ||
        !motion_planning_services_[mapping]) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Planning service not found. This strategy must be registered first.", mapping.c_str());
        return;
    }

    // 检查目标关节状态
    sensor_msgs::msg::JointState target_state;

    // ✅ CSV 数据是度数，需要转换为弧度供 MoveIt 使用
    target_state.position.reserve(start_position.size());
    for (double deg : start_position) {
        target_state.position.push_back(deg * M_PI / 180.0);  // deg to rad
    }

    if (target_state.position.size() != hardware_manager_->get_joint_count(mapping)) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Invalid start position size", mapping.c_str());
        return;
    }

    // 进行轨迹规划
    auto planning_result = motion_planning_services_[mapping]->planJointMotion(target_state);
    if (!planning_result.success) {
        return;
    }

    // 检查轨迹点数
    if (planning_result.trajectory.size() < 3) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ TrajectoryReplay: Already at start position, no movement needed",
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

trajectory_interpolator::Trajectory TrajectoryReplayController::interpolate_trajectory(
    const trajectory_interpolator::Trajectory& interpolator_trajectory,
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    const std::string& mapping) {

    if (!trajectory_interpolator_) {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] ⚠️ TrajectoryReplay: No interpolator available, using original trajectory", mapping.c_str());
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
            RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ TrajectoryReplay: Interpolation failed, using original trajectory", mapping.c_str());
            return interpolator_trajectory;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "[%s] ⚠️ TrajectoryReplay: Interpolation exception: %s, using original trajectory",
                   mapping.c_str(), e.what());
        return interpolator_trajectory;
    }
}

