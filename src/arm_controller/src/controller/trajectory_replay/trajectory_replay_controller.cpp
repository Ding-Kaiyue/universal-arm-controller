#include "trajectory_replay_controller.hpp"
#include "controller_interface.hpp"
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"
#include "arm_controller/hardware/motor_data_reloader.hpp"
#include "trajectory_segmenter.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include "arm_controller/ipc/controller_state_manager.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

TrajectoryReplayController::TrajectoryReplayController(const rclcpp::Node::SharedPtr& node)
    : TeachControllerBase("TrajectoryReplay", node) {

    hardware_manager_ = HardwareManager::getInstance();

    motor_data_reloader_ = std::make_unique<MotorDataReloader>(node);
    trajectory_segmenter_ = std::make_unique<TrajectorySegmenter>(100000);  // 100k点每段

    // 初始化轨迹插值器（仅用于规划到起点）
    trajectory_interpolator_ = std::make_unique<TrajectoryInterpolator>();
    load_interpolator_config(*trajectory_interpolator_);
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

    // ✅ 启动IPC命令队列消费线程
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::make_unique<std::thread>(&TrajectoryReplayController::command_queue_consumer_thread, this);
    }

    RCLCPP_INFO(node_->get_logger(), "TrajectoryReplayController initialized");
}

void TrajectoryReplayController::start(const std::string& mapping) {
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error("TrajectoryReplay: mapping not found");
    }

    // 调用基类 start() 设置 per-mapping 的 active_mappings_[mapping] = true
    ModeControllerBase::start(mapping);

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

    RCLCPP_INFO(node_->get_logger(), "TrajectoryReplayController activated. Input dir: %s", replay_dir_.c_str());
}

bool TrajectoryReplayController::stop(const std::string& mapping) {
    // 调用基类 start() 设置 per-mapping 的 active_mappings_[mapping] = false
    ModeControllerBase::stop(mapping);

    cancel();
    // disable_teaching_mode();
    
    cleanup_subscriptions(mapping);

    replaying_ = false;
    paused_ = false;

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
                // 与 MoveJ 对齐：使用 movej 配置路径，确保同一套速度/加速度参数与 joint limits 生效
                auto moveit_adapter = std::make_shared<trajectory_planning::infrastructure::integration::MoveItAdapter>(
                    node_, planning_group, "movej");

                if (!moveit_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Failed to create MoveItAdapter", mapping.c_str());
                    continue;
                }

                moveit_adapters_[mapping] = moveit_adapter;

                // 创建 TracIKAdapter
                auto tracik_adapter = std::make_shared<trajectory_planning::infrastructure::integration::TracIKAdapter>(
                    node_, planning_group);

                if (!tracik_adapter) {
                    RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Failed to create TracIKAdapter", mapping.c_str());
                    continue;
                }

                tracik_adapters_[mapping] = tracik_adapter;

                // 创建轨迹规划服务
                auto motion_planning_service = std::make_shared<trajectory_planning::application::services::MotionPlanningService>(
                    moveit_adapter,
                    tracik_adapter,
                    node_,
                    hardware_manager_->get_robot_type(mapping));

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

void TrajectoryReplayController::teach_callback(const controller_interfaces::msg::TeachingControl::SharedPtr msg) {
    std::string action = msg->action;
    std::string mapping = msg->mapping;
    std::string filename = msg->filename;

    RCLCPP_INFO(node_->get_logger(), "📨 TeachingControl: action=%s, mapping=%s, filename=%s",
                action.c_str(), mapping.c_str(), filename.c_str());

    if (action == "Start") {
        if (replaying_) {
            RCLCPP_WARN(node_->get_logger(), "❎ Already replaying, ignoring new command");
            return;
        }

        std::string file_path = replay_dir_ + "/" + filename + "_smooth.csv";

        if (!std::filesystem::exists(file_path)) {
            RCLCPP_ERROR(node_->get_logger(), "❎ File not found: %s", file_path.c_str());
            return;
        }

        // ✅ 记录启动的 mappings
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            replaying_mappings_.clear();

            if (mapping.empty() || mapping == "*") {
                // 展开 "*" 到所有可用 mappings
                for (const auto& m : hardware_manager_->get_all_mappings()) {
                    replaying_mappings_[m] = true;
                }
            } else {
                replaying_mappings_[mapping] = true;
            }
        }

        // 启动后台线程来加载和执行轨迹
        if (replay_thread_ && replay_thread_->joinable()) {
            replaying_ = false;
            replay_thread_->join();
        }

        replaying_ = true;
        paused_ = false;
        std::string replay_mapping = (mapping.empty() || mapping == "*") ? "*" : mapping;
        replay_thread_ = std::make_unique<std::thread>(&TrajectoryReplayController::replay_thread_func, this, file_path);
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

void TrajectoryReplayController::replay_thread_func(const std::string& file_path) {
    std::map<std::string, std::vector<double>> mapping_times;
    std::map<std::string, std::vector<std::vector<double>>> mapping_positions;
    std::map<std::string, std::vector<std::vector<double>>> mapping_velocities;
    // ✅ 删除：mapping_efforts - 改为动态计算重力补偿

    // 打开并解析CSV文件
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ Failed to open trajectory file: %s", file_path.c_str());
        replaying_ = false;
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

        // CSV解析：timestamp,mapping,position1-6,velocity1-6,effort1-6
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string token;

        while (std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            tokens.push_back(token);
        }

        if (tokens.size() < 20) continue;  // timestamp + mapping + 6*pos + 6*vel + 6*eff

        try {
            std::string interface = tokens[1];  // interface 名称（can0 或 can1）
            double timestamp = std::stod(tokens[0]);

            // ✅ 从 interface 获取对应的 mapping
            std::string mapping_name = hardware_manager_->get_mapping_by_interface(interface);
            if (mapping_name.empty()) {
                RCLCPP_WARN(node_->get_logger(), "⚠️ Failed to get mapping for interface %s, skipping this line", interface.c_str());
                continue;
            }

            mapping_times[mapping_name].push_back(timestamp);

            std::vector<double> pos(6), vel(6);
            for (int i = 0; i < 6; i++) {
                pos[i] = std::stod(tokens[2 + i]);
                vel[i] = std::stod(tokens[8 + i]);
                // ❌ 不再从文件读取 eff
                // ❌ eff[i] = std::stod(tokens[14 + i]);
            }
            mapping_positions[mapping_name].push_back(pos);
            mapping_velocities[mapping_name].push_back(vel);
            // ✅ eff 将在执行时动态计算
        } catch (...) {
            continue;
        }
    }
    file.close();

    if (mapping_positions.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ No valid trajectory data loaded from: %s", file_path.c_str());
        replaying_ = false;
        return;
    }

    // ✅ Step 0：排序并归一化时间基准
    // 确保所有 mapping 的数据按时间递增，且从 t=0 开始
    for (auto& [mapping_name, times] : mapping_times) {
        if (times.empty()) continue;

        // 计算排序索引
        std::vector<size_t> indices(times.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(),
            [&times](size_t a, size_t b) { return times[a] < times[b]; });

        // 应用排序到 positions, velocities, times
        auto& positions = mapping_positions[mapping_name];
        auto& velocities = mapping_velocities[mapping_name];

        std::vector<std::vector<double>> sorted_pos(positions.size());
        std::vector<std::vector<double>> sorted_vel(velocities.size());
        std::vector<double> sorted_times(times.size());

        for (size_t i = 0; i < indices.size(); i++) {
            sorted_pos[i] = positions[indices[i]];
            sorted_vel[i] = velocities[indices[i]];
            sorted_times[i] = times[indices[i]];
        }

        positions = sorted_pos;
        velocities = sorted_vel;
        times = sorted_times;

        // 归一化时间基准：所有 mapping 从 t=0 开始
        double t_min = times.front();
        for (auto& t : times) {
            t -= t_min;
        }

        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ Time normalized: 0 - %.3f seconds (%zu points)",
                    mapping_name.c_str(), times.back(), times.size());
    }

    // ✅ Phase 1：预计算所有 mapping 的分段信息和关节名称
    std::map<std::string, std::vector<TrajectorySegmenter::Segment>> all_segments;
    std::map<std::string, std::vector<std::string>> all_joint_names;

    for (auto& [mapping_name, positions] : mapping_positions) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (replaying_mappings_.find(mapping_name) == replaying_mappings_.end())
                continue;
        }

        size_t total_points = positions.size();
        auto segments = trajectory_segmenter_->compute_segments(total_points);
        auto joint_names = hardware_manager_->get_joint_names(mapping_name);

        all_segments[mapping_name] = segments;
        all_joint_names[mapping_name] = joint_names;

        RCLCPP_INFO(node_->get_logger(), "[%s] ▶️  Starting replay with %zu points in %zu segments",
                    mapping_name.c_str(), total_points, segments.size());
    }

    if (all_segments.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "❎ No mappings to replay");
        replaying_ = false;
        return;
    }

    // ✅ Phase 2：所有 mapping 并行移动到起点
    RCLCPP_INFO(node_->get_logger(), "▶️ Moving all mappings to start point...");
    for (auto& [mapping_name, positions] : mapping_positions) {
        if (all_segments.find(mapping_name) == all_segments.end()) continue;
        move_to_start_point(positions[0], mapping_name);
    }
    // 等所有 mapping 都到达起点
    for (auto& [mapping_name, segments] : all_segments) {
        hardware_manager_->wait_for_trajectory_completion(mapping_name);
    }
    RCLCPP_INFO(node_->get_logger(), "✅ All mappings reached start point");

    // ✅ Phase 3：逐段并行执行
    RCLCPP_INFO(node_->get_logger(), "▶️ Starting parallel segment execution...");

    // 计算最多段数
    size_t max_segments = 0;
    for (auto& [m, segs] : all_segments) {
        max_segments = std::max(max_segments, segs.size());
    }

    // 外层：按段数迭代
    for (size_t seg_idx = 0; seg_idx < max_segments && replaying_; ++seg_idx) {
        // 检查暂停
        while (paused_ && replaying_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!replaying_) break;

        // 为所有 mapping 构建并异步提交当前段轨迹（不阻塞）
        for (auto& [mapping_name, positions] : mapping_positions) {
            auto& segments = all_segments[mapping_name];
            if (seg_idx >= segments.size()) continue;

            auto& times = mapping_times[mapping_name];
            auto& joint_names = all_joint_names[mapping_name];
            const auto& segment = segments[seg_idx];

            RCLCPP_INFO(node_->get_logger(), "[%s] Processing segment %zu/%zu (%zu-%zu, %zu points)",
                        mapping_name.c_str(), seg_idx + 1, segments.size(), segment.start_idx, segment.end_idx - 1, segment.point_count);

            // 提取当前分段的数据
            std::vector<double> segment_times(times.begin() + segment.start_idx,
                                             times.begin() + segment.end_idx);
            std::vector<std::vector<double>> segment_positions(positions.begin() + segment.start_idx,
                                                               positions.begin() + segment.end_idx);

            // ✅ 使用原始位置/时间（来自平滑后的CSV），速度由位置差分重算
            std::vector<double> fixed_times;
            std::vector<std::vector<double>> resampled_positions;
            std::vector<std::vector<double>> resampled_velocities;

            double t0 = segment_times.front();
            for (size_t i = 0; i < segment_positions.size(); ++i) {
                fixed_times.push_back(segment_times[i] - t0);
                resampled_positions.push_back(segment_positions[i]);
            }

            // 由平滑后位置和时间差重算速度，确保 position/velocity 一致，减少回放抖动
            if (!resampled_positions.empty()) {
                const size_t dof = resampled_positions.front().size();
                resampled_velocities.assign(resampled_positions.size(), std::vector<double>(dof, 0.0));

                if (resampled_positions.size() >= 3) {
                    for (size_t i = 1; i + 1 < resampled_positions.size(); ++i) {
                        const double dt = fixed_times[i + 1] - fixed_times[i - 1];
                        if (dt <= 1e-9) continue;
                        for (size_t j = 0; j < dof; ++j) {
                            resampled_velocities[i][j] =
                                (resampled_positions[i + 1][j] - resampled_positions[i - 1][j]) / dt;
                        }
                    }
                }

                // 对差分速度做轻微平滑，避免高频毛刺
                if (resampled_velocities.size() >= 3) {
                    auto vel_filtered = resampled_velocities;
                    for (size_t i = 1; i + 1 < resampled_velocities.size(); ++i) {
                        for (size_t j = 0; j < dof; ++j) {
                            vel_filtered[i][j] =
                                0.25 * resampled_velocities[i - 1][j] +
                                0.50 * resampled_velocities[i][j] +
                                0.25 * resampled_velocities[i + 1][j];
                        }
                    }
                    resampled_velocities.swap(vel_filtered);
                }
            }

            // 非首段需删除第一个点（避免重复）
            if (!segment.is_first) {
                if (!resampled_positions.empty()) {
                    fixed_times.erase(fixed_times.begin());
                    resampled_positions.erase(resampled_positions.begin());
                    resampled_velocities.erase(resampled_velocities.begin());
                }
            }

            if (resampled_positions.empty()) {
                continue;
            }

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

            // ✅ 异步提交轨迹，不阻塞
            RCLCPP_INFO(node_->get_logger(), "[%s] ▶️  Submitting segment %zu/%zu", mapping_name.c_str(), seg_idx + 1, segments.size());
            execute_trajectory(traj, mapping_name);
        }

        // ✅ 等所有 mapping 当前段完成后，再进入下一段
        for (auto& [mapping_name, segments] : all_segments) {
            if (seg_idx < segments.size()) {
                hardware_manager_->wait_for_trajectory_completion(mapping_name);
            }
        }

        if (!replaying_) break;
    }

    replaying_ = false;
    {
        std::lock_guard<std::mutex> lock(execution_mutex_);
        execution_ids_.clear();
    }
    RCLCPP_INFO(node_->get_logger(), "✅ Overall replay completed");
}


void TrajectoryReplayController::execute_trajectory(
    const trajectory_interpolator::Trajectory& trajectory,
    const std::string& mapping) {

    // ✅ 转换为hardware_driver格式
    Trajectory hw_trajectory = arm_controller::utils::TrajectoryConverter::convertInterpolatorToHardwareDriver(trajectory);

    // ✅ 为每个轨迹点动态计算重力补偿力矩
    for (auto& point : hw_trajectory.points) {
        // 计算当前位置的重力补偿
        auto gravity_torques = hardware_manager_->compute_gravity_torques(mapping, point.positions);
        point.efforts = gravity_torques;
    }

    // ✅ 执行轨迹
    try {
        std::string exec_id = hardware_manager_->execute_trajectory_async(
            mapping, hw_trajectory, true);

        if (exec_id.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Failed to execute trajectory", mapping.c_str());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(execution_mutex_);
            execution_ids_[mapping] = exec_id;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] ❎ TrajectoryReplay: Exception during trajectory execution: %s", mapping.c_str(), e.what());
        return;
    }
}

void TrajectoryReplayController::pause() {
    if (!replaying_ || paused_) return;
    paused_ = true;

    // ✅ 为启动的 mappings 暂停轨迹
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const auto& [mapping, _] : replaying_mappings_) {
            hardware_manager_->pause_trajectory(mapping);
        }
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Paused");
}

void TrajectoryReplayController::resume() {
    if (!replaying_ || !paused_) return;
    paused_ = false;

    // ✅ 为启动的 mappings 恢复轨迹
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const auto& [mapping, _] : replaying_mappings_) {
            hardware_manager_->resume_trajectory(mapping);
        }
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Resumed");
}

void TrajectoryReplayController::cancel() {
    replaying_ = false;

    // ✅ 为启动的 mappings 取消轨迹
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const auto& [mapping, _] : replaying_mappings_) {
            hardware_manager_->cancel_trajectory(mapping);

            {
                std::lock_guard<std::mutex> exec_lock(execution_mutex_);
                execution_ids_.erase(mapping);
            }
        }
        replaying_mappings_.clear();
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Cancelled");
}

void TrajectoryReplayController::complete() {
    replaying_ = false;

    // ✅ 清除回放状态
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        replaying_mappings_.clear();
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Completed");
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

// ============ IPC 接口实现 ============

bool TrajectoryReplayController::execute(const std::string& mapping, const std::string& command, const std::string& filename) {
    // mapping: 具体的映射名
    // command: start、pause、resume、cancel、complete
    // filename: 要操作的回放文件名

    if (command == "start") {
        std::string file_path = replay_dir_ + "/" + filename + "_smooth.csv";

        if (!std::filesystem::exists(file_path)) {
            RCLCPP_ERROR(node_->get_logger(), "❎ File not found: %s", file_path.c_str());
            return false;
        }

        // 检查是否已在回放中
        if (replaying_) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Already replaying, ignoring new command");
            return false;
        }

        // ✅ 记录启动的 mapping
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            replaying_mappings_[mapping] = true;
        }

        replaying_ = true;
        paused_ = false;

        // 启动后台回放线程
        if (replay_thread_ && replay_thread_->joinable()) {
            replaying_ = false;
            replay_thread_->join();
        }

        replay_thread_ = std::make_unique<std::thread>([this, file_path, mapping]() {
            this->replay_thread_func(file_path);
        });

        RCLCPP_INFO(node_->get_logger(), "✅ Started replay: %s", file_path.c_str());
        return true;

    } else if (command == "pause") {
        pause();
        return true;

    } else if (command == "resume") {
        resume();
        return true;

    } else if (command == "cancel") {
        cancel();
        return true;

    } else if (command == "complete") {
        complete();
        return true;

    } else {
        RCLCPP_WARN(node_->get_logger(), "❎ Unknown command: %s", command.c_str());
        return false;
    }

}
