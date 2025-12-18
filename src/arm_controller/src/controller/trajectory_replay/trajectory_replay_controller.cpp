#include "trajectory_replay_controller.hpp"
#include "controller_interface.hpp"
#include <filesystem>
#include <thread>
#include <future>
#include <ament_index_cpp/get_package_share_directory.hpp>

TrajectoryReplayController::TrajectoryReplayController(const rclcpp::Node::SharedPtr & node)
    : TeachControllerBase("TrajectoryReplay", node), executing_(false), waiting_for_smoothed_trajectory_(false)
{
    std::string input_topic, output_topic, traj_output_topic;
    node_->get_parameter("controllers.TrajectoryReplay.input_topic", input_topic);
    node_->get_parameter("controllers.TrajectoryReplay.output_topic", output_topic);

    // 获取工作空间中的轨迹存储目录
    try {
        // 获取 arm_controller 包的共享目录路径
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("arm_controller");

        // 从包目录获取工作空间根目录 (install/arm_controller -> install -> workspace_root)
        std::filesystem::path pkg_path(package_share_dir);
        std::filesystem::path workspace_root = pkg_path.parent_path().parent_path();

        // 构造轨迹目录路径: workspace_root/trajectories
        record_input_dir_ = (workspace_root / "trajectories").string();

        RCLCPP_INFO(node_->get_logger(), "Trajectory directory: %s", record_input_dir_.c_str());

        // 检查目录是否存在
        if (!std::filesystem::exists(record_input_dir_)) {
            RCLCPP_ERROR(node_->get_logger(), "❎ Trajectory directory does not exist: %s", record_input_dir_.c_str());
            throw std::runtime_error("Trajectory directory not found: " + record_input_dir_);
        }
    } catch (const std::exception& e) {
        record_input_dir_ = "/tmp/arm_recording_trajectories";
        RCLCPP_WARN(node_->get_logger(), "❎ Using fallback trajectory directory: %s", record_input_dir_.c_str());
    }

    // 创建录制器实例（用于加载轨迹）
    recorder_ = std::make_unique<JointRecorder>(100.0);

    // 订阅轨迹回放命令
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        input_topic, rclcpp::QoS(10).reliable(), 
        std::bind(&TrajectoryReplayController::teach_callback, this, std::placeholders::_1)
    );

    // 订阅关节状态（用于回到起点）
    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&TrajectoryReplayController::joint_states_callback, this, std::placeholders::_1)
    );

    // 订阅当前模式状态（用于检查是否在录制）
    // mode_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    //     "/controller_api/running_status", 10, std::bind(&TrajectoryReplayController::mode_status_callback, this, std::placeholders::_1)
    // );

    // 发布状态反馈
    pub_ = node_->create_publisher<std_msgs::msg::String>(output_topic, 10);

    // [已移除] 不再通过ROS话题进行平滑节点通信
    // raw_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/raw_joint_trajectory", 10);
    // smoothed_traj_sub_ = node_->create_subscription<...>("/smoothed_joint_trajectory", ...);

    // 创建模式切换服务客户端
    // mode_service_client_ = node_->create_client<controller_interfaces::srv::WorkMode>(
    //     "/controller_api/controller_mode");

    // 创建 MoveJ 命令发布器（使用 JointState 类型）
    movej_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/controller_api/movej_action", 10);

    // 初始化 MoveGroupInterface（用于回到起点）
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");

    // 获取 HardwareManager 实例（用于直接执行轨迹）
    hardware_manager_ = HardwareManager::getInstance();

    RCLCPP_INFO(node_->get_logger(), "TrajectoryReplayController initialized. Input dir: %s (using C++ csaps for smoothing)", record_input_dir_.c_str());
}

void TrajectoryReplayController::start(const std::string& mapping) {
    active_mapping_ = mapping;
    is_active_ = true;
    RCLCPP_INFO(node_->get_logger(), "TrajectoryReplayController activated with mapping: %s", mapping.c_str());
}

bool TrajectoryReplayController::stop(const std::string& mapping) {
    is_active_ = false;

    // 清空轨迹池
    {
        std::lock_guard<std::mutex> lock(traj_pool_mutex_);
        while (!traj_pool_.empty()) {
            traj_pool_.pop();
        }
    }

    executing_ = false;

    // 清理该 mapping 的话题订阅
    cleanup_subscriptions(mapping);

    // 清理资源
    active_mapping_.clear();

    RCLCPP_INFO(node_->get_logger(), "TrajectoryReplayController deactivated");
    return true;
}

void TrajectoryReplayController::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joint_states_mutex_);
    latest_joint_states_ = msg;
}

// void TrajectoryReplayController::mode_status_callback(const std_msgs::msg::String::SharedPtr msg) {
//     std::lock_guard<std::mutex> lock(mode_mutex_);
//     current_mode_ = msg->data;
// }

// bool TrajectoryReplayController::is_recording_active() {
//     std::lock_guard<std::mutex> lock(mode_mutex_);
//     return current_mode_.find("TrajectoryRecord") != std::string::npos;
// }

void TrajectoryReplayController::teach_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "trajectory_replay_callback called with data: '%s', is_active: %d",
                msg->data.c_str(), is_active_);

    if (!is_active_) {
        RCLCPP_WARN(node_->get_logger(), "Controller is not active, ignoring replay request");
        return;
    }
    if (msg->data.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Empty data received, ignoring");
        return;
    }

    // 处理停止命令
    if (msg->data == "stop") {
        RCLCPP_INFO(node_->get_logger(), "Stopping trajectory replay");

        // 清空轨迹池
        {
            std::lock_guard<std::mutex> lock(traj_pool_mutex_);
            while (!traj_pool_.empty()) {
                traj_pool_.pop();
            }
        }

        executing_ = false;
        publish_status("stopped");
        return;
    }

    // 检查是否在录制模式
    // if (get_mode() != "TrajectoryReplay") {
    //     RCLCPP_ERROR(node_->get_logger(), "Cannot replay while recording! Please stop recording first.");
    //     publish_status("error:recording_active");
    //     return;
    // }

    // 加载轨迹文件
    std::string file_path = record_input_dir_ + "/" + msg->data + ".txt";

    if (!std::filesystem::exists(file_path)) {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory file not found: %s", file_path.c_str());
        publish_status("error:file_not_found");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Loading trajectory from: %s", file_path.c_str());

    // 使用 JointRecorder 加载轨迹
    auto traj_msg = recorder_->repeat(file_path);

    if (traj_msg.points.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Loaded trajectory is empty");
        publish_status("error:empty_trajectory");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Loaded trajectory with %zu points", traj_msg.points.size());

    // 设置轨迹消息头
    traj_msg.header.stamp = node_->get_clock()->now();
    traj_msg.header.frame_id = "base_link";

    // 将轨迹加入队列
    {
        std::lock_guard<std::mutex> lock(traj_pool_mutex_);
        traj_pool_.push(traj_msg);
        RCLCPP_INFO(node_->get_logger(), "Trajectory added to pool. Pool size: %zu", traj_pool_.size());
    }

    // 如果当前没有在执行，开始执行
    if (!executing_) {
        execute_next_trajectory();
    }
}

void TrajectoryReplayController::execute_next_trajectory() {
    std::lock_guard<std::mutex> lock(traj_pool_mutex_);

    if (traj_pool_.empty()) {
        RCLCPP_INFO(node_->get_logger(), "No trajectories in pool.");
        executing_ = false;
        return;
    }

    // 从队列取出轨迹
    current_traj_ = traj_pool_.front();
    traj_pool_.pop();

    RCLCPP_INFO(node_->get_logger(), "Starting trajectory execution (%zu points)", current_traj_.points.size());

    // 检查轨迹是否有效
    if (current_traj_.points.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory has no points!");
        execute_next_trajectory();  // 尝试执行下一个
        return;
    }

    // 获取配置的初始位置（而不是轨迹的起点）
    std::vector<double> initial_position = hardware_manager_->get_initial_position(active_mapping_);
    if (initial_position.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No initial_position configured, skipping move to initial");
    }

    // 标记为正在执行（在发送之前设置，避免重复发送）
    executing_ = true;

    // 回到初始位置
    if (!initial_position.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Moving to initial position...");
        if (!move_to_start_position(initial_position)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to move to initial position!");
            publish_status("error:move_to_initial_failed");
            executing_ = false;
            return;
        }
    }

    // 回到起点成功，使用csaps库直接平滑轨迹（不再通过ROS话题）
    RCLCPP_INFO(node_->get_logger(), "Move to start completed, smoothing trajectory (%zu points) with csaps...",
                current_traj_.points.size());

    auto smoothed_traj = smooth_trajectory(current_traj_);

    RCLCPP_INFO(node_->get_logger(), "Trajectory smoothing completed, executing...");

    // 转换 ROS trajectory 为 trajectory_interpolator::Trajectory
    trajectory_interpolator::Trajectory hw_trajectory;
    hw_trajectory.joint_names = smoothed_traj.joint_names;
    hw_trajectory.points.reserve(smoothed_traj.points.size());

    for (const auto& point : smoothed_traj.points) {
        trajectory_interpolator::TrajectoryPoint hw_point;
        hw_point.time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;

        // 将位置从弧度转换为度数（硬件驱动期望度数）
        hw_point.positions.reserve(point.positions.size());
        for (const auto& pos_rad : point.positions) {
            hw_point.positions.push_back(pos_rad * 180.0 / M_PI);
        }

        // 速度和加速度也需要转换（rad/s -> deg/s, rad/s^2 -> deg/s^2）
        hw_point.velocities.reserve(point.velocities.size());
        for (const auto& vel_rad : point.velocities) {
            hw_point.velocities.push_back(vel_rad * 180.0 / M_PI);
        }

        hw_point.accelerations.reserve(point.accelerations.size());
        for (const auto& acc_rad : point.accelerations) {
            hw_point.accelerations.push_back(acc_rad * 180.0 / M_PI);
        }

        hw_trajectory.points.push_back(hw_point);
    }

    // 获取 interface
    std::string interface = hardware_manager_->get_interface(active_mapping_);
    if (interface.empty() || interface == "unknown_interface") {
        RCLCPP_ERROR(node_->get_logger(), "Invalid interface for mapping: %s", active_mapping_.c_str());
        publish_status("error:invalid_interface");
        executing_ = false;
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Executing trajectory on interface: %s (mapping: %s)",
                interface.c_str(), active_mapping_.c_str());

    // 直接调用 hardware_manager 执行轨迹（阻塞调用）
    bool success = hardware_manager_->executeTrajectory(interface, hw_trajectory);

    if (success) {
        RCLCPP_INFO(node_->get_logger(), "Trajectory execution completed successfully");

        // 轨迹执行完成后，立即发送保持当前位置的命令
        // 避免 HoldState 使用旧的位置（轨迹起点）导致电机突然回到起点
        auto current_positions = hardware_manager_->get_current_joint_positions(active_mapping_);
        if (!current_positions.empty()) {
            hardware_manager_->send_hold_state_command(active_mapping_, current_positions);
            RCLCPP_INFO(node_->get_logger(), "Sent hold command at trajectory end position");
        }

        publish_status("completed");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory execution failed");
        publish_status("error:execution_failed");
    }

    executing_ = false;
}

bool TrajectoryReplayController::move_to_start_position(const std::vector<double>& start_pos) {
    // 使用 MoveIt 的 MoveGroupInterface 直接规划和执行（像旧代码那样）
    RCLCPP_INFO(node_->get_logger(), "Moving to start position using MoveIt...");

    // 设置目标关节角度
    move_group_->setJointValueTarget(start_pos);

    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "MoveIt failed to plan motion to start position!");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "MoveIt successfully planned, executing...");

    // 执行（阻塞调用，等待完成）
    success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "MoveIt failed to execute motion to start position!");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Successfully moved to start position");
    return true;
}

// [已移除] smoothed_trajectory_callback
// 该函数已被整合到execute_next_trajectory()中，使用C++ csaps库直接平滑轨迹
// 不再通过ROS话题进行平滑节点通信

void TrajectoryReplayController::publish_status(const std::string& status) {
    auto status_msg = std_msgs::msg::String();
    status_msg.data = status;
    pub_->publish(status_msg);
}

trajectory_msgs::msg::JointTrajectory TrajectoryReplayController::smooth_trajectory(
    const trajectory_msgs::msg::JointTrajectory& raw_traj) {

    using namespace csaps;

    int n_joints = raw_traj.joint_names.size();
    int n_points = raw_traj.points.size();

    if (n_points < 2 || n_joints == 0) {
        RCLCPP_WARN(node_->get_logger(), "Trajectory too small for smoothing, returning as-is");
        return raw_traj;
    }

    // 提取时间序列和多维位置数据
    DoubleArray t(n_points);
    DoubleArray2D y_data(n_points, n_joints);  // (n_points, n_joints)

    for (int i = 0; i < n_points; ++i) {
        t(i) = raw_traj.points[i].time_from_start.sec +
               raw_traj.points[i].time_from_start.nanosec * 1e-9;

        for (int j = 0; j < n_joints; ++j) {
            y_data(i, j) = raw_traj.points[i].positions[j];
        }
    }

    // 创建平滑后的轨迹消息
    trajectory_msgs::msg::JointTrajectory smoothed_traj = raw_traj;

    // 使用多元样条一次性平滑所有关节（性能更优）
    const double smooth_factor = 0.95;  // 平滑因子 (0-1, 越大越平滑)

    try {
        // 创建多变量样条（一次处理所有关节）
        MultivariateCubicSmoothingSpline spline(t, y_data, smooth_factor);

        // 计算平滑位置和速度
        DoubleArray2D smooth_y = spline(t);         // 位置
        DoubleArray2D smooth_dy = spline(t, 1);     // 一阶导（速度）

        // 更新轨迹点
        for (int i = 0; i < n_points; ++i) {
            for (int j = 0; j < n_joints; ++j) {
                smoothed_traj.points[i].positions[j] = smooth_y(i, j);
                smoothed_traj.points[i].velocities[j] = smooth_dy(i, j);
            }
        }

        // 强制首尾速度为零（避免轨迹执行完成后仍有速度）
        for (int j = 0; j < n_joints; ++j) {
            smoothed_traj.points.front().velocities[j] = 0.0;
            smoothed_traj.points.back().velocities[j] = 0.0;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error smoothing trajectory: %s", e.what());
        return raw_traj;  // 平滑失败，返回原始轨迹
    }

    RCLCPP_INFO(node_->get_logger(), "Trajectory smoothing completed: %d joints, %d points",
                n_joints, n_points);

    return smoothed_traj;
}
