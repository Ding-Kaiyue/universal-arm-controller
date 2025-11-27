#include "trajectory_replay_controller.hpp"
#include "controller_interface.hpp"
#include <filesystem>
#include <thread>
#include <future>

TrajectoryReplayController::TrajectoryReplayController(const rclcpp::Node::SharedPtr & node)
    : RecordControllerBase("TrajectoryReplay", node), executing_(false), waiting_for_smoothed_trajectory_(false)
{
    std::string input_topic, output_topic, traj_output_topic;
    node_->get_parameter("controllers.TrajectoryReplay.input_topic", input_topic);
    node_->get_parameter("controllers.TrajectoryReplay.output_topic", output_topic);

    // 使用工作空间的统一轨迹存储目录
    record_input_dir_ = "/home/w/work/robotic_arm_ws/trajectories";

    // 检查目录是否存在
    if (!std::filesystem::exists(record_input_dir_)) {
        RCLCPP_WARN(node_->get_logger(), "Trajectory directory does not exist: %s", record_input_dir_.c_str());
        try {
            std::filesystem::create_directories(record_input_dir_);
            RCLCPP_INFO(node_->get_logger(), "Created trajectory directory: %s", record_input_dir_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create directory %s: %s",
                        record_input_dir_.c_str(), e.what());
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "Using trajectory directory: %s", record_input_dir_.c_str());
    }

    // 创建录制器实例（用于加载轨迹）
    recorder_ = std::make_unique<JointRecorder>(100.0);

    // 订阅轨迹回放命令
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        input_topic, 10, std::bind(&TrajectoryReplayController::trajectory_replay_callback, this, std::placeholders::_1)
    );

    // 订阅关节状态（用于回到起点）
    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&TrajectoryReplayController::joint_states_callback, this, std::placeholders::_1)
    );

    // 订阅当前模式状态（用于检查是否在录制）
    mode_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/controller_api/running_status", 10, std::bind(&TrajectoryReplayController::mode_status_callback, this, std::placeholders::_1)
    );

    // 发布状态反馈
    pub_ = node_->create_publisher<std_msgs::msg::String>(output_topic, 10);

    // 发布原始轨迹（给平滑节点，平滑节点会处理并执行）
    raw_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/raw_joint_trajectory", 10);

    // 创建模式切换服务客户端
    mode_service_client_ = node_->create_client<controller_interfaces::srv::WorkMode>(
        "/controller_api/controller_mode");

    // 创建 MoveJ 命令发布器（使用 JointState 类型）
    movej_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/controller_api/movej_action", 10);

    // 初始化 MoveGroupInterface（用于回到起点）
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");

    // 获取 HardwareManager 实例（用于直接执行轨迹）
    hardware_manager_ = HardwareManager::getInstance();

    // 订阅平滑后的轨迹（smoother_node 发布）
    smoothed_traj_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/smoothed_joint_trajectory", 10,
        std::bind(&TrajectoryReplayController::smoothed_trajectory_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "TrajectoryReplayController initialized. Input dir: %s", record_input_dir_.c_str());
}

void TrajectoryReplayController::start(const std::string& mapping) {
    current_mapping_ = mapping;
    is_active_ = true;
    RCLCPP_INFO(node_->get_logger(), "TrajectoryReplayController activated with mapping: %s", mapping.c_str());
}

bool TrajectoryReplayController::stop(const std::string& mapping) {
    (void)mapping;
    is_active_ = false;

    // 清空轨迹池
    {
        std::lock_guard<std::mutex> lock(traj_pool_mutex_);
        while (!traj_pool_.empty()) {
            traj_pool_.pop();
        }
    }

    executing_ = false;
    RCLCPP_INFO(node_->get_logger(), "TrajectoryReplayController deactivated");
    return true;
}

void TrajectoryReplayController::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joint_states_mutex_);
    latest_joint_states_ = msg;
}

void TrajectoryReplayController::mode_status_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    current_mode_ = msg->data;
}

bool TrajectoryReplayController::is_recording_active() {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    return current_mode_.find("TrajectoryRecord") != std::string::npos;
}

void TrajectoryReplayController::trajectory_replay_callback(const std_msgs::msg::String::SharedPtr msg) {
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
    if (is_recording_active()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot replay while recording! Please stop recording first.");
        publish_status("error:recording_active");
        return;
    }

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
    std::vector<double> initial_position = hardware_manager_->get_initial_position(current_mapping_);
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

    // 回到起点成功，设置等待标志并发布轨迹到平滑节点
    waiting_for_smoothed_trajectory_ = true;
    RCLCPP_INFO(node_->get_logger(), "Move to start completed, publishing trajectory (%zu points) to smoother...",
                current_traj_.points.size());
    raw_traj_pub_->publish(current_traj_);

    // 发布完成，平滑节点会接管后续执行
    executing_ = false;
    publish_status("replaying");
    RCLCPP_INFO(node_->get_logger(), "Trajectory published to smoother. Waiting for smoothed trajectory...");
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

void TrajectoryReplayController::smoothed_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    // 使用 waiting_for_smoothed_trajectory_ 标志而不是 is_active_
    // 因为 MoveIt 回到起点后会触发 action_succeeded，导致 controller 被 deactivate
    // 但此时 smoother_node 还没有发布平滑后的轨迹
    if (!waiting_for_smoothed_trajectory_) {
        RCLCPP_DEBUG(node_->get_logger(), "Received smoothed trajectory but not waiting for it, ignoring");
        return;
    }

    // 清除等待标志
    waiting_for_smoothed_trajectory_ = false;

    if (msg->points.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Received empty smoothed trajectory, ignoring");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Received smoothed trajectory with %zu points, executing directly via hardware_manager",
                msg->points.size());

    // 转换 ROS trajectory 为 trajectory_interpolator::Trajectory
    trajectory_interpolator::Trajectory hw_trajectory;
    hw_trajectory.joint_names = msg->joint_names;
    hw_trajectory.points.reserve(msg->points.size());

    for (const auto& point : msg->points) {
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
    std::string interface = hardware_manager_->get_interface(current_mapping_);
    if (interface.empty() || interface == "unknown_interface") {
        RCLCPP_ERROR(node_->get_logger(), "Invalid interface for mapping: %s", current_mapping_.c_str());
        publish_status("error:invalid_interface");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Executing trajectory on interface: %s (mapping: %s)",
                interface.c_str(), current_mapping_.c_str());

    // 直接调用 hardware_manager 执行轨迹（阻塞调用）
    bool success = hardware_manager_->executeTrajectory(interface, hw_trajectory);

    if (success) {
        RCLCPP_INFO(node_->get_logger(), "Trajectory execution completed successfully");

        // 轨迹执行完成后，立即发送保持当前位置的命令
        // 避免 HoldState 使用旧的位置（轨迹起点）导致电机突然回到起点
        auto current_positions = hardware_manager_->get_current_joint_positions(current_mapping_);
        if (!current_positions.empty()) {
            hardware_manager_->send_hold_state_command(current_mapping_, current_positions);
            RCLCPP_INFO(node_->get_logger(), "Sent hold command at trajectory end position");
        }

        publish_status("completed");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory execution failed");
        publish_status("error:execution_failed");
    }

    executing_ = false;
}

void TrajectoryReplayController::publish_status(const std::string& status) {
    auto status_msg = std_msgs::msg::String();
    status_msg.data = status;
    pub_->publish(status_msg);
}
