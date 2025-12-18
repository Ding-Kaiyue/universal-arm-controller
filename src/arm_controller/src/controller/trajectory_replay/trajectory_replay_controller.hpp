#ifndef __TRAJECTORY_REPLAY_CONTROLLER_HPP__
#define __TRAJECTORY_REPLAY_CONTROLLER_HPP__

#include "controller_base/teach_controller_base.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "controller_interfaces/srv/work_mode.hpp"
#include "arm_controller/utils/joint_recorder.hpp"
#include "arm_controller/hardware/hardware_manager.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <csaps.h>  // C++ CSAPS库，用于轨迹平滑
#include <memory>
#include <queue>
#include <mutex>
#include <atomic>

class TrajectoryReplayController final: public TeachControllerBase {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    explicit TrajectoryReplayController(const rclcpp::Node::SharedPtr& node);
    ~TrajectoryReplayController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

private:
    // ===== 订阅者 =====
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_status_sub_;

    // ===== 发布者 =====
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    // rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr raw_traj_pub_;  // [已移除] 不再使用ROS话题平滑
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr movej_pub_;  // MoveJ 命令发布器

    // [已移除] 使用C++直接csaps平滑，不再需要ROS话题通信
    // rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr smoothed_traj_sub_;

    // ===== 服务客户端 =====
    // rclcpp::Client<controller_interfaces::srv::WorkMode>::SharedPtr mode_service_client_;

    // ===== 回调函数 =====
    void teach_callback(const std_msgs::msg::String::SharedPtr msg) override;
    // void trajectory_replay_callback(const std_msgs::msg::String::SharedPtr msg);
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    // void mode_status_callback(const std_msgs::msg::String::SharedPtr msg);
    // [已移除] void smoothed_trajectory_callback(...)  // 使用C++直接csaps平滑，不再需要此回调

    // ===== 核心功能 =====
    // bool is_recording_active();  // 检查是否在录制模式
    bool move_to_start_position(const std::vector<double>& start_pos);  // 回到起点
    void execute_next_trajectory();  // 执行下一个轨迹
    trajectory_msgs::msg::JointTrajectory smooth_trajectory(
        const trajectory_msgs::msg::JointTrajectory& raw_traj);  // 使用csaps平滑轨迹

    // ===== 辅助函数 =====
    void publish_status(const std::string& status);

    // ===== 成员变量 =====
    std::unique_ptr<JointRecorder> recorder_;  // 关节录制器实例（用于加载轨迹）
    std::string record_input_dir_;  // 录制输入目录

    // 当前关节状态
    sensor_msgs::msg::JointState::SharedPtr latest_joint_states_;
    std::mutex joint_states_mutex_;

    // 当前模式状态
    // std::string current_mode_;
    // std::mutex mode_mutex_;

    // 轨迹池（支持多轨迹排队）
    std::queue<trajectory_msgs::msg::JointTrajectory> traj_pool_;
    std::mutex traj_pool_mutex_;
    std::atomic<bool> executing_;  // 是否正在执行轨迹
    std::atomic<bool> waiting_for_smoothed_trajectory_;  // 是否正在等待平滑后的轨迹

    // 当前轨迹和执行时间
    trajectory_msgs::msg::JointTrajectory current_traj_;
    rclcpp::Time traj_start_time_;

    // 当前action goal handle
    // GoalHandleFollowJointTrajectory::SharedPtr current_goal_handle_;
    // std::mutex goal_handle_mutex_;

    // MoveIt MoveGroupInterface（用于回到起点）
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // HardwareManager（用于直接执行轨迹）
    std::shared_ptr<HardwareManager> hardware_manager_;
    std::string active_mapping_;  // 当前mapping
};

#endif      // __TRAJECTORY_REPLAY_CONTROLLER_HPP__
