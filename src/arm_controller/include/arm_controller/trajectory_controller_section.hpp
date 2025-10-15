#ifndef __TRAJECTORY_CONTROLLER_SECTION_HPP__
#define __TRAJECTORY_CONTROLLER_SECTION_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>
#include "hardware/hardware_manager.hpp"
#include "trajectory_interpolator/trajectory_interpolator.hpp"
#include "utils/trajectory_converter.hpp"

class TrajectoryControllerNode : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    TrajectoryControllerNode();
    ~TrajectoryControllerNode() = default;

    // 延迟初始化 - 在构造函数外调用
    void post_init();

private:
    // 初始化
    // void init_parameters();
    void init_hardware();
    void init_action_servers();
    void init_trajectory_interpolator();

    // Action Server 回调
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal,
        const std::string& mapping);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle,
        const std::string& mapping);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle,
                        const std::string& mapping);

    // 轨迹执行
    void execute_trajectory(const std::shared_ptr<GoalHandle> goal_handle, const std::string& mapping);
    // bool check_permissions(const std::shared_ptr<const FollowJointTrajectory::Goal> goal);

    // 事件发布
    void init_event_publisher();
    void publish_action_event(const std::string& event_type, const std::string& mapping = "");
    void on_trajectory_completed(const std::string& mapping = "");

    // 成员变量
    std::string interface_name_;

    // 硬件管理
    std::shared_ptr<HardwareManager> hardware_manager_;

    // 轨迹插值
    std::unique_ptr<TrajectoryInterpolator> trajectory_interpolator_;

    // Action Servers - 每个mapping一个
    std::map<std::string, rclcpp_action::Server<FollowJointTrajectory>::SharedPtr> action_servers_;
    std::shared_ptr<GoalHandle> current_goal_handle_;

    // 事件发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_event_publisher_;

    // 配置
    YAML::Node yaml_config_;
};

#endif // __TRAJECTORY_CONTROLLER_SECTION_HPP__