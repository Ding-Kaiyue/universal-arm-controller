#ifndef __CONTROLLER_NODE_HPP__
#define __CONTROLLER_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_interfaces/srv/work_mode.hpp>
#include "arm_controller/controller_base/mode_controller_base.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include "trajectory_interpolator/moveit_spline_adapter.hpp"
#include "arm_controller/utils/trajectory_converter.hpp"

class ControllerNode : public rclcpp::Node {

public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>;

    explicit ControllerNode(const std::string& node_name);
    
    void post_init();

private:
    // 配置加载
    std::string load_urdf_file(const std::string& robot_model_name) {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_description");
        std::string urdf_file_path = package_share_directory + "/urdf/" + robot_model_name + ".urdf";
        std::ifstream urdf_file(urdf_file_path);

        if (!urdf_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", urdf_file_path.c_str());
            return "";
        }
        std::stringstream urdf_string_stream;
        urdf_string_stream << urdf_file.rdbuf();
        urdf_file.close();

        return urdf_string_stream.str();
    }

    void init_controllers();
    void init_commons();
    void init_ros_interfaces();
    void init_interpolation_config();
    void load_joint_limits();
    void load_joint_velocity_limits();
    
    // Action Server 相关方法
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute_trajectory(const std::shared_ptr<GoalHandle> goal_handle);
    
    // 权限和模式检查
    bool check_permissions(const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    bool check_work_mode(const std::string& required_mode);
    
    // 现有的工作模式服务
    void handle_work_mode(
        const std::shared_ptr<controller_interfaces::srv::WorkMode::Request> request,
        std::shared_ptr<controller_interfaces::srv::WorkMode::Response> response);

    bool start_working_controller(const std::string& mode_name);
    bool stop_working_controller(bool& need_hook);
    
    // 钩子状态相关方法
    bool enter_hook_state(const std::string& target_mode);
    bool exit_hook_state();
    bool check_hook_state_transition();
    bool switch_to_mode(const std::string& mode_name);
    
public:
    // 测试访问方法
    bool is_in_hook_state() const { return in_hook_state_; }
    std::string get_target_mode() const { return target_mode_; }

    // 插值配置相关方法 - 公共接口供子控制器使用
    const trajectory_interpolator::SplineConfig& get_default_spline_config() const { return default_spline_config_; }
    // double get_min_joint_velocity_limit() const;
    
private:
    struct TopicInfo {
        std::string key;
        std::string name;
        std::string type;
        std::string kind; // "input_topic", "output_topic", "service", "action_server" 等
    };
    std::unordered_map<std::string, TopicInfo> common_topics_;
    std::unordered_map<std::string, TopicInfo> controller_topics_;

    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

    // Action Server
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;
    // 转发给TrajectoryNode的发布器
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_command_pub_;

    // 从TrajectoryNode接收反馈和结果的订阅器
    rclcpp::Subscription<control_msgs::action::FollowJointTrajectory::Feedback>::SharedPtr trajectory_feedback_sub_;
    rclcpp::Subscription<control_msgs::action::FollowJointTrajectory::Result>::SharedPtr trajectory_result_sub_;

    rclcpp::Service<controller_interfaces::srv::WorkMode>::SharedPtr working_mode_service_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr running_status_sub_;

    std::string current_mode_;
    std::string target_mode_;  // 目标模式（当处于钩子状态时）
    std::unordered_map<std::string, std::shared_ptr<ModeControllerBase>> controller_map_;
    std::unordered_map<std::string, std::any> cached_messages_; // 用于缓存最后一条控制消息，避免指令先到模式转移后到发生丢失

    // 当前执行的goal handle
    std::shared_ptr<GoalHandle> current_goal_handle_;
    
    bool need_hook_;    // 是否需要hook
    bool in_hook_state_; // 是否处于钩子状态
    // 权限控制相关
    bool emergency_stop_active_;
    bool safety_zone_violation_;

    // 从urdf中读取的关节限制
    YAML::Node yaml_config_;
    std::unordered_map<std::string, std::pair<double, double>> joint_position_limits_; // 关节限位
    std::unordered_map<std::string, double> joint_velocity_limits_;   // 关节限速

    // 轨迹插值器
    std::unique_ptr<trajectory_interpolator::MoveItSplineAdapter> trajectory_interpolator_;

    // 默认插值配置
    trajectory_interpolator::SplineConfig default_spline_config_;

};

#endif   // __CONTROLLER_NODE_HPP__
