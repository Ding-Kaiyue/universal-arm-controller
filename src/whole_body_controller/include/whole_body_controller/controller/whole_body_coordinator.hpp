#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace whole_body_controller {

struct PillarObstacle {
    double x{0.0};
    double y{0.0};
    double radius{0.075};
};

struct DualHandTarget {
    geometry_msgs::msg::Pose left;
    geometry_msgs::msg::Pose right;
    std::string frame_id;
    rclcpp::Time stamp;
    bool left_sent{false};
    bool right_sent{false};
};

class WholeBodyCoordinator {
public:
    explicit WholeBodyCoordinator(rclcpp::Node& node);

    void start();

private:
    struct BasePose2d {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    void loadParameters();
    void handleTarget(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void controlTick();

    std::optional<BasePose2d> lookupBasePose() const;
    geometry_msgs::msg::Pose transformPoseToWorld(
        const geometry_msgs::msg::Pose& pose,
        const std::string& source_frame,
        const rclcpp::Time& stamp) const;
    geometry_msgs::msg::Twist computeBaseVelocityCommand(
        const DualHandTarget& target,
        const BasePose2d& base_pose) const;
    geometry_msgs::msg::Twist obstacleAvoidanceCommand(
        const BasePose2d& base_pose) const;
    bool isBaseReadyForHands(
        const DualHandTarget& target,
        const BasePose2d& base_pose) const;
    void publishHandTargetsIfReady(DualHandTarget* target);
    void publishStopCommand();
    void publishStatus(const std::string& status) const;

    rclcpp::Node& node_;

    std::string target_topic_;
    std::string cmd_vel_topic_;
    std::string left_hand_topic_;
    std::string right_hand_topic_;
    std::string world_frame_;
    std::string base_frame_;

    double control_frequency_hz_{30.0};
    double desired_target_x_in_base_{0.55};
    double desired_target_y_in_base_{0.0};
    double base_ready_tolerance_m_{0.08};
    double base_kp_xy_{0.8};
    double max_vx_{0.25};
    double max_vy_{0.25};
    double obstacle_robot_radius_m_{0.40};
    double obstacle_safety_margin_m_{0.12};
    double obstacle_influence_distance_m_{0.80};
    double obstacle_gain_{0.20};
    double obstacle_max_speed_{0.20};

    std::vector<PillarObstacle> pillar_obstacles_;
    std::optional<DualHandTarget> active_target_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr left_hand_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr right_hand_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace whole_body_controller
