#include "whole_body_controller/controller/whole_body_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace whole_body_controller {
namespace {

double clamp(const double value, const double lower, const double upper) {
    return std::max(lower, std::min(value, upper));
}

double norm2(const double x, const double y) {
    return std::sqrt(x * x + y * y);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q_msg) {
    tf2::Quaternion q;
    tf2::fromMsg(q_msg, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

}  // namespace

WholeBodyCoordinator::WholeBodyCoordinator(rclcpp::Node& node)
    : node_(node) {
    loadParameters();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void WholeBodyCoordinator::start() {
    target_sub_ = node_.create_subscription<geometry_msgs::msg::PoseArray>(
        target_topic_, rclcpp::QoS(10).reliable(),
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) { handleTarget(msg); });

    chassis_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(10).reliable());
    left_hand_pub_ = node_.create_publisher<geometry_msgs::msg::Pose>(
        left_hand_topic_, rclcpp::QoS(10).reliable());
    right_hand_pub_ = node_.create_publisher<geometry_msgs::msg::Pose>(
        right_hand_topic_, rclcpp::QoS(10).reliable());
    status_pub_ = node_.create_publisher<std_msgs::msg::String>(
        "whole_body_controller/status", rclcpp::QoS(10).reliable());

    const auto period = std::chrono::duration<double>(1.0 / control_frequency_hz_);
    timer_ = node_.create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { controlTick(); });

    RCLCPP_INFO(
        node_.get_logger(),
        "whole_body_controller ready: target=%s cmd_vel=%s left=%s right=%s",
        target_topic_.c_str(),
        cmd_vel_topic_.c_str(),
        left_hand_topic_.c_str(),
        right_hand_topic_.c_str());
}

void WholeBodyCoordinator::loadParameters() {
    target_topic_ = node_.declare_parameter<std::string>(
        "target_topic", "whole_body_controller/dual_hand_targets");
    cmd_vel_topic_ = node_.declare_parameter<std::string>(
        "cmd_vel_topic", "cmd_vel");
    left_hand_topic_ = node_.declare_parameter<std::string>(
        "left_hand_topic", "/controller_api/reactive_task_action/left_arm");
    right_hand_topic_ = node_.declare_parameter<std::string>(
        "right_hand_topic", "/controller_api/reactive_task_action/right_arm");
    world_frame_ = node_.declare_parameter<std::string>("world_frame", "odom");
    base_frame_ = node_.declare_parameter<std::string>("base_frame", "base_footprint");

    control_frequency_hz_ = node_.declare_parameter<double>("control_frequency_hz", 30.0);
    desired_target_x_in_base_ =
        node_.declare_parameter<double>("desired_target_x_in_base", 0.55);
    desired_target_y_in_base_ =
        node_.declare_parameter<double>("desired_target_y_in_base", 0.0);
    base_ready_tolerance_m_ =
        node_.declare_parameter<double>("base_ready_tolerance_m", 0.08);
    base_kp_xy_ = node_.declare_parameter<double>("base_kp_xy", 0.8);
    max_vx_ = node_.declare_parameter<double>("max_vx", 0.25);
    max_vy_ = node_.declare_parameter<double>("max_vy", 0.25);
    obstacle_robot_radius_m_ =
        node_.declare_parameter<double>("obstacle_robot_radius_m", 0.40);
    obstacle_safety_margin_m_ =
        node_.declare_parameter<double>("obstacle_safety_margin_m", 0.12);
    obstacle_influence_distance_m_ =
        node_.declare_parameter<double>("obstacle_influence_distance_m", 0.80);
    obstacle_gain_ = node_.declare_parameter<double>("obstacle_gain", 0.20);
    obstacle_max_speed_ = node_.declare_parameter<double>("obstacle_max_speed", 0.20);

    const std::vector<double> default_obstacles = {
        1.10, 0.55, 0.075,
        1.45, -0.75, 0.075,
        1.95, 0.20, 0.075,
        2.35, -1.05, 0.075,
        2.65, 0.75, 0.075,
        3.20, -0.25, 0.075,
        3.65, 1.05, 0.075,
        4.05, -0.85, 0.075,
        4.55, 0.35, 0.075,
    };
    const std::vector<double> obstacle_values =
        node_.declare_parameter<std::vector<double>>("pillar_obstacles", default_obstacles);
    if (obstacle_values.size() % 3 != 0) {
        RCLCPP_WARN(
            node_.get_logger(),
            "pillar_obstacles must be [x, y, radius] triples; got %zu values",
            obstacle_values.size());
        return;
    }
    pillar_obstacles_.clear();
    for (std::size_t i = 0; i + 2 < obstacle_values.size(); i += 3) {
        pillar_obstacles_.push_back(
            PillarObstacle{obstacle_values[i], obstacle_values[i + 1], obstacle_values[i + 2]});
    }
}

void WholeBodyCoordinator::handleTarget(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.size() < 2) {
        RCLCPP_WARN(
            node_.get_logger(),
            "dual hand target requires at least two poses: poses[0]=left, poses[1]=right");
        return;
    }

    const rclcpp::Time stamp =
        msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
            ? node_.now()
            : rclcpp::Time(msg->header.stamp);
    const std::string source_frame =
        msg->header.frame_id.empty() ? world_frame_ : msg->header.frame_id;

    DualHandTarget target;
    target.left = transformPoseToWorld(msg->poses[0], source_frame, stamp);
    target.right = transformPoseToWorld(msg->poses[1], source_frame, stamp);
    target.frame_id = world_frame_;
    target.stamp = stamp;
    active_target_ = target;

    publishStatus("accepted_dual_hand_target");
}

void WholeBodyCoordinator::controlTick() {
    if (!active_target_) {
        return;
    }

    const std::optional<BasePose2d> base_pose = lookupBasePose();
    if (!base_pose) {
        RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000,
            "waiting for transform %s -> %s",
            world_frame_.c_str(),
            base_frame_.c_str());
        return;
    }

    if (isBaseReadyForHands(*active_target_, *base_pose)) {
        publishStopCommand();
        publishHandTargetsIfReady(&(*active_target_));
        return;
    }

    geometry_msgs::msg::Twist cmd = computeBaseVelocityCommand(*active_target_, *base_pose);
    const geometry_msgs::msg::Twist avoidance = obstacleAvoidanceCommand(*base_pose);
    cmd.linear.x = clamp(cmd.linear.x + avoidance.linear.x, -max_vx_, max_vx_);
    cmd.linear.y = clamp(cmd.linear.y + avoidance.linear.y, -max_vy_, max_vy_);
    chassis_pub_->publish(cmd);
}

std::optional<WholeBodyCoordinator::BasePose2d> WholeBodyCoordinator::lookupBasePose() const {
    try {
        const geometry_msgs::msg::TransformStamped tf =
            tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
        BasePose2d pose;
        pose.x = tf.transform.translation.x;
        pose.y = tf.transform.translation.y;
        pose.yaw = yawFromQuaternion(tf.transform.rotation);
        return pose;
    } catch (const tf2::TransformException&) {
        return std::nullopt;
    }
}

geometry_msgs::msg::Pose WholeBodyCoordinator::transformPoseToWorld(
    const geometry_msgs::msg::Pose& pose,
    const std::string& source_frame,
    const rclcpp::Time& stamp) const {
    if (source_frame == world_frame_) {
        return pose;
    }

    geometry_msgs::msg::PoseStamped in;
    in.header.frame_id = source_frame;
    in.header.stamp = stamp;
    in.pose = pose;

    try {
        const geometry_msgs::msg::TransformStamped tf =
            tf_buffer_->lookupTransform(world_frame_, source_frame, stamp, rclcpp::Duration::from_seconds(0.05));
        geometry_msgs::msg::PoseStamped out;
        tf2::doTransform(in, out, tf);
        return out.pose;
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(
            node_.get_logger(),
            "failed to transform hand target from %s to %s: %s; using original pose",
            source_frame.c_str(),
            world_frame_.c_str(),
            e.what());
        return pose;
    }
}

geometry_msgs::msg::Twist WholeBodyCoordinator::computeBaseVelocityCommand(
    const DualHandTarget& target,
    const BasePose2d& base_pose) const {
    const double mid_x = 0.5 * (target.left.position.x + target.right.position.x);
    const double mid_y = 0.5 * (target.left.position.y + target.right.position.y);

    const double dx_world = mid_x - base_pose.x;
    const double dy_world = mid_y - base_pose.y;
    const double c = std::cos(base_pose.yaw);
    const double s = std::sin(base_pose.yaw);
    const double target_x_base = c * dx_world + s * dy_world;
    const double target_y_base = -s * dx_world + c * dy_world;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x =
        clamp(base_kp_xy_ * (target_x_base - desired_target_x_in_base_), -max_vx_, max_vx_);
    cmd.linear.y =
        clamp(base_kp_xy_ * (target_y_base - desired_target_y_in_base_), -max_vy_, max_vy_);
    return cmd;
}

geometry_msgs::msg::Twist WholeBodyCoordinator::obstacleAvoidanceCommand(
    const BasePose2d& base_pose) const {
    double repel_x_world = 0.0;
    double repel_y_world = 0.0;

    for (const PillarObstacle& obstacle : pillar_obstacles_) {
        const double dx = base_pose.x - obstacle.x;
        const double dy = base_pose.y - obstacle.y;
        const double distance = std::max(norm2(dx, dy), 1e-6);
        const double clearance =
            distance - obstacle.radius - obstacle_robot_radius_m_ - obstacle_safety_margin_m_;
        if (clearance >= obstacle_influence_distance_m_) {
            continue;
        }

        const double safe_clearance = std::max(clearance, 0.02);
        const double strength =
            obstacle_gain_ * (1.0 / safe_clearance - 1.0 / obstacle_influence_distance_m_);
        repel_x_world += strength * dx / distance;
        repel_y_world += strength * dy / distance;
    }

    const double speed = norm2(repel_x_world, repel_y_world);
    if (speed > obstacle_max_speed_ && speed > 1e-6) {
        repel_x_world *= obstacle_max_speed_ / speed;
        repel_y_world *= obstacle_max_speed_ / speed;
    }

    const double c = std::cos(base_pose.yaw);
    const double s = std::sin(base_pose.yaw);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = c * repel_x_world + s * repel_y_world;
    cmd.linear.y = -s * repel_x_world + c * repel_y_world;
    return cmd;
}

bool WholeBodyCoordinator::isBaseReadyForHands(
    const DualHandTarget& target,
    const BasePose2d& base_pose) const {
    const geometry_msgs::msg::Twist cmd = computeBaseVelocityCommand(target, base_pose);
    return norm2(cmd.linear.x / std::max(base_kp_xy_, 1e-6),
                 cmd.linear.y / std::max(base_kp_xy_, 1e-6)) <= base_ready_tolerance_m_;
}

void WholeBodyCoordinator::publishHandTargetsIfReady(DualHandTarget* target) {
    if (!target) {
        return;
    }

    if (!target->left_sent) {
        left_hand_pub_->publish(target->left);
        target->left_sent = true;
    }
    if (!target->right_sent) {
        right_hand_pub_->publish(target->right);
        target->right_sent = true;
    }

    if (target->left_sent && target->right_sent) {
        publishStatus("hand_targets_published");
    }
}

void WholeBodyCoordinator::publishStopCommand() {
    geometry_msgs::msg::Twist stop;
    chassis_pub_->publish(stop);
}

void WholeBodyCoordinator::publishStatus(const std::string& status) const {
    if (!status_pub_) {
        return;
    }
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
}

}  // namespace whole_body_controller
