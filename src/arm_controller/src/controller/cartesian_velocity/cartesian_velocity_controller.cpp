#include "cartesian_velocity_controller.hpp"
#include "controller_interface.hpp"

CartesianVelocityController::CartesianVelocityController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<geometry_msgs::msg::Twist>("CartesianVelocity", node)
{
    std::string input_topic;
    node_->get_parameter("controllers.CartesianVelocity.input_topic", input_topic);

    sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        input_topic, 10,
        std::bind(&CartesianVelocityController::trajectory_callback, this, std::placeholders::_1)
    );
}

void CartesianVelocityController::start(const std::string& mapping) {
    ModeControllerBase::start(mapping);
    RCLCPP_INFO(node_->get_logger(), "CartesianVelocityController activated");
}

bool CartesianVelocityController::stop() {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "CartesianVelocityController deactivated");
    return true;  // 需要钩子状态来安全停止
}

void CartesianVelocityController::trajectory_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!is_active_) return;

    // TODO: 实现末端速度到关节速度的转换
    // 这里将使用trajectory_planning包中的实时速度控制功能

    RCLCPP_INFO(node_->get_logger(), "[CartesianVelocityController] Received velocity command: linear=[%f,%f,%f], angular=[%f,%f,%f]",
                msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);

    // 发布轨迹命令
    publish_trajectory(msg);
}

void CartesianVelocityController::publish_trajectory(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // TODO: 实现轨迹发布逻辑
    // 这里将通过硬件驱动发送速度命令到电机
    auto hardware_driver = get_hardware_driver();
    if (!hardware_driver) {
        RCLCPP_ERROR(node_->get_logger(), "Hardware driver not initialized");
        return;
    }

    // 暂时的实现：将Twist消息转换为关节速度并发送
    // TODO: 使用正确的运动学转换
    RCLCPP_INFO(node_->get_logger(), "[CartesianVelocityController] Publishing trajectory for velocity control");
}


