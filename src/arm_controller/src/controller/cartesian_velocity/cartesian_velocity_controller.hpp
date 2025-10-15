#ifndef __CARTESIAN_VELOCITY_CONTROLLER_HPP__
#define __CARTESIAN_VELOCITY_CONTROLLER_HPP__

#include "controller_base/trajectory_controller_base.hpp"
#include <geometry_msgs/msg/twist.hpp>

class CartesianVelocityController final
    : public TrajectoryControllerImpl<geometry_msgs::msg::Twist> {
public:
    explicit CartesianVelocityController(const rclcpp::Node::SharedPtr& node);
    ~CartesianVelocityController() override = default;

    void start(const std::string& mapping) override;
    bool stop() override;

protected:
    void trajectory_callback(const geometry_msgs::msg::Twist::SharedPtr msg) override;
    void publish_trajectory(const geometry_msgs::msg::Twist::SharedPtr msg) override;

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

#endif      // __CARTESIAN_VELOCITY_CONTROLLER_HPP__
