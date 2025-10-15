#ifndef __JOINT_VELOCITY_CONTROLLER_HPP__
#define __JOINT_VELOCITY_CONTROLLER_HPP__

#include "controller_base/velocity_controller_base.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hardware/hardware_manager.hpp"

class JointVelocityController final
    : public VelocityControllerImpl<sensor_msgs::msg::JointState> {
public:
    explicit JointVelocityController(const rclcpp::Node::SharedPtr& node);
    ~JointVelocityController() override;

    void start(const std::string& mapping) override;
    bool stop(const std::string& mapping) override;

protected:
    void velocity_callback(const sensor_msgs::msg::JointState::SharedPtr msg) override;
    bool send_joint_velocities(const std::vector<double>& joint_velocities);

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    std::shared_ptr<HardwareManager> hardware_manager_;
    std::string mapping_;
};

#endif      // __JOINT_VELOCITY_CONTROLLER_HPP__
