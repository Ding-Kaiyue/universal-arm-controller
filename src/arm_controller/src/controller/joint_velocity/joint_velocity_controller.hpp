#ifndef __JOINT_VELOCITY_CONTROLLER_HPP__
#define __JOINT_VELOCITY_CONTROLLER_HPP__

#include "controller_base/velocity_controller_base.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "hardware/hardware_manager.hpp"
#include <mutex>
#include <chrono>

class JointVelocityController final
    : public VelocityControllerImpl<sensor_msgs::msg::JointState> {
public:
    explicit JointVelocityController(const rclcpp::Node::SharedPtr& node);
    ~JointVelocityController() override = default;

    void start(const std::string& mapping) override;
    bool stop(const std::string& mapping) override;

protected:
    void velocity_callback(const sensor_msgs::msg::JointState::SharedPtr msg) override;
    bool send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities);

private:
    void control_loop();  // 10ms 控制循环

    std::shared_ptr<HardwareManager> hardware_manager_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    std::string active_mapping_;

    // ✅ 实时控制循环
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::mutex cmd_mutex_;
    sensor_msgs::msg::JointState last_cmd_;
    std::chrono::steady_clock::time_point last_cmd_time_;
    std::chrono::steady_clock steady_clock_;
};

#endif      // __JOINT_VELOCITY_CONTROLLER_HPP__
