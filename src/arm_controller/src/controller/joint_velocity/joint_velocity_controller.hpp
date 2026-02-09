#ifndef __JOINT_VELOCITY_CONTROLLER_HPP__
#define __JOINT_VELOCITY_CONTROLLER_HPP__

#include "controller_base/velocity_controller_base.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <map>

class JointVelocityController final
    : public VelocityControllerImpl<sensor_msgs::msg::JointState> {
public:
    explicit JointVelocityController(const rclcpp::Node::SharedPtr& node);
    ~JointVelocityController() override = default;

    void start(const std::string& mapping) override;
    bool stop(const std::string& mapping) override;

    bool send_velocity(const std::string& mapping, const std::vector<double>& velocity) override;

private:
    void velocity_callback(const std::string& mapping, const sensor_msgs::msg::JointState::SharedPtr msg) override;
    bool send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities);

    void control_loop(const std::string& mapping);  // 10ms 控制循环

    void command_queue_consumer_thread() override;

    std::shared_ptr<HardwareManager> hardware_manager_;

    // Per-mapping 实时控制循环（支持多臂）
    struct MappingControlState {
        rclcpp::TimerBase::SharedPtr control_timer;
        sensor_msgs::msg::JointState last_cmd;
        std::chrono::steady_clock::time_point last_cmd_time;
    };
    std::map<std::string, MappingControlState> mapping_states_;
    std::mutex cmd_mutex_;
    std::chrono::steady_clock steady_clock_;

    // 队列消费者线程
    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};
};

#endif      // __JOINT_VELOCITY_CONTROLLER_HPP__