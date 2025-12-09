#ifndef __JOINT_VELOCITY_CONTROLLER_HPP__
#define __JOINT_VELOCITY_CONTROLLER_HPP__

#include "controller_base/velocity_controller_base.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include <thread>
#include <atomic>

class JointVelocityController final
    : public VelocityControllerImpl<sensor_msgs::msg::JointState> {
public:
    explicit JointVelocityController(const rclcpp::Node::SharedPtr& node);
    ~JointVelocityController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;
    
    bool move(const std::string& mapping, const std::vector<double>& parameters) override;

private:
    bool send_joint_velocities(const std::string& mapping, const std::vector<double>& velocities);

    // 队列消费线程 - 后台处理来自C++ API的命令
    void command_queue_consumer_thread();

    // 硬件接口
    std::shared_ptr<HardwareManager> hardware_manager_;
    
    // 队列消费者线程
    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};
};

#endif      // __JOINT_VELOCITY_CONTROLLER_HPP__
