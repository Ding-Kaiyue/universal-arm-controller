#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <string>
#include "arm_controller/hardware/hardware_manager.hpp"

namespace arm_controller::basic_ops {

class BasicOpsIPCService {
public:
    BasicOpsIPCService(const rclcpp::Node::SharedPtr& node,
                       const std::shared_ptr<HardwareManager>& hardware_manager);
    ~BasicOpsIPCService();

    void start();
    void stop();

private:
    void gripper_consumer_loop();
    void motor_enable_consumer_loop();
    void motor_disable_consumer_loop();

    bool execute_gripper_position_command(const std::string& mapping,
                                          int gripper_type,
                                          uint8_t position_raw,
                                          uint8_t velocity_raw,
                                          uint8_t effort_raw);
    void update_execution_state(const std::string& mapping, bool success);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<HardwareManager> hardware_manager_;
    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> gripper_thread_;
    std::unique_ptr<std::thread> motor_enable_thread_;
    std::unique_ptr<std::thread> motor_disable_thread_;
};

}  // namespace arm_controller::basic_ops
