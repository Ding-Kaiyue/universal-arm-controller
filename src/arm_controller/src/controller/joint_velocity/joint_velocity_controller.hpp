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
#include <set>

// JointVelocityController implements a latch-based velocity controller:
// - commands update cache only
// - fixed-rate 10ms control loop
// - timeout-based stop
class JointVelocityController final
    : public VelocityControllerImpl<sensor_msgs::msg::JointState> {
public:
    explicit JointVelocityController(const rclcpp::Node::SharedPtr& node);
    ~JointVelocityController() override;

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
        std::vector<double> last_cmd_velocity;        // 最新的速度命令
        std::chrono::steady_clock::time_point last_cmd_time;  // 最后一次收到命令的时间
        bool has_valid_command = false;               // 是否收到过有效命令
        bool timeout_triggered = false;               // 超时触发状态，用于去重
    };
    std::map<std::string, MappingControlState> mapping_states_;
    std::set<std::string> started_mappings_;  // ⭐ 追踪已启动的映射，避免重复调用 start()
    std::mutex cmd_mutex_;
    std::chrono::steady_clock steady_clock_;

    // 队列消费者线程
    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};
};

#endif      // __JOINT_VELOCITY_CONTROLLER_HPP__
