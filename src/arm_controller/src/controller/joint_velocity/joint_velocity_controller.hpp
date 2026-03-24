#ifndef __JOINT_VELOCITY_CONTROLLER_HPP__
#define __JOINT_VELOCITY_CONTROLLER_HPP__

#include "controller_base/velocity_controller_base.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include <thread>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <map>
#include <set>

class JointVelocityController final
    : public VelocityControllerImpl<sensor_msgs::msg::JointState> {
public:
    explicit JointVelocityController(const rclcpp::Node::SharedPtr& node);
    ~JointVelocityController() override;

    void start(const std::string& mapping) override;
    bool stop(const std::string& mapping) override;

    bool send_velocity(const std::string& mapping, const std::vector<double>& velocity) override;

private:
    // ===== ROS topic callback =====
    void velocity_callback(const std::string& mapping, 
                           const sensor_msgs::msg::JointState::SharedPtr msg) override;
    
    // ===== IPC consumer thread =====
    void command_queue_consumer_thread() override;
    
    // RT loop
    void control_loop_rt(const std::string& mapping); 

    bool send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities);
    
private:
    std::shared_ptr<HardwareManager> hardware_manager_;

    struct RtCommand {
        std::vector<double> velocity;
        std::chrono::steady_clock::time_point stamp;
    };

    struct RtState {
        std::vector<double> target;
        std::chrono::steady_clock::time_point last_update;
    };

    // per mapping realtime
    std::unordered_map<std::string, std::unique_ptr<SPSCQueue<RtCommand, 128>>> rt_buffers_;
    std::unordered_map<std::string, RtState> rt_states_;
    std::unordered_map<std::string, std::thread> rt_threads_;

    // Per-mapping RT线程运行标志（避免stop()关闭所有线程）
    std::unordered_map<std::string, std::shared_ptr<std::atomic<bool>>> rt_running_per_mapping_;

    // ===== RT缓冲区保护 =====
    std::mutex rt_buffers_mutex_;

    // 队列消费者线程
    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};

    std::chrono::steady_clock steady_clock_;  // 用于命令超时检查的单调时钟

    // 保护 start/stop 的并发调用，避免重复创建订阅和RT线程
    std::mutex lifecycle_mutex_;
};

#endif      // __JOINT_VELOCITY_CONTROLLER_HPP__