#ifndef __CARTESIAN_VELOCITY_CONTROLLER_HPP__
#define __CARTESIAN_VELOCITY_CONTROLLER_HPP__

#include "controller_base/velocity_controller_base.hpp"
#include "hardware/hardware_manager.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "arm_controller/kinematics/jacobian_provider.hpp"
#include "arm_controller/utils/velocity_strict_solver.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <cmath>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <set>

/**
 * @brief 笛卡尔速度控制器
 * @details 通过二次规划（QP）将笛卡尔任务空间速度转换为关节速度命令。
 *          - 控制频率: 10Hz（dt = 0.01s）
 *          - 模式: 连续速度模式（订阅一次速度命令，持续运动直到收到新命令）
 *          - 安全机制: 三层安全检测（前置几何可行性、QP求解、后置方向验证）
 *          - 限制恢复: 允许从关节限制违规中恢复（反向运动离开限制）
 */
class CartesianVelocityController final
    : public VelocityControllerImpl<geometry_msgs::msg::TwistStamped> {
public:
    explicit CartesianVelocityController(const rclcpp::Node::SharedPtr& node);
    ~CartesianVelocityController() override;

    void start(const std::string& mapping) override;
    bool stop(const std::string& mapping) override;

    bool send_velocity(const std::string& mapping, const std::vector<double>& velocity) override;

private:
    // ===== ROS topic callback =====
    void velocity_callback(const std::string& mapping, const geometry_msgs::msg::TwistStamped::SharedPtr msg) override;

    // ===== IPC consumer thread =====
    void command_queue_consumer_thread() override;

    // RT loop
    void control_loop_rt(const std::string& mapping);

    // ===== 计算线程 =====
    // 处理所有重型计算：Jacobian、SVD、QP求解等
    // 不在RT线程中执行，避免RT超期
    void cartesian_computation_thread(const std::string& mapping);

    void initialize_moveit_adapter(const std::string& mapping);
    void initialize_jacobian_provider(const std::string& mapping);
    bool send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities);

private:
    std::shared_ptr<HardwareManager> hardware_manager_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;
    std::unordered_map<std::string, std::shared_ptr<arm_controller::kinematics::JacobianProvider>> jacobian_providers_;

    // ===== Command structure for RT processing =====
    struct TwistCommand {
        geometry_msgs::msg::TwistStamped twist;
        std::chrono::steady_clock::time_point stamp;
    };

    struct RtState {
        geometry_msgs::msg::TwistStamped target;
        std::chrono::steady_clock::time_point last_update;
        bool first_command_received = false;  // ✅ 是否收到过命令（用于初始化超时检测）
    };

    // ===== 计算线程结果结构体 =====
    struct ComputationResult {
        Eigen::VectorXd qd;        // 当前计算的关节速度
        Eigen::VectorXd qd_last;   // ✅ 上一次有效的关节速度（缓存）
        bool valid;                // 计算是否成功
        std::chrono::steady_clock::time_point timestamp;
    };

    // per mapping realtime
    std::unordered_map<std::string, std::unique_ptr<SPSCQueue<TwistCommand, 128>>> rt_buffers_;
    std::unordered_map<std::string, RtState> rt_states_;
    std::unordered_map<std::string, std::thread> rt_threads_;

    // Per-mapping RT线程运行标志（避免stop()关闭所有线程）
    std::unordered_map<std::string, std::shared_ptr<std::atomic<bool>>> rt_running_per_mapping_;

    // ===== 计算线程相关成员变量 =====
    // 计算结果（使用 unique_ptr 避免 mutex 复制问题）
    struct ComputationResultWithMutex {
        std::mutex mtx;
        ComputationResult result;
    };
    std::unordered_map<std::string, std::unique_ptr<ComputationResultWithMutex>>
        computation_results_;

    // 计算线程
    std::unordered_map<std::string, std::thread> computation_threads_;

    // 计算线程运行标志
    std::unordered_map<std::string, std::shared_ptr<std::atomic<bool>>>
        computation_running_per_mapping_;

    // ===== RT缓冲区保护 =====
    std::mutex rt_buffers_mutex_;

    // ===== RT状态保护（防止RT线程和计算线程的数据竞态） =====
    std::unordered_map<std::string, std::unique_ptr<std::mutex>> rt_states_mutexes_;

    // 队列消费者线程
    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};

    // TF2 坐标系转换
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // ✅ per-mapping 基座坐标系（支持双臂不同的基座）
    std::unordered_map<std::string, std::string> mapping_base_frames_;

    std::chrono::steady_clock steady_clock_;
    arm_controller::utils::VelocityStrictSolver solver_;

    // 保护 start/stop 的并发调用，避免重复创建订阅和线程
    std::mutex lifecycle_mutex_;
};

#endif      // __CARTESIAN_VELOCITY_CONTROLLER_HPP__
