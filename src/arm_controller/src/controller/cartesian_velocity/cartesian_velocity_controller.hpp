#ifndef __CARTESIAN_VELOCITY_CONTROLLER_HPP__
#define __CARTESIAN_VELOCITY_CONTROLLER_HPP__

#include "controller_base/velocity_controller_base.hpp"
#include "hardware/hardware_manager.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
#include "arm_controller/ipc/command_queue_ipc.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <cmath>
#include <thread>
#include <atomic>

/**
 * @brief 笛卡尔速度控制器
 * @details 通过二次规划（QP）将笛卡尔任务空间速度转换为关节速度命令。
 *          - 控制频率: 100Hz（dt = 0.01s）
 *          - 模式: 连续速度模式（订阅一次速度命令，持续运动直到收到新命令）
 *          - 安全机制: 三层安全检测（前置几何可行性、QP求解、后置方向验证）
 *          - 限制恢复: 允许从关节限制违规中恢复（反向运动离开限制）
 */
class CartesianVelocityController final
    : public VelocityControllerImpl<geometry_msgs::msg::TwistStamped> {
public:
    explicit CartesianVelocityController(const rclcpp::Node::SharedPtr& node);
    ~CartesianVelocityController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;
    
    bool move(const std::string& mapping, const std::vector<double>& parameters) override;

private:
    void initialize_moveit_service();
    bool send_joint_velocities(const std::string& mapping, const std::vector<double>& velocities);

    // 队列消费线程 - 后台处理来自C++ API的命令
    void command_queue_consumer_thread();

private:
    std::shared_ptr<HardwareManager> hardware_manager_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;

    // TF2 坐标系转换
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string base_frame_;  // 基座坐标系，通常是 "base_link"

    // 队列消费者线程
    std::unique_ptr<std::thread> queue_consumer_;
    std::atomic<bool> consumer_running_{false};
};

#endif      // __CARTESIAN_VELOCITY_CONTROLLER_HPP__
