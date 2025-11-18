#ifndef __CARTESIAN_VELOCITY_CONTROLLER_HPP__
#define __CARTESIAN_VELOCITY_CONTROLLER_HPP__

#include "controller_base/velocity_controller_base.hpp"
#include "hardware/hardware_manager.hpp"
#include "trajectory_planning_v3/infrastructure/integration/moveit_adapter.hpp"
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

    void start(const std::string& mapping) override;
    bool stop(const std::string& mapping) override;

private:
    void initialize_moveit_service();
    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) override;
    bool solve_velocity_qp(const std::string& mapping,
                           const Eigen::MatrixXd& J,
                           const Eigen::VectorXd& v_ee,
                           Eigen::VectorXd& qd_solution,
                           const std::vector<std::string>& joint_names);
    bool send_joint_velocities(const std::string& mapping, const std::vector<double>& joint_velocities);
    /**
     * @brief 检查工作空间边界约束
     * @details 预测下一个关节位置，确保不会超出关节限制。允许从限制违规中恢复。
     *          dt: 控制周期（秒），默认 0.01s = 10ms（对应 100Hz 控制频率）
     *          注意：此方法假设控制器以 100Hz 运行。如果控制频率不同，需要相应调整 dt。
     * @param joint_positions 当前关节位置（弧度）
     * @param qd_command 关节速度命令（弧度/秒）
     * @param dt 控制周期，用于预测下一位置（秒）。默认 0.01s 对应 100Hz 控制频率。
     * @return true 如果预测位置不违反关节限制；false 如果会违反限制
     */
    bool check_workspace_boundary(const std::string& mapping,
                                  const std::vector<double>& joint_positions,
                                  const Eigen::VectorXd& qd_command,
                                  double dt = 0.01);

    // 几何可行性检测（前置）
    void check_direction_feasibility(
        const Eigen::MatrixXd &J,
        const Eigen::VectorXd &v_ee,
        double &out_feasibility_ratio,
        double &out_residual_norm,
        double &out_alignment,
        double &out_cond) const;

    // 方向一致性验证（后置）
    bool post_verify_direction(
        const std::string& mapping,
        const Eigen::MatrixXd &J,
        const Eigen::VectorXd &qd_solution,
        const Eigen::VectorXd &v_ee,
        double allowed_angle_deg) const;

private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
    std::shared_ptr<HardwareManager> hardware_manager_;
    std::map<std::string, std::shared_ptr<trajectory_planning::infrastructure::integration::MoveItAdapter>> moveit_adapters_;
    bool is_active_ = false;
    std::string active_mapping_;

    // TF2 坐标系转换
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string base_frame_;  // 基座坐标系，通常是 "base_link"
    std::string reference_frame_;  // 参考坐标系：命令发布时刻的base_link（固定）
    Eigen::Matrix3d R_world_to_ref_;  // 旋转矩阵：从world到参考坐标系
    bool has_reference_frame_ = false;  // 是否已设置参考坐标系

    // ========== 安全性检测阈值 ==========
    // 前置几何可行性检测（SVD 投影）
    static constexpr double FEASIBILITY_RATIO_THRESHOLD = 0.95;        // 可达分量比例 >= 95%
    static constexpr double RESIDUAL_RATIO_THRESHOLD = 0.1;            // 不可达分量 <= 10% * ||v||
    static constexpr double ALIGNMENT_THRESHOLD = 0.98;                // SVD 投影对齐度 >= 0.98
    static constexpr double CONDITION_NUMBER_THRESHOLD = 1e6;          // 条件数 <= 1e6

    // 后置方向验证
    static constexpr double ALLOWED_ANGLE_DEG = 5.0;                   // 允许 5° 的方向偏差
    static constexpr double DIRECTION_COS_THRESHOLD = 0.99619;         // cos(5°) ≈ 0.99619

    // QP 求解参数 - 数值稳定性和正则化
    // eps: 关节速度正则化权重。较大值（如1e-4）对高自由度机器人更稳定，
    //      较小值（如1e-6）对低自由度机器人更精确。
    // eps_s: 缩放变量的正则化，极小值确保 s 优化优先级
    // epsilon_sv: SVD 奇异值阈值，用于判断数值精度和奇异性
    static constexpr double QDOT_REGULARIZATION = 1e-6;                // q̇ 的 Hessian 正则化权重
    static constexpr double EPSILON_S = 1e-8;                          // s 变量的数值稳定性项
    static constexpr double EPSILON_SV = 1e-12;                        // SVD 奇异值数值精度阈值
};

#endif      // __CARTESIAN_VELOCITY_CONTROLLER_HPP__
