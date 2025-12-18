#ifndef __VELOCITY_QP_SOLVER_HPP__
#define __VELOCITY_QP_SOLVER_HPP__

#include <Eigen/Dense>
#include <memory>
#include <rclcpp/logger.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include "hardware/hardware_manager.hpp"

namespace arm_controller::utils {

class VelocityQPSolver {
public:
    // /**
    //  * @brief 前置几何可行性检测
    //  * @details 通过 SVD 分解检测速度方向是否在 Jacobian 列空间内
    //  */
    // static void check_direction_feasibility(
    //     const Eigen::MatrixXd &J,
    //     const Eigen::VectorXd &v_ee,
    //     double &out_feasibility_ratio,
    //     double &out_residual_norm,
    //     double &out_alignment,
    //     double &out_cond);

    // /**
    //  * @brief 后置方向一致性验证
    //  * @details 验证 QP 求解后的实际末端速度是否与期望方向一致
    //  * @param J 雅可比矩阵
    //  * @param qd_solution 关节速度解
    //  * @param v_ee 期望末端速度
    //  * @param allowed_angle_deg 允许的最大角度偏差（度）
    //  * @param logger ROS 日志记录器
    //  * @return true 如果方向偏差 <= allowed_angle_deg；false 如果偏差过大
    //  */
    // static bool post_verify_direction(
    //     const Eigen::MatrixXd &J,
    //     const Eigen::VectorXd &qd_solution,
    //     const Eigen::VectorXd &v_ee,
    //     double allowed_angle_deg,
    //     const rclcpp::Logger& logger);

    /**
     * @brief 检查工作空间边界约束
     * @details 预测下一个关节位置，确保不会超出关节限制
     * @param joint_positions 当前关节位置（弧度）
     * @param qd 关节速度命令（弧度/秒）
     * @param joint_names 关节名称列表
     * @param hardware_manager 硬件管理器指针
     * @param dt 控制周期（秒），默认 0.01s
     * @return true 如果预测位置不违反关节限制；false 如果会违反限制
     */
    bool check_workspace_boundary(
        const std::vector<double>& joint_positions,
        const Eigen::VectorXd& qd,
        const std::vector<std::string>& joint_names,
        const std::shared_ptr<HardwareManager>& hardware_manager,
        double dt = 0.01);
    
    Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& J, double lambda);

    double computeMinSingularValue(const Eigen::MatrixXd& J);
    /**
     * @brief 求解速度 QP
     * @details 软约束 IK with 缩放变量：
     *   min  (1/2)ε·||q̇||² - s
     *   s.t. J·q̇ - s·v_ee = 0
     *        q̇_min ≤ q̇ ≤ q̇_max
     *        0 ≤ s ≤ 1
     *
     * @param J 雅可比矩阵（行数=任务维数，列数=自由度）
     * @param v_ee 期望末端速度（6维：线速度+角速度）
     * @param qd_out 输出的关节速度解
     * @param qd_min 关节速度最小值
     * @param qd_max 关节速度最大值
     * @param logger ROS 日志记录器
     * @return true 如果 QP 求解成功；false 如果求解失败
     */
    bool solve_velocity_qp(
        const Eigen::MatrixXd &J,
        const Eigen::VectorXd &v_ee,
        Eigen::VectorXd &qd_out,
        const Eigen::VectorXd &qd_min,
        const Eigen::VectorXd &qd_max,
        const rclcpp::Logger& logger);
    
    bool solve_velocity_hqp(
        const Eigen::MatrixXd& J,
        const Eigen::VectorXd& v_ee,
        const Eigen::VectorXd& q,
        Eigen::VectorXd& qd_out,
        const Eigen::VectorXd& qd_min,
        const Eigen::VectorXd& qd_max,
        const std::vector<std::string>& joint_names,
        const std::shared_ptr<HardwareManager>& hardware_manager,
        const rclcpp::Logger& logger);

private:
    // Cache for smoothness: previous joint velocities
    static inline Eigen::VectorXd qd_prev_;

    // OSQP Solver instance (member variable for warm start across iterations)
    // This is crucial for high-frequency Jog control (>100 Hz)
    // Warm start significantly reduces convergence time and prevents lambda jumps
    mutable OsqpEigen::Solver hqp_solver_;
    mutable Eigen::VectorXd x_prev_;  // Previous solution for warm start
    mutable bool solver_initialized_ = false;
};

} // namespace arm_controller::utils

#endif  // __VELOCITY_QP_SOLVER_HPP__