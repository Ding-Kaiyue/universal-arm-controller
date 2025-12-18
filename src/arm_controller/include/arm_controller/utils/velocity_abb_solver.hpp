#pragma once

#include <Eigen/Core>
#include <rclcpp/logger.hpp>
#include "hardware/hardware_manager.hpp"


namespace arm_controller::utils {

/**
 * @brief ABB-style Cartesian Jog velocity solver
 *
 * Characteristics:
 *  - Direction-preserving (never deviates)
 *  - Continuous deceleration near singularity
 *  - Never hard-stop (crawl speed guaranteed)
 *  - Damped least squares with adaptive damping
 *  - Joint limit safe
 */
class VelocityABBSolver
{
public:
    VelocityABBSolver()
        : scale_state_(1.0) {} 

    /**
     * @brief Solve joint velocity for Cartesian jog command
     *
     * @param J        Jacobian (task_dim x dof)
     * @param v_ee     Desired Cartesian velocity (base frame)
     * @param qd_out   Output joint velocity
     * @param qd_min   Joint velocity lower bounds
     * @param qd_max   Joint velocity upper bounds
     * @param logger   ROS logger
     * @return true if solution valid
     */
    bool solve(
        const Eigen::MatrixXd& J,
        const Eigen::VectorXd& v_ee,
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& q_min_pos,
        const Eigen::VectorXd& q_max_pos,
        Eigen::VectorXd& qd_out,
        const Eigen::VectorXd& qd_max,
        const rclcpp::Logger& logger);


    bool check_workspace_boundary(
        const std::vector<double>& joint_positions,
        const Eigen::VectorXd& qd,
        const std::vector<std::string>& joint_names,
        const std::shared_ptr<HardwareManager>& hardware_manager,
        double dt);

    Eigen::VectorXd jointLimitGradient(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& q_min,
        const Eigen::VectorXd& q_max);

private:
    /* ================= Internal State ================= */

    // Velocity scale smoothing state (ABB exponential decay behavior)
    double scale_state_;

    /* ================= Internal Helpers ================= */

    static double minSingularValue(const Eigen::MatrixXd& J);

    static double abbVelocityScale(double sigma_min);

    static Eigen::MatrixXd abbDampedPseudoInverse(
        const Eigen::MatrixXd& J,
        double sigma_min);
};

}  // namespace arm_controller::utils
