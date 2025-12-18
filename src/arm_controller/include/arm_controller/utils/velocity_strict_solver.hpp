#pragma once

#include <Eigen/Core>
#include <rclcpp/logger.hpp>
#include "hardware/hardware_manager.hpp"

namespace arm_controller::utils {

/**
 * @brief Strict Cartesian velocity solver (RM-style)
 *
 * Semantics:
 *  - Velocity expressed strictly in base/world frame
 *  - No direction modification, no jog bias
 *  - No crawl speed near singularity
 *  - If unreachable -> fail
 *  - Damped least squares only for numerical stability
 */
class VelocityStrictSolver
{
public:
    VelocityStrictSolver() = default;

    /**
     * @brief Solve qd such that J * qd == v_ee
     *
     * @return true  Cartesian velocity is achievable
     * @return false Cartesian velocity is unreachable
     */
    bool solve(
        const Eigen::MatrixXd& J,
        const Eigen::VectorXd& v_ee,
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& q_min_pos,
        const Eigen::VectorXd& q_max_pos,
        const Eigen::VectorXd& qd_max,
        Eigen::VectorXd& qd_out,
        const rclcpp::Logger& logger);

    bool check_workspace_boundary(
        const std::vector<double>& joint_positions,
        const Eigen::VectorXd& qd,
        const std::vector<std::string>& joint_names,
        const std::shared_ptr<HardwareManager>& hardware_manager,
        double dt);

private:
    static Eigen::MatrixXd dampedPseudoInverse(
        const Eigen::MatrixXd& J,
        double lambda);
};

}  // namespace arm_controller::utils
