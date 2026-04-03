#include "obstacle_damper.hpp"

#include <cmath>
#include <limits>

namespace arm_controller::algorithm::reactive_qp {

namespace {
constexpr double kInfinity = 1e20;
}

int ObstacleDamper::countActiveRows(
    const std::vector<ObstacleConstraintInput>& constraints,
    const ObstacleDamperConfig& config,
    const int dof) {
    if (dof <= 0) {
        return 0;
    }
    if (config.influence_distance < 0.0 || config.safety_distance < 0.0 || config.cbf_gain <= 0.0) {
        return 0;
    }

    int rows = 0;
    for (const auto& c : constraints) {
        if (!std::isfinite(c.distance)) {
            continue;
        }
        if (c.normal_jacobian.size() != dof || !c.normal_jacobian.allFinite()) {
            continue;
        }

        if (c.distance <= config.influence_distance) {
            ++rows;
        }
    }
    return rows;
}

int ObstacleDamper::appendConstraints(
    const std::vector<ObstacleConstraintInput>& constraints,
    const ObstacleDamperConfig& config,
    Eigen::MatrixXd& A_qdot,
    Eigen::VectorXd& lb,
    Eigen::VectorXd& ub,
    int start_row) {
    if (start_row < 0) {
        return 0;
    }
    if (config.influence_distance < 0.0 || config.safety_distance < 0.0 || config.cbf_gain <= 0.0) {
        return 0;
    }
    if (lb.size() != A_qdot.rows() || ub.size() != A_qdot.rows()) {
        return 0;
    }
    const int dof = static_cast<int>(A_qdot.cols());
    int row = start_row;

    for (const auto& c : constraints) {
        if (!std::isfinite(c.distance)) {
            continue;
        }
        if (c.normal_jacobian.size() != dof || !c.normal_jacobian.allFinite()) {
            continue;
        }

        if (c.distance > config.influence_distance) {
            continue;
        }
        if (row >= A_qdot.rows()) {
            return row - start_row;
        }

        // Barrier function:
        //   h(q) = d(q) - safety_distance
        //
        // CBF condition:
        //   h_dot >= -gamma * h
        //
        // 因为:
        //   h_dot = d_dot = normal_jacobian * qdot
        //
        // 所以:
        //   normal_jacobian * qdot >= -gamma * (distance - safety_distance)
        const double h = c.distance - config.safety_distance;
        const double lower_bound = -config.cbf_gain * h;

        A_qdot.row(row).setZero();
        A_qdot.row(row) = c.normal_jacobian;
        lb(row) = lower_bound;
        ub(row) = kInfinity;
        ++row;
    }

    return row - start_row;
}

}  // namespace arm_controller::algorithm::reactive_qp
