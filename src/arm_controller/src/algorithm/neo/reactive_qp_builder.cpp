#include "reactive_qp_builder.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "reactive_qp_validator.hpp"

namespace arm_controller::algorithm::reactive_qp {

namespace {
constexpr double kInfinity = 1e20;
constexpr double kEps = 1e-9;

bool validateJointVelocityAndCbfCompatibility(
    const ReactiveQpBuildInput& input,
    const ReactiveQpBuildConfig& config,
    std::string* error) {
    const int dof = static_cast<int>(input.q_current.size());
    if (input.qd_min.size() != dof || input.qd_max.size() != dof) {
        if (error != nullptr) {
            *error = "qd_min/qd_max size mismatch in joint-CBF compatibility check.";
        }
        return false;
    }

    for (int i = 0; i < dof; ++i) {
        double cbf_lb = -kInfinity;
        double cbf_ub = kInfinity;

        const double q = input.q_current(i);
        const double q_min = input.joint_limits.q_min(i);
        const double q_max = input.joint_limits.q_max(i);

        if (!(q_min < q_max)) {
            continue;
        }

        const double q_low_safe = q_min + config.joint_limit_damper.safety_distance;
        const double q_up_safe = q_max - config.joint_limit_damper.safety_distance;
        if (!(q_low_safe < q_up_safe - kEps)) {
            continue;
        }

        const double q_low_inf = q_min + config.joint_limit_damper.influence_distance;
        const double q_up_inf = q_max - config.joint_limit_damper.influence_distance;

        if (q <= q_low_inf) {
            const double h_low = q - q_low_safe;
            const double lower_bound = -config.joint_limit_damper.cbf_gain_lower * h_low;
            cbf_lb = std::max(cbf_lb, lower_bound);
        }
        if (q >= q_up_inf) {
            const double h_up = q_up_safe - q;
            const double upper_bound = config.joint_limit_damper.cbf_gain_upper * h_up;
            cbf_ub = std::min(cbf_ub, upper_bound);
        }

        const double merged_lb = std::max(input.qd_min(i), cbf_lb);
        const double merged_ub = std::min(input.qd_max(i), cbf_ub);
        if (merged_lb > merged_ub + kEps) {
        if (error != nullptr) {
            std::ostringstream oss;
            oss << "Infeasible joint " << i
                << ": velocity bounds [" << input.qd_min(i) << ", " << input.qd_max(i)
                << "] conflict with CBF bounds [" << cbf_lb << ", " << cbf_ub
                << "], merged interval [" << merged_lb << ", " << merged_ub << "].";
            *error = oss.str();
        }
        return false;
        }
    }
    return true;
}

bool validateObstacleVelocityCompatibility(
    const ReactiveQpBuildInput& input,
    const ReactiveQpBuildConfig& config,
    std::string* error) {
    const int dof = static_cast<int>(input.q_current.size());
    if (input.qd_min.size() != dof || input.qd_max.size() != dof) {
        if (error != nullptr) {
            *error = "qd_min/qd_max size mismatch in obstacle-CBF compatibility check.";
        }
        return false;
    }
    if (config.obstacle_damper.influence_distance < 0.0 ||
        config.obstacle_damper.safety_distance < 0.0 ||
        config.obstacle_damper.cbf_gain <= 0.0) {
        if (error != nullptr) {
            *error = "Invalid obstacle CBF config.";
        }
        return false;
    }

    for (std::size_t i = 0; i < input.obstacle_constraints.size(); ++i) {
        const auto& c = input.obstacle_constraints[i];
        if (!std::isfinite(c.distance) ||
            c.distance > config.obstacle_damper.influence_distance ||
            c.normal_jacobian.size() != dof ||
            !c.normal_jacobian.allFinite()) {
            continue;
        }

        const double h = c.distance - config.obstacle_damper.safety_distance;
        const double lower_bound = -config.obstacle_damper.cbf_gain * h;

        // Max possible value of a^T qdot under box constraints qd_min <= qdot <= qd_max.
        double lhs_max = 0.0;
        for (int j = 0; j < dof; ++j) {
            const double a = c.normal_jacobian(j);
            lhs_max += (a >= 0.0) ? a * input.qd_max(j) : a * input.qd_min(j);
        }

        if (lhs_max + kEps < lower_bound) {
            if (error != nullptr) {
                std::ostringstream oss;
                oss << "Infeasible obstacle CBF at index " << i
                    << ": max(nJ*qdot)=" << lhs_max
                    << " < required lower bound " << lower_bound
                    << ", distance=" << c.distance
                    << ", safety_distance=" << config.obstacle_damper.safety_distance << ".";
                *error = oss.str();
            }
            return false;
        }
    }

    return true;
}
}

bool ReactiveQpBuilder::build(
    const ReactiveQpBuildInput& input,
    const ReactiveQpBuildConfig& config,
    ReactiveQpProblem& out_problem,
    std::string* error) {
    if (!ReactiveQpValidator::validateTaskInput(
            input.jacobian_task,
            input.desired_twist,
            input.q_current,
            input.qd_min,
            input.qd_max,
            input.joint_limits.q_min,
            input.joint_limits.q_max,
            error)) {
        return false;
    }
    if (config.enable_joint_limit_damper &&
        !validateJointVelocityAndCbfCompatibility(input, config, error)) {
        return false;
    }
    if (config.enable_obstacle_damper &&
        !validateObstacleVelocityCompatibility(input, config, error)) {
        return false;
    }

    const int dof = static_cast<int>(input.q_current.size());
    const int task_dim = static_cast<int>(input.jacobian_task.rows());
    // Decision variable: x = [qdot; s], qdot in R^dof, s in R^task_dim.
    const int nv = dof + task_dim;

    HessianBuildInput hessian_input;
    hessian_input.jacobian_task = input.jacobian_task;
    hessian_input.desired_twist = input.desired_twist;
    hessian_input.manipulability_gradient = input.manipulability_gradient;
    if (!HessianBuilder::build(
            hessian_input, config.hessian, out_problem.hessian, out_problem.gradient)) {
        if (error != nullptr) {
            *error = "Failed to build Hessian/gradient.";
        }
        return false;
    }

    const int joint_limit_rows =
        config.enable_joint_limit_damper
            ? JointLimitDamper::countActiveRows(
                    input.q_current, input.joint_limits, config.joint_limit_damper)
            : 0;
    const int obstacle_rows =
        config.enable_obstacle_damper
            ? ObstacleDamper::countActiveRows(
                    input.obstacle_constraints, config.obstacle_damper, dof)
            : 0;
    // Constraint rows in standard form l <= A x <= u:
    //   block 1 (dof rows): qd_min <= qdot <= qd_max
    //   block 2 (task_dim rows): -s_max <= s <= s_max
    //   block 3: joint-limit CBF (on qdot block only)
    //   block 4: optional obstacle dampers (on qdot block only)
    const int base_bound_rows = nv;
    const int nc = base_bound_rows + joint_limit_rows + obstacle_rows;

    out_problem.constraint_matrix = Eigen::MatrixXd::Zero(nc, nv);
    out_problem.lower_bound = Eigen::VectorXd::Constant(nc, -kInfinity);
    out_problem.upper_bound = Eigen::VectorXd::Constant(nc, kInfinity);

    // Base variable bounds (joint velocity + slack bounds).
    for (int i = 0; i < dof; ++i) {
        out_problem.constraint_matrix(i, i) = 1.0;
        out_problem.lower_bound(i) = input.qd_min(i);
        out_problem.upper_bound(i) = input.qd_max(i);
    }
    for (int i = 0; i < task_dim; ++i) {
        const int row = dof + i;
        const int col = dof + i;
        out_problem.constraint_matrix(row, col) = 1.0;
        out_problem.lower_bound(row) = -std::abs(config.slack_abs_bound);
        out_problem.upper_bound(row) = std::abs(config.slack_abs_bound);
    }

    int next_row = base_bound_rows;
    if (joint_limit_rows > 0) {
        Eigen::MatrixXd A_qdot = Eigen::MatrixXd::Zero(joint_limit_rows, dof);
        Eigen::VectorXd lb = Eigen::VectorXd::Zero(joint_limit_rows);
        Eigen::VectorXd ub = Eigen::VectorXd::Zero(joint_limit_rows);
        const int written = JointLimitDamper::appendConstraints(
            input.q_current,
            input.joint_limits,
            config.joint_limit_damper,
            A_qdot,
            lb,
            ub,
            0);
        if (written < 0 || written > joint_limit_rows) {
            if (error != nullptr) {
                *error = "Joint-limit constraint writer produced invalid row count.";
            }
            return false;
        }
        out_problem.constraint_matrix.block(next_row, 0, written, dof) = A_qdot.topRows(written);
        out_problem.lower_bound.segment(next_row, written) = lb.head(written);
        out_problem.upper_bound.segment(next_row, written) = ub.head(written);
        next_row += written;
    }

    if (obstacle_rows > 0) {
        Eigen::MatrixXd A_qdot = Eigen::MatrixXd::Zero(obstacle_rows, dof);
        Eigen::VectorXd lb = Eigen::VectorXd::Zero(obstacle_rows);
        Eigen::VectorXd ub = Eigen::VectorXd::Zero(obstacle_rows);
        const int written = ObstacleDamper::appendConstraints(
            input.obstacle_constraints,
            config.obstacle_damper,
            A_qdot,
            lb,
            ub,
            0);
        if (written < 0 || written > obstacle_rows) {
            if (error != nullptr) {
                *error = "Obstacle constraint writer produced invalid row count.";
            }
            return false;
        }
        out_problem.constraint_matrix.block(next_row, 0, written, dof) = A_qdot.topRows(written);
        out_problem.lower_bound.segment(next_row, written) = lb.head(written);
        out_problem.upper_bound.segment(next_row, written) = ub.head(written);
        next_row += written;
    }

    // Hardening: keep only actually written rows, even if countActiveRows and
    // appendConstraints diverge in future edits.
    if (next_row < nc) {
        out_problem.constraint_matrix = out_problem.constraint_matrix.topRows(next_row);
        out_problem.lower_bound = out_problem.lower_bound.head(next_row);
        out_problem.upper_bound = out_problem.upper_bound.head(next_row);
    }

    if (!ReactiveQpValidator::validateProblem(out_problem, error)) {
        return false;
    }
    return true;
}

}  // namespace arm_controller::algorithm::reactive_qp
