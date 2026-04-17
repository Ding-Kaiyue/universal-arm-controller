#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "hessian_builder.hpp"
#include "joint_limit_adapter.hpp"
#include "obstacle_damper.hpp"
#include "reactive_qp_problem.hpp"

namespace arm_controller::algorithm::reactive_qp {

struct ReactiveQpBuildConfig {
    HessianBuilderConfig hessian;
    JointLimitDamperConfig joint_limit_damper;
    ObstacleDamperConfig obstacle_damper;
    bool enable_joint_limit_damper{true};
    bool enable_obstacle_damper{true};
    double slack_abs_bound{1.0};
};

struct ReactiveQpBuildInput {
    // Current joint state q in R^n
    Eigen::VectorXd q_current;

    // Primary task: J * qdot + s ~= v_des
    Eigen::MatrixXd jacobian_task;
    Eigen::VectorXd desired_twist;

    // Gradient of log manipulability, i.e. ∇log m(q), in R^n.
    Eigen::VectorXd manipulability_gradient;
    // Optional posture reference qdot_ref and per-joint posture weights.
    // If empty, posture term is effectively disabled unless config assigns defaults.
    Eigen::VectorXd posture_velocity_reference;
    Eigen::VectorXd posture_joint_weights;

    // Joint velocity bounds: qd_min <= qdot <= qd_max
    Eigen::VectorXd qd_min;
    Eigen::VectorXd qd_max;

    // Joint position bounds used by joint-limit CBF.
    JointLimitData joint_limits;
    std::vector<ObstacleConstraintInput> obstacle_constraints;
};

class ReactiveQpBuilder {
public:
    // Build standard QP:
    //   min_x  0.5 x^T H x + g^T x
    //   s.t.   l <= A x <= u
    //
    // Decision variable:
    //   x = [qdot; s], qdot in R^n, s in R^m.
    //
    // Objective terms:
    //   ||J qdot + s - v_des||^2, ||qdot||^2, ||s||^2, -wm*(∇log m)^T qdot.
    //
    // Constraint blocks:
    //   1) joint velocity bounds
    //   2) slack bounds
    //   3) joint-limit CBF inequalities
    //   4) optional obstacle dampers
    static bool build(
        const ReactiveQpBuildInput& input,
        const ReactiveQpBuildConfig& config,
        ReactiveQpProblem& out_problem,
        std::string* error = nullptr);
};

}  // namespace arm_controller::algorithm::reactive_qp
