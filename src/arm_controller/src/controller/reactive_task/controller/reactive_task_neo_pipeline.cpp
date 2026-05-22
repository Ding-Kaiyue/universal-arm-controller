#include "controller/reactive_task/controller/reactive_task_neo_pipeline.hpp"

#include <algorithm>

namespace arm_controller::controller::reactive_task {

namespace {
constexpr double kTerminalBranchJointWeight = 5.0;
constexpr double kLocalTrajoptBranchJointWeight = 7.0;
}

bool ReactiveTaskNeoPipeline::prepare(const PrepareInput& input, PrepareOutput* output) const {
    if (output == nullptr || input.jacobian_provider == nullptr || input.task_out == nullptr ||
        input.phase_flags == nullptr || input.exec_ctx == nullptr) {
        return false;
    }

    *output = PrepareOutput{};
    output->J =
        input.jacobian_provider->computeJacobian(input.arm_state.q, "", Eigen::Vector3d::Zero());
    if (output->J.rows() != 6 || output->J.cols() != input.arm_state.q.size()) {
        return false;
    }

    output->J_task = output->J;

    output->manipulability_gradient = Eigen::VectorXd::Zero(input.arm_state.q.size());
    if (input.manipulability_gradient != nullptr && input.manipulability_cfg != nullptr) {
        double log_m = 0.0;
        const bool ok_manip = input.manipulability_gradient->compute(
            input.arm_state.q,
            *input.manipulability_cfg,
            output->manipulability_gradient,
            &log_m);
        if (!ok_manip) {
            output->manipulability_gradient.setZero();
        }
    }

    output->posture_joint_weights = Eigen::VectorXd::Ones(input.arm_state.q.size());
    const bool has_local_joint_target =
        input.local_planner_joint_target != nullptr &&
        input.local_planner_joint_target->size() == input.arm_state.q.size() &&
        input.local_planner_joint_target->allFinite();
    if (has_local_joint_target) {
        const bool tracking_local_trajopt_branch =
            input.phase_flags->local_trajopt_tracking &&
            !input.phase_flags->terminal_goal_tracking;
        output->posture_joint_weights.setConstant(
            tracking_local_trajopt_branch ? kLocalTrajoptBranchJointWeight
                                          : kTerminalBranchJointWeight);
    }
    ReactiveTaskTerminalPolicy::PostureReferenceInput posture_input;
    posture_input.flags = *input.phase_flags;
    posture_input.arm_state = input.arm_state;
    posture_input.path_follow_joint_anchor_sample =
        input.path_follow_joint_anchor_sample_valid ? input.path_follow_joint_anchor_sample : nullptr;
    posture_input.path_follow_joint_anchor_sample_valid =
        input.path_follow_joint_anchor_sample_valid;
    posture_input.current_sample = input.current_sample;
    posture_input.local_planner_joint_target = input.local_planner_joint_target;
    posture_input.local_planner_joint_target_dt_sec = input.local_planner_joint_target_dt_sec;
    posture_input.joint_preference_cfg = input.joint_preference_cfg;
    ReactiveTaskTerminalPolicy terminal_policy;
    output->posture_qdot_ref = terminal_policy.buildPostureReference(posture_input);

    output->qp_input.q_current = input.arm_state.q;
    output->qp_input.jacobian_task = output->J_task;
    output->qp_input.desired_twist = input.task_out->v_des;
    output->qp_input.manipulability_gradient = output->manipulability_gradient;
    output->qp_input.posture_velocity_reference = output->posture_qdot_ref;
    if (input.exec_ctx->previous_qdot_reference_valid &&
        input.exec_ctx->previous_qdot_reference.size() == input.arm_state.q.size()) {
        output->qp_input.previous_qdot_reference = input.exec_ctx->previous_qdot_reference;
    }
    output->qp_input.posture_joint_weights = output->posture_joint_weights;
    output->qp_input.qd_min = input.arm_state.qd_min;
    output->qp_input.qd_max = input.arm_state.qd_max;
    output->qp_input.joint_limits.q_min = input.arm_state.joint_limits.q_min;
    output->qp_input.joint_limits.q_max = input.arm_state.joint_limits.q_max;
    output->ok = true;
    return true;
}

bool ReactiveTaskNeoPipeline::solve(const SolveInput& input, SolveOutput* output) const {
    if (output == nullptr || input.qp_input == nullptr || input.qp_build_cfg == nullptr ||
        input.solver == nullptr || input.task_jacobian == nullptr ||
        input.desired_twist == nullptr) {
        return false;
    }
    *output = SolveOutput{};
    rq::ReactiveQpProblem problem;
    std::string error;
    if (!rq::ReactiveQpBuilder::build(*input.qp_input, *input.qp_build_cfg, problem, &error)) {
        RCLCPP_WARN(
            input.logger,
            "[%s] neo build QP failed at tick %d: %s",
            input.mapping.c_str(),
            input.planner_tick,
            error.c_str());
        return false;
    }

    Eigen::VectorXd solution;
    if (!input.solver->solve(problem, solution, &error)) {
        RCLCPP_WARN(
            input.logger,
            "[%s] neo solve QP failed at tick %d: %s (nv=%d nc=%d obstacle_constraints=%zu)",
            input.mapping.c_str(),
            input.planner_tick,
            error.c_str(),
            problem.numVariables(),
            problem.numConstraints(),
            input.qp_input->obstacle_constraints.size());
        return false;
    }

    const int dof = static_cast<int>(input.arm_state.qd_min.size());
    output->qdot_cmd.assign(static_cast<std::size_t>(dof), 0.0);
    output->qdot_eigen = Eigen::VectorXd::Zero(dof);
    for (int i = 0; i < dof; ++i) {
        output->qdot_cmd[static_cast<std::size_t>(i)] = solution(i);
        output->qdot_eigen(i) = solution(i);
    }
    if (!output->qdot_eigen.allFinite()) {
        RCLCPP_ERROR(
            input.logger,
            "[%s] neo abort: QP produced non-finite qdot.",
            input.mapping.c_str());
        return false;
    }
    for (int i = 0; i < dof; ++i) {
        output->qdot_eigen(i) =
            std::clamp(output->qdot_eigen(i), input.arm_state.qd_min(i), input.arm_state.qd_max(i));
        output->qdot_cmd[static_cast<std::size_t>(i)] = output->qdot_eigen(i);
    }

    output->task_pred = *input.task_jacobian * output->qdot_eigen;
    output->task_residual = output->task_pred - *input.desired_twist;
    output->task_residual_norm = output->task_residual.norm();
    output->ok = true;
    return true;
}

}  // namespace arm_controller::controller::reactive_task
