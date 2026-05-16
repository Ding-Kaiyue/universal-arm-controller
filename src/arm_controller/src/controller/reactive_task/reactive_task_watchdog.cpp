#include "controller/reactive_task/reactive_task_watchdog.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace arm_controller::controller::reactive_task {

ReactiveTaskWatchdog::ReactiveTaskWatchdog()
    : ReactiveTaskWatchdog(Config{}) {}

ReactiveTaskWatchdog::ReactiveTaskWatchdog(Config cfg)
    : cfg_(std::move(cfg)) {}

bool ReactiveTaskWatchdog::shouldApplyPrimaryRecovery(
    const ReactiveTaskExecutionContext& exec_ctx,
    const double pos_err_goal,
    const double ori_err_goal,
    const int no_motion_cycle_limit,
    const int no_progress_cycle_limit) const {
    return (exec_ctx.no_motion_cycles >= std::max(8, no_motion_cycle_limit / 4) ||
            exec_ctx.no_progress_cycles >= std::max(12, no_progress_cycle_limit / 8)) &&
           pos_err_goal <= std::max(0.03, 3.0 * cfg_.goal_position_tolerance) &&
           ori_err_goal <= std::max(0.10, cfg_.goal_orientation_tolerance_rad + 0.03);
}

void ReactiveTaskWatchdog::update(
    ReactiveTaskExecutionContext& exec_ctx,
    const UpdateInput& input,
    const int no_motion_cycle_limit,
    const int no_progress_cycle_limit,
    UpdateOutput* output) const {
    if (output == nullptr) {
        return;
    }

    *output = UpdateOutput{};
    output->qdot_max_abs = input.qdot_eigen.cwiseAbs().maxCoeff();
    output->qdot_norm = input.qdot_eigen.norm();
    output->qdot_delta_norm =
        (exec_ctx.previous_qdot_reference_valid &&
         exec_ctx.previous_qdot_reference.size() == input.qdot_eigen.size())
            ? (input.qdot_eigen - exec_ctx.previous_qdot_reference).norm()
            : 0.0;
    output->qdot_limit_violation =
        ((input.qdot_eigen.array() < input.qd_min.array() - 1e-9) ||
         (input.qdot_eigen.array() > input.qd_max.array() + 1e-9))
            .any();
    exec_ctx.last_qdot_limit_violation = output->qdot_limit_violation;

    const bool task_error_moving =
        exec_ctx.previous_task_error_valid &&
        (std::abs(input.pos_err_goal - exec_ctx.previous_pos_err) >= cfg_.pos_err_delta_min ||
         std::abs(input.ori_err_goal - exec_ctx.previous_ori_err) >= cfg_.ori_err_delta_min);
    if (exec_ctx.q_prev_feedback_valid && exec_ctx.q_prev_feedback.size() == input.q_now.size()) {
        const double joint_delta_max =
            (input.q_now - exec_ctx.q_prev_feedback).cwiseAbs().maxCoeff();
        const double observed_motion_threshold = std::max(
            cfg_.joint_delta_min_abs,
            cfg_.joint_delta_response_ratio * output->qdot_max_abs * std::max(1e-4, input.control_dt_sec));
        if ((output->qdot_norm >= cfg_.cmd_norm_min ||
             output->qdot_max_abs >= cfg_.cmd_max_joint_min) &&
            joint_delta_max < observed_motion_threshold && !task_error_moving) {
            ++exec_ctx.no_motion_cycles;
        } else {
            exec_ctx.no_motion_cycles = 0;
        }
    } else {
        exec_ctx.no_motion_cycles = 0;
    }

    exec_ctx.q_prev_feedback = input.q_now;
    exec_ctx.q_prev_feedback_valid = input.q_now.allFinite();
    exec_ctx.previous_pos_err = input.pos_err_goal;
    exec_ctx.previous_ori_err = input.ori_err_goal;
    exec_ctx.previous_task_error_valid =
        std::isfinite(input.pos_err_goal) && std::isfinite(input.ori_err_goal);

    bool has_progress = false;
    if (input.pos_err_goal + cfg_.pos_progress_eps < exec_ctx.best_pos_err) {
        exec_ctx.best_pos_err = input.pos_err_goal;
        has_progress = true;
    }
    if (input.ori_err_goal + cfg_.ori_progress_eps < exec_ctx.best_ori_err) {
        exec_ctx.best_ori_err = input.ori_err_goal;
        has_progress = true;
    }
    const bool path_follow_progress =
        input.phase == ExecutionPhase::Track &&
        input.path_progress > exec_ctx.best_path_progress + cfg_.path_progress_eps;
    const bool path_progress_closing_goal =
        input.pos_err_goal <= exec_ctx.best_pos_err + cfg_.path_progress_pos_slack ||
        input.ori_err_goal <= exec_ctx.best_ori_err + cfg_.path_progress_ori_slack;
    if (path_follow_progress ||
        (path_progress_closing_goal &&
         input.path_progress > exec_ctx.best_path_progress + cfg_.path_progress_eps)) {
        exec_ctx.best_path_progress = input.path_progress;
        has_progress = true;
    } else {
        exec_ctx.best_path_progress = std::max(exec_ctx.best_path_progress, input.path_progress);
    }
    if (has_progress) {
        exec_ctx.no_progress_cycles = 0;
    } else {
        ++exec_ctx.no_progress_cycles;
    }

    output->primary_no_motion_recovery_active = shouldApplyPrimaryRecovery(
        exec_ctx,
        input.pos_err_goal,
        input.ori_err_goal,
        no_motion_cycle_limit,
        no_progress_cycle_limit);

    const bool obstacle_recovery_context =
        input.phase == ExecutionPhase::Hold ||
        input.obstacle_guidance_gate >= 0.70;
    if (exec_ctx.no_motion_cycles >= no_motion_cycle_limit && !obstacle_recovery_context) {
        output->no_motion_abort = true;
        output->no_motion_error = "no_joint_motion";
    }
    if (exec_ctx.no_progress_cycles >= no_progress_cycle_limit) {
        output->no_progress_abort = true;
        output->no_progress_error = "no_effective_progress";
    }
}

void ReactiveTaskWatchdog::applyPrimaryRecoveryBoost(
    const bool enabled,
    const Eigen::VectorXd& qd_max,
    Eigen::VectorXd& qdot_eigen) const {
    if (!enabled) {
        return;
    }
    for (int i = 0; i < qdot_eigen.size(); ++i) {
        const double abs_qdot = std::abs(qdot_eigen(i));
        if (abs_qdot < 1e-5 || abs_qdot >= cfg_.primary_recovery_joint_min) {
            continue;
        }
        const double boosted_abs_qdot =
            std::min(std::abs(qd_max(i)), cfg_.primary_recovery_joint_min);
        qdot_eigen(i) = std::copysign(boosted_abs_qdot, qdot_eigen(i));
    }
}

}  // namespace arm_controller::controller::reactive_task
