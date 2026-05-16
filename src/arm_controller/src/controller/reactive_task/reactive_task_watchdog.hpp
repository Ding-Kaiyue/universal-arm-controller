#pragma once

#include <Eigen/Core>

#include <string>

#include "controller/reactive_task/reactive_task_execution_context.hpp"

namespace arm_controller::controller::reactive_task {

class ReactiveTaskWatchdog {
public:
    struct Config {
        double cmd_norm_min{0.10};
        double cmd_max_joint_min{0.05};
        double joint_delta_min_abs{1e-4};
        double joint_delta_response_ratio{0.25};
        double pos_err_delta_min{5e-5};
        double ori_err_delta_min{2e-4};
        double pos_progress_eps{1e-4};
        double ori_progress_eps{1e-3};
        double path_progress_eps{5e-3};
        double path_progress_pos_slack{0.015};
        double path_progress_ori_slack{0.05};
        double primary_recovery_joint_min{0.08};
        double goal_position_tolerance{0.01};
        double goal_orientation_tolerance_rad{0.08};
    };

    struct UpdateInput {
        Eigen::VectorXd q_now;
        Eigen::VectorXd qdot_eigen;
        Eigen::VectorXd qd_min;
        Eigen::VectorXd qd_max;
        ExecutionPhase phase{ExecutionPhase::Track};
        double pos_err_goal{0.0};
        double ori_err_goal{0.0};
        double path_progress{0.0};
        double obstacle_guidance_gate{0.0};
        double obstacle_min_distance{std::numeric_limits<double>::quiet_NaN()};
        double control_dt_sec{0.01};
    };

    struct UpdateOutput {
        double qdot_norm{0.0};
        double qdot_max_abs{0.0};
        double qdot_delta_norm{0.0};
        bool qdot_limit_violation{false};
        bool primary_no_motion_recovery_active{false};
        bool no_motion_abort{false};
        bool no_progress_abort{false};
        std::string no_motion_error;
        std::string no_progress_error;
    };

    ReactiveTaskWatchdog();
    explicit ReactiveTaskWatchdog(Config cfg);

    bool shouldApplyPrimaryRecovery(
        const ReactiveTaskExecutionContext& exec_ctx,
        double pos_err_goal,
        double ori_err_goal,
        int no_motion_cycle_limit,
        int no_progress_cycle_limit) const;

    void update(
        ReactiveTaskExecutionContext& exec_ctx,
        const UpdateInput& input,
        int no_motion_cycle_limit,
        int no_progress_cycle_limit,
        UpdateOutput* output) const;

    void applyPrimaryRecoveryBoost(
        bool enabled,
        const Eigen::VectorXd& qd_max,
        Eigen::VectorXd& qdot_eigen) const;

private:
    Config cfg_;
};

}  // namespace arm_controller::controller::reactive_task
