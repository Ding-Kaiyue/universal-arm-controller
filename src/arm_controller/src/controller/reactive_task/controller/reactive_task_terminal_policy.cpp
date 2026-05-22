#include "controller/reactive_task/controller/reactive_task_terminal_policy.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::controller::reactive_task {

ReactiveTaskTerminalPolicy::PhaseFlags ReactiveTaskTerminalPolicy::describePhase(
    const ExecutionPhase phase) const {
    PhaseFlags flags;
    flags.hold_active = phase == ExecutionPhase::Hold;
    flags.path_follow_active = phase == ExecutionPhase::Track;
    return flags;
}

ReactiveTaskTerminalPolicy::CommandTargetOutput ReactiveTaskTerminalPolicy::buildCommandTarget(
    const ReactiveTaskExecutionContext& exec_ctx,
    const PhaseFlags& flags,
    const Eigen::Vector3d& goal_position,
    const Eigen::Matrix3d& goal_rotation,
    const cp::TimedCartesianSample& local_reference_sample,
    const Eigen::Isometry3d& local_reference_target_pose,
    const Eigen::Matrix<double, 6, 1>& local_reference_target_twist,
    const Eigen::Isometry3d& current_pose,
    const bool reference_finished,
    const Eigen::Vector3d& clearance_direction,
    const bool clearance_direction_valid,
    const double whole_body_min_margin) const {
    CommandTargetOutput output;
    output.flags = flags;

    output.commanded_target_pose = local_reference_target_pose;
    output.commanded_target_twist = local_reference_target_twist;
    if (flags.path_follow_active && reference_finished &&
        !flags.local_trajopt_tracking) {
        output.commanded_target_pose = Eigen::Isometry3d::Identity();
        output.commanded_target_pose.linear() = goal_rotation;
        output.commanded_target_pose.translation() = goal_position;
        output.commanded_target_twist.setZero();
        output.flags.terminal_goal_tracking = true;
    }
    if (flags.hold_active) {
        output.commanded_target_pose = current_pose;
        if (exec_ctx.hold_pose_valid) {
            output.commanded_target_pose = exec_ctx.hold_pose;
        }
        output.commanded_target_twist.setZero();
    }

    (void)clearance_direction;
    (void)clearance_direction_valid;
    (void)whole_body_min_margin;
    (void)local_reference_sample;
    return output;
}

void ReactiveTaskTerminalPolicy::shapeTaskVelocity(
    const PhaseFlags& flags,
    rq::TaskVelocityOutput& task_out,
    const rq::TaskVelocityConfig& task_cfg) const {
    (void)flags;
    task_out.v_des = task_out.v_ff + task_out.v_fb;
    if (task_out.v_des.head<3>().norm() > 1e-12) {
        task_out.v_des.head<3>() = task_out.v_des.head<3>().normalized() *
                                   std::min(task_out.v_des.head<3>().norm(),
                                            task_cfg.max_linear_speed);
    } else {
        task_out.v_des.head<3>().setZero();
    }
    if (task_out.v_des.tail<3>().norm() > 1e-12) {
        task_out.v_des.tail<3>() = task_out.v_des.tail<3>().normalized() *
                                   std::min(task_out.v_des.tail<3>().norm(),
                                            task_cfg.max_angular_speed);
    } else {
        task_out.v_des.tail<3>().setZero();
    }
}

Eigen::VectorXd ReactiveTaskTerminalPolicy::buildPostureReference(
    const PostureReferenceInput& input) const {
    const Eigen::VectorXd& q_now = input.arm_state.q;
    Eigen::VectorXd posture_qdot_ref = Eigen::VectorXd::Zero(q_now.size());
    if (input.joint_preference_cfg == nullptr) {
        return posture_qdot_ref;
    }

    if (input.flags.path_follow_active) {
        if (input.local_planner_joint_target != nullptr &&
            input.local_planner_joint_target->size() == q_now.size() &&
            input.local_planner_joint_target->allFinite()) {
            const double dt =
                input.flags.terminal_goal_tracking
                    ? 1.0
                    : std::max(0.08, 4.0 * input.local_planner_joint_target_dt_sec);
            const double path_follow_posture_gain =
                input.flags.terminal_goal_tracking ? 0.55 : 0.35;
            const double path_follow_posture_joint_speed_cap =
                input.flags.terminal_goal_tracking ? 0.35 : 0.35;
            posture_qdot_ref =
                path_follow_posture_gain * (*input.local_planner_joint_target - q_now) / dt;
            for (int i = 0; i < posture_qdot_ref.size(); ++i) {
                posture_qdot_ref(i) = std::clamp(
                    posture_qdot_ref(i),
                    -path_follow_posture_joint_speed_cap,
                    path_follow_posture_joint_speed_cap);
            }
            return posture_qdot_ref;
        }
        if (input.flags.local_trajopt_tracking) {
            return posture_qdot_ref;
        }

        const cp::TimedCartesianSample* branch_hold_sample = nullptr;
        if (input.flags.terminal_goal_tracking && input.current_sample != nullptr &&
            sampleHasFiniteJointTarget(*input.current_sample, q_now.size())) {
            branch_hold_sample = input.current_sample;
        } else if (input.path_follow_joint_anchor_sample_valid && input.path_follow_joint_anchor_sample != nullptr &&
            sampleHasFiniteJointTarget(*input.path_follow_joint_anchor_sample, q_now.size())) {
            branch_hold_sample = input.path_follow_joint_anchor_sample;
        } else if (input.current_sample != nullptr &&
                   sampleHasFiniteJointTarget(*input.current_sample, q_now.size())) {
            branch_hold_sample = input.current_sample;
        }
        if (branch_hold_sample != nullptr) {
            const double path_follow_posture_gain =
                input.flags.terminal_goal_tracking ? 0.55 : 0.18;
            const double path_follow_posture_joint_speed_cap =
                input.flags.terminal_goal_tracking ? 0.22 : 0.10;
            posture_qdot_ref =
                path_follow_posture_gain * input.joint_preference_cfg->posture_k *
                (branch_hold_sample->ik_joint_target - q_now);
            for (int i = 0; i < posture_qdot_ref.size(); ++i) {
                posture_qdot_ref(i) = std::clamp(
                    posture_qdot_ref(i),
                    -path_follow_posture_joint_speed_cap,
                    path_follow_posture_joint_speed_cap);
            }
        }
    }

    return posture_qdot_ref;
}

bool ReactiveTaskTerminalPolicy::sampleHasFiniteJointTarget(
    const cp::TimedCartesianSample& sample,
    const Eigen::Index expected_size) {
    return sample.has_ik_joint_target && sample.ik_joint_target.size() == expected_size &&
           sample.ik_joint_target.allFinite();
}

}  // namespace arm_controller::controller::reactive_task
