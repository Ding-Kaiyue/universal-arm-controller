#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "algorithm/cartesian_path_planner/types.hpp"
#include "algorithm/neo/joint_preference_loader.hpp"
#include "algorithm/neo/task_velocity_generator.hpp"
#include "controller/reactive_task/controller/reactive_task_execution_context.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace rq = arm_controller::algorithm::reactive_qp;

class ReactiveTaskTerminalPolicy {
public:
    struct PhaseFlags {
        bool path_follow_active{true};
        bool hold_active{false};
        bool terminal_goal_tracking{false};
        bool local_trajopt_tracking{false};
    };

    struct CommandTargetOutput {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PhaseFlags flags;
        Eigen::Isometry3d commanded_target_pose{Eigen::Isometry3d::Identity()};
        Eigen::Matrix<double, 6, 1> commanded_target_twist{Eigen::Matrix<double, 6, 1>::Zero()};
    };

    struct PostureReferenceInput {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PhaseFlags flags;
        ArmState arm_state;
        const cp::TimedCartesianSample* path_follow_joint_anchor_sample{nullptr};
        bool path_follow_joint_anchor_sample_valid{false};
        const cp::TimedCartesianSample* current_sample{nullptr};
        const Eigen::VectorXd* local_planner_joint_target{nullptr};
        double local_planner_joint_target_dt_sec{0.0};
        const rq::HumanLikeJointPreferenceConfig* joint_preference_cfg{nullptr};
    };

    ReactiveTaskTerminalPolicy() = default;

    PhaseFlags describePhase(ExecutionPhase phase) const;

    CommandTargetOutput buildCommandTarget(
        const ReactiveTaskExecutionContext& exec_ctx,
        const PhaseFlags& flags,
        const Eigen::Vector3d& goal_position,
        const Eigen::Matrix3d& goal_rotation,
        const cp::TimedCartesianSample& local_reference_sample,
        const Eigen::Isometry3d& local_reference_target_pose,
        const Eigen::Matrix<double, 6, 1>& local_reference_target_twist,
        const Eigen::Isometry3d& current_pose,
        bool reference_finished,
        const Eigen::Vector3d& clearance_direction,
        bool clearance_direction_valid,
        double whole_body_min_margin) const;

    void shapeTaskVelocity(
        const PhaseFlags& flags,
        rq::TaskVelocityOutput& task_out,
        const rq::TaskVelocityConfig& task_cfg) const;

    Eigen::VectorXd buildPostureReference(const PostureReferenceInput& input) const;

private:
    static bool sampleHasFiniteJointTarget(
        const cp::TimedCartesianSample& sample,
        Eigen::Index expected_size);
};

}  // namespace arm_controller::controller::reactive_task
