#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace arm_controller::algorithm::cartesian_path_planner {

struct CartesianWaypoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d orientation{Eigen::Matrix3d::Identity()};
};

using CartesianWaypointList =
    std::vector<CartesianWaypoint, Eigen::aligned_allocator<CartesianWaypoint>>;

struct CartesianPath {
    CartesianWaypointList waypoints;
    double length{0.0};
};

struct TimedCartesianSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Isometry3d T_target{Eigen::Isometry3d::Identity()};
    Eigen::Matrix<double, 6, 1> target_twist{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::VectorXd ik_joint_target;
    bool has_ik_joint_target{false};
    bool is_cartesian_tracking_target{false};
    double time_from_start{0.0};
};

using TimedCartesianSampleList =
    std::vector<TimedCartesianSample, Eigen::aligned_allocator<TimedCartesianSample>>;

enum class PlannedSegmentKind {
    Goal,
    Recovery,
};

struct TimedCartesianTrajectory {
    CartesianWaypointList waypoints;
    // Optional per-waypoint IK joint targets aligned with `waypoints`.
    // Empty vectors indicate unavailable IK at that waypoint.
    std::vector<Eigen::VectorXd> waypoint_joint_targets;
    std::vector<double> segment_durations;  // size = waypoints.size() - 1
    std::vector<double> cumulative_times;  // size = waypoints.size()
    double total_duration{0.0};
    PlannedSegmentKind segment_kind{PlannedSegmentKind::Goal};

    bool empty() const { return waypoints.size() < 2; }
};

struct TimedJointSample {
    Eigen::VectorXd joint_target;
    bool has_joint_target{false};
    double time_from_start{0.0};
};

struct TimedJointTrajectory {
    std::vector<Eigen::VectorXd> joint_targets;
    std::vector<double> segment_durations;  // size = joint_targets.size() - 1
    std::vector<double> cumulative_times;  // size = joint_targets.size()
    double total_duration{0.0};
    PlannedSegmentKind segment_kind{PlannedSegmentKind::Goal};

    bool empty() const { return joint_targets.size() < 2; }
};

struct PathPlanningInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct WholeBodyPoseDiagnostic;

    using WholeBodyPoseValidatorFn = std::function<bool(
        const Eigen::Vector3d& /*p*/,
        const Eigen::Matrix3d& /*R*/,
        double /*safe_distance*/,
        const std::optional<Eigen::VectorXd>& /*q_seed*/,
        Eigen::VectorXd& /*q_solution*/)>;

    using WholeBodySegmentValidatorFn = std::function<bool(
        const CartesianWaypoint& /*from*/,
        const CartesianWaypoint& /*to*/,
        double /*safe_distance*/,
        const std::optional<Eigen::VectorXd>& /*q_seed*/,
        Eigen::VectorXd& /*q_end*/,
        WholeBodyPoseDiagnostic* /*failed_diag*/)>;

    struct WholeBodyPoseDiagnostic {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        bool ik_ok{false};
        bool external_ik_ok{false};
        bool fallback_ik_used{false};
        bool collision_free{false};
        double min_margin{0.0};
        std::string reason;
        std::string worst_link_name;
        Eigen::Vector3d worst_point_world{Eigen::Vector3d::Zero()};
        double worst_distance{0.0};
        double worst_effective_radius{0.0};
        double required_clearance{0.0};
        double safe_distance_used{0.0};
        double worst_gradient_norm{0.0};
        Eigen::Vector3d worst_gradient_world{Eigen::Vector3d::Zero()};
        bool has_failed_pose{false};
        bool failed_on_segment_sample{false};
        double failed_segment_t{0.0};
        Eigen::Vector3d failed_pose_world{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d failed_pose_orientation{Eigen::Matrix3d::Identity()};
        Eigen::VectorXd q_solution;
    };

    using WholeBodyPoseDiagnosticFn = std::function<WholeBodyPoseDiagnostic(
        const Eigen::Vector3d& /*p*/,
        const Eigen::Matrix3d& /*R*/,
        double /*safe_distance*/,
        const std::optional<Eigen::VectorXd>& /*q_seed*/)>;

    using JointStateValidatorFn = std::function<bool(
        const Eigen::VectorXd& /*q*/,
        double /*safe_distance*/,
        WholeBodyPoseDiagnostic* /*diag*/)>;

    using JointSegmentValidatorFn = std::function<bool(
        const Eigen::VectorXd& /*q_from*/,
        const Eigen::VectorXd& /*q_to*/,
        double /*safe_distance*/,
        WholeBodyPoseDiagnostic* /*diag*/)>;

    using JointToPoseFn = std::function<bool(
        const Eigen::VectorXd& /*q*/,
        CartesianWaypoint& /*wp*/)>;

    Eigen::Vector3d p_start{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p_goal{Eigen::Vector3d::Zero()};
    Eigen::Vector3d bypass_axis_hint{Eigen::Vector3d::Zero()};

    Eigen::Matrix3d R_start{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_goal{Eigen::Matrix3d::Identity()};
    std::optional<Eigen::VectorXd> q_start_seed;
    std::vector<Eigen::VectorXd> q_goal_candidates;
    Eigen::VectorXd q_min;
    Eigen::VectorXd q_max;

    double safe_distance{0.05};
    double feasibility_safe_distance{0.0};
    double hard_clearance{0.0};
    double goal_tolerance{0.02};

    // Optional full-body validation:
    // - pose validator: IK + full-body collision check for one pose
    // - segment validator: continuous check between two poses
    //
    // If segment validator is not set but pose validator is set, A* will perform
    // sampled segment checks by repeatedly calling pose validator.
    WholeBodyPoseValidatorFn whole_body_pose_validator;
    WholeBodySegmentValidatorFn whole_body_segment_validator;
    WholeBodyPoseDiagnosticFn whole_body_pose_diagnostic;
    JointStateValidatorFn joint_state_validator;
    JointSegmentValidatorFn joint_segment_validator;
    JointToPoseFn joint_to_pose_fn;
};

struct PathPlanningOutput {
    bool success{false};
    PlannedSegmentKind segment_kind{PlannedSegmentKind::Goal};
    std::vector<Eigen::VectorXd> joint_waypoints;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
