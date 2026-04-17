#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace arm_controller::algorithm::cartesian_path_planner {

struct CartesianWaypoint {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d orientation{Eigen::Matrix3d::Identity()};
};

struct CartesianPath {
    std::vector<CartesianWaypoint> waypoints;
    double length{0.0};
};

struct TimedCartesianSample {
    Eigen::Isometry3d T_target{Eigen::Isometry3d::Identity()};
    Eigen::Matrix<double, 6, 1> target_twist{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::VectorXd ik_joint_target;
    bool has_ik_joint_target{false};
    double time_from_start{0.0};
};

struct TimedCartesianTrajectory {
    std::vector<CartesianWaypoint> waypoints;
    // Optional per-waypoint IK joint targets aligned with `waypoints`.
    // Empty vectors indicate unavailable IK at that waypoint.
    std::vector<Eigen::VectorXd> waypoint_joint_targets;
    std::vector<double> segment_durations;  // size = waypoints.size() - 1
    std::vector<double> cumulative_times;  // size = waypoints.size()
    double total_duration{0.0};

    bool empty() const { return waypoints.size() < 2; }
};

struct PathPlanningInput {
    struct ForbiddenSphere {
        Eigen::Vector3d center{Eigen::Vector3d::Zero()};
        double radius{0.0};
    };

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
        Eigen::VectorXd& /*q_end*/)>;

    struct WholeBodyPoseDiagnostic {
        bool ik_ok{false};
        bool external_ik_ok{false};
        bool fallback_ik_used{false};
        bool collision_free{false};
        double min_margin{0.0};
        std::string reason;
        std::string worst_link_name;
        Eigen::VectorXd q_solution;
    };

    struct WholeBodyPostcheckFailureEvent {
        int attempt_index{1};
        int max_attempts{1};
        int waypoint_index{-1};
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d orientation{Eigen::Matrix3d::Identity()};
        WholeBodyPoseDiagnostic diagnostic;
    };

    using WholeBodyPoseDiagnosticFn = std::function<WholeBodyPoseDiagnostic(
        const Eigen::Vector3d& /*p*/,
        const Eigen::Matrix3d& /*R*/,
        double /*safe_distance*/,
        const std::optional<Eigen::VectorXd>& /*q_seed*/)>;

    using WholeBodyPostcheckFailureCallback = std::function<void(
        const WholeBodyPostcheckFailureEvent&)>;

    Eigen::Vector3d p_start{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p_goal{Eigen::Vector3d::Zero()};

    Eigen::Matrix3d R_start{Eigen::Matrix3d::Identity()};
    Eigen::Matrix3d R_goal{Eigen::Matrix3d::Identity()};
    std::optional<Eigen::VectorXd> q_start_seed;

    double safe_distance{0.05};
    double goal_tolerance{0.02};
    bool whole_body_postcheck_non_blocking{false};
    int whole_body_postcheck_max_attempts{1};
    double whole_body_retry_forbidden_radius{0.04};
    double whole_body_retry_penalty_margin{0.05};
    double whole_body_retry_penalty_weight{2.5};
    std::vector<ForbiddenSphere> forbidden_spheres;

    // Optional full-body validation:
    // - pose validator: IK + full-body collision check for one pose
    // - segment validator: continuous check between two poses
    //
    // If segment validator is not set but pose validator is set, A* will perform
    // sampled segment checks by repeatedly calling pose validator.
    WholeBodyPoseValidatorFn whole_body_pose_validator;
    WholeBodySegmentValidatorFn whole_body_segment_validator;
    WholeBodyPoseDiagnosticFn whole_body_pose_diagnostic;
    WholeBodyPostcheckFailureCallback whole_body_postcheck_failure_callback;
};

struct PathPlanningOutput {
    bool success{false};
    CartesianPath path;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
