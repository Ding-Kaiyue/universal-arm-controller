#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <string>
#include <vector>

#include "algorithm/cartesian_path_planner/global_trajectory/global_trajectory_manager.hpp"
#include "algorithm/cartesian_path_planner/types.hpp"
#include "algorithm/neo/joint_limit_adapter.hpp"

namespace arm_controller::controller::reactive_task {

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace rq = arm_controller::algorithm::reactive_qp;

enum class ExecutionPhase {
    Track,
    Hold,
    Abort,
};

inline const char* toString(const ExecutionPhase phase) {
    switch (phase) {
        case ExecutionPhase::Track:
            return "track";
        case ExecutionPhase::Hold:
            return "hold";
        case ExecutionPhase::Abort:
            return "abort";
    }
    return "unknown";
}

struct ArmState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<std::string> joint_names;
    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qd_min;
    Eigen::VectorXd qd_max;
    rq::JointLimitData joint_limits;
};

struct BaseState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool available{false};
    Eigen::Vector3d pose_xy_yaw{Eigen::Vector3d::Zero()};
    Eigen::Vector3d twist_xy_yaw{Eigen::Vector3d::Zero()};
    std::string world_frame{"world"};
    std::string base_frame{"base_link"};
};

struct RobotState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp_sec{0.0};
    BaseState base;
    ArmState left_arm;
    ArmState right_arm;
};

struct ControlTarget {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    Eigen::Matrix<double, 6, 1> twist{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::VectorXd local_planner_joint_target;
    double local_planner_joint_target_dt_sec{0.0};
    bool local_planner_tracking{false};

    void setPoseTwist(
        const Eigen::Isometry3d& target_pose,
        const Eigen::Matrix<double, 6, 1>& target_twist) {
        pose = target_pose;
        twist = target_twist;
    }

    void clearLocalPlannerJointTarget() {
        local_planner_joint_target.resize(0);
        local_planner_joint_target_dt_sec = 0.0;
    }

    void setLocalPlannerJointTarget(
        const Eigen::VectorXd& joint_target,
        const double dt_sec) {
        local_planner_joint_target = joint_target;
        local_planner_joint_target_dt_sec = dt_sec;
    }

    const Eigen::VectorXd* localPlannerJointTarget(
        const Eigen::Index expected_size) const {
        return local_planner_joint_target.size() == expected_size &&
                       local_planner_joint_target.allFinite()
                   ? &local_planner_joint_target
                   : nullptr;
    }
};

struct WholeBodyStatusSnapshot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double min_margin{std::numeric_limits<double>::quiet_NaN()};
    bool collision_free{true};
    std::string state{"ok"};
    std::string worst_link;
    double worst_distance{std::numeric_limits<double>::quiet_NaN()};
    double worst_effective_radius{std::numeric_limits<double>::quiet_NaN()};
    double required_clearance{std::numeric_limits<double>::quiet_NaN()};
    double safe_distance_used{std::numeric_limits<double>::quiet_NaN()};
    double worst_gradient_norm{0.0};
    Eigen::Vector3d worst_gradient_world{Eigen::Vector3d::Zero()};
    Eigen::Vector3d worst_point_world{Eigen::Vector3d::Zero()};
    Eigen::Vector3d nearest_obstacle_point_world{Eigen::Vector3d::Zero()};
    bool nearest_obstacle_point_valid{false};
};

struct ExecutionStatusInput {
    ExecutionPhase current_phase{ExecutionPhase::Track};
    double pos_err_goal{std::numeric_limits<double>::infinity()};
    double ori_err_goal{std::numeric_limits<double>::infinity()};
    double path_progress{0.0};
    double task_residual_norm{0.0};
    double whole_body_min_margin{std::numeric_limits<double>::quiet_NaN()};
    bool whole_body_collision_free{true};
    std::string whole_body_state{"ok"};
    bool reference_finished{false};
    bool hold_ready{false};
    int no_progress_cycles{0};
    int no_motion_cycles{0};
    bool qdot_limit_violation{false};
    int phase_ticks{0};
    int hard_collision_margin_cycles{0};
};

struct ExecutionStatusOutput {
    ExecutionPhase phase{ExecutionPhase::Track};
    bool freeze_reference_progress{false};
    bool allow_lookahead{true};
    bool use_anchor_pose_only{false};
    std::string transition_reason;
    bool phase_changed{false};
};

struct LocalReferenceInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ExecutionPhase phase{ExecutionPhase::Track};
    bool freeze_reference_progress{false};
    bool allow_lookahead{true};
    bool use_anchor_pose_only{false};
    const cp::GlobalTrajectoryManager* global_trajectory{nullptr};
    Eigen::VectorXd q_now;
    Eigen::Vector3d ee_position{Eigen::Vector3d::Zero()};
    Eigen::Isometry3d ee_pose{Eigen::Isometry3d::Identity()};
    double planner_tick_sec{0.0};
    double planner_tick_accumulator{0.0};
    double tracked_reference_time_sec{0.0};
    int path_follow_joint_anchor_index_state{0};
    bool whole_body_collision_free{true};
};

struct LocalReferenceOutput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool ok{false};
    std::string error;
    double tracked_reference_time_sec{0.0};
    double continuous_sample_time_sec{0.0};
    double path_progress{0.0};
    double progress_scale{1.0};
    int time_hint_index{0};
    int anchor_index{0};
    int lookahead_index{0};
    int next_anchor_index_state{0};
    bool reference_finished{false};
    bool anchor_sample_valid{false};
    bool lookahead_sample_valid{false};
    bool used_lookahead{false};
    cp::TimedCartesianSample current_sample;
    cp::TimedCartesianSample anchor_sample;
    cp::TimedCartesianSample lookahead_sample;
};

}  // namespace arm_controller::controller::reactive_task
