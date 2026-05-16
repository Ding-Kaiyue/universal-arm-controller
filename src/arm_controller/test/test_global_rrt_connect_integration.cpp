#include <gtest/gtest.h>

#include <algorithm>

#include "algorithm/global_planner/ompl_rrt_connect_global_planner.hpp"
#include "reactive_task_real_arm_test_helpers.hpp"

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace gp = arm_controller::algorithm::global_planner;

namespace {

constexpr double kObservedObstacleRadius = 0.025;
constexpr double kRequiredEeClearance = 0.010;
const Eigen::Vector3d kObservedObstacleCenter(-0.043, -0.172, 0.631);

double eeObstacleMargin(
    const reactive_task_test::RealArmModel& arm,
    const Eigen::VectorXd& q) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (!arm.computePose(q, &pose)) {
        return -std::numeric_limits<double>::infinity();
    }
    return (pose.translation() - kObservedObstacleCenter).norm() -
           (kObservedObstacleRadius + kRequiredEeClearance);
}

bool validateEeClearanceState(
    const reactive_task_test::RealArmModel& arm,
    const Eigen::VectorXd& q,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
    const bool in_bounds = arm.withinBounds(q);
    const double margin = in_bounds ? eeObstacleMargin(arm, q) : -0.10;
    const bool ok = in_bounds && margin >= 0.0;
    if (diag != nullptr) {
        diag->collision_free = ok;
        diag->min_margin = margin;
        diag->reason = ok ? "ok" : (in_bounds ? "observed_obstacle" : "joint_bounds");
        diag->worst_link_name = "left_Link6";
    }
    return ok;
}

bool validateEeClearanceSegment(
    const reactive_task_test::RealArmModel& arm,
    const Eigen::VectorXd& q_from,
    const Eigen::VectorXd& q_to,
    cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
    constexpr int kChecks = 24;
    double min_margin = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= kChecks; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(kChecks);
        const Eigen::VectorXd q = (1.0 - s) * q_from + s * q_to;
        const double margin = arm.withinBounds(q) ? eeObstacleMargin(arm, q) : -0.10;
        min_margin = std::min(min_margin, margin);
        if (margin < 0.0) {
            if (diag != nullptr) {
                diag->collision_free = false;
                diag->min_margin = min_margin;
                diag->reason = "observed_obstacle";
                diag->worst_link_name = "left_Link6";
            }
            return false;
        }
    }
    if (diag != nullptr) {
        diag->collision_free = true;
        diag->min_margin = min_margin;
        diag->reason = "ok";
        diag->worst_link_name = "left_Link6";
    }
    return true;
}

}  // namespace

TEST(GlobalRrtConnectIntegrationTest, AvoidsObservedObstacleDuringMotionOnRealDualArmModel) {
    const reactive_task_test::RealArmModel arm = reactive_task_test::RealArmModel::load();
    const Eigen::VectorXd q_start = reactive_task_test::defaultStartQ();
    const Eigen::VectorXd q_goal = reactive_task_test::defaultGoalQ();

    ASSERT_TRUE(validateEeClearanceState(arm, q_start, nullptr));
    ASSERT_TRUE(validateEeClearanceState(arm, q_goal, nullptr));
    EXPECT_FALSE(validateEeClearanceSegment(arm, q_start, q_goal, nullptr));

    cp::PathPlanningInput input;
    input.q_start_seed = q_start;
    input.q_goal_candidates = {q_goal};
    input.q_min = arm.q_min;
    input.q_max = arm.q_max;
    input.safe_distance = kRequiredEeClearance;
    input.feasibility_safe_distance = kRequiredEeClearance;
    input.joint_state_validator =
        [&arm](const Eigen::VectorXd& q, double,
               cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            return validateEeClearanceState(arm, q, diag);
        };
    input.joint_segment_validator =
        [&arm](const Eigen::VectorXd& q_from, const Eigen::VectorXd& q_to,
               double, cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            return validateEeClearanceSegment(arm, q_from, q_to, diag);
        };
    input.joint_to_pose_fn =
        [&arm](const Eigen::VectorXd& q, cp::CartesianWaypoint& wp) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            if (!arm.computePose(q, &pose)) {
                return false;
            }
            wp.position = pose.translation();
            wp.orientation = pose.linear();
            return true;
        };

    cp::PlannerCommonConfig cfg;
    cfg.joint_space_sampling_step_rad = 0.25;
    cfg.joint_space_sampling_time_budget_sec = 2.0;
    cfg.joint_space_shortcut_trials = 24;
    cfg.joint_space_preferred_min_margin_m = kRequiredEeClearance;
    cfg.enable_joint_trajectory_post_optimization = false;

    gp::OmplRrtConnectGlobalPlanner planner(cfg, cp::SmoothingConfig{});
    const cp::TimedJointTrajectory traj = planner.planTrajectory(input);

    ASSERT_FALSE(traj.empty());
    ASSERT_GE(traj.joint_targets.size(), 2u);
    ASSERT_EQ(traj.cumulative_times.size(), traj.joint_targets.size());
    ASSERT_EQ(traj.segment_durations.size() + 1u, traj.joint_targets.size());
    EXPECT_EQ(traj.segment_kind, cp::PlannedSegmentKind::Goal);
    EXPECT_GT(traj.total_duration, 0.0);
    EXPECT_TRUE(traj.joint_targets.front().isApprox(q_start, 1e-9));
    EXPECT_TRUE(traj.joint_targets.back().isApprox(q_goal, 1e-6));

    for (const Eigen::VectorXd& q : traj.joint_targets) {
        EXPECT_EQ(q.size(), q_start.size());
        EXPECT_TRUE(q.allFinite());
        EXPECT_TRUE(arm.withinBounds(q));
        EXPECT_GE(eeObstacleMargin(arm, q), -1e-9);
    }
    for (std::size_t i = 1; i < traj.joint_targets.size(); ++i) {
        EXPECT_TRUE(validateEeClearanceSegment(
            arm, traj.joint_targets[i - 1], traj.joint_targets[i], nullptr));
    }

    Eigen::Isometry3d final_pose = Eigen::Isometry3d::Identity();
    ASSERT_TRUE(arm.computePose(traj.joint_targets.back(), &final_pose));
    EXPECT_TRUE(final_pose.matrix().allFinite());
}
