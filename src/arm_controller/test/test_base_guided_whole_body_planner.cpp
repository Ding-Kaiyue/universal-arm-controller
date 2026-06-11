#include <gtest/gtest.h>

#include "algorithm/global_planner/base_guided_whole_body_planner.hpp"

namespace gp = arm_controller::algorithm::global_planner;
namespace cp = arm_controller::algorithm::cartesian_path_planner;

namespace {

constexpr int kDof = 15;

cp::PathPlanningInput makeInput() {
    cp::PathPlanningInput input;
    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd q_goal = Eigen::VectorXd::Zero(kDof);
    q_goal[0] = 1.20;
    q_goal[1] = 0.20;
    q_goal[2] = 0.10;
    q_goal.segment(3, 12).setConstant(0.25);

    input.q_start_seed = q_start;
    input.q_goal_candidates.push_back(q_goal);
    input.safe_distance = 0.05;
    input.feasibility_safe_distance = 0.03;
    input.joint_state_validator =
        [](const Eigen::VectorXd& q,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool ok = q.size() == kDof && q.allFinite();
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.10 : -0.10;
                diag->reason = ok ? "ok" : "invalid";
            }
            return ok;
        };
    input.joint_segment_validator =
        [](const Eigen::VectorXd& from,
           const Eigen::VectorXd& to,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool ok = from.size() == kDof && to.size() == kDof &&
                            from.allFinite() && to.allFinite();
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.10 : -0.10;
                diag->reason = ok ? "ok" : "invalid_segment";
            }
            return ok;
        };
    input.mobile_base_state_validator =
        [](double, double, double, double) {
            return true;
        };
    input.mobile_base_segment_validator =
        [](double, double, double, double, double, double, double) {
            return true;
        };
    return input;
}

}  // namespace

TEST(BaseGuidedWholeBodyPlannerTest, BuildsLayeredTrajectoryToVerifiedGoal) {
    cp::PlannerCommonConfig cfg;
    cfg.path_resolution = 0.04;
    cfg.default_segment_speed = 0.20;

    gp::BaseGuidedWholeBodyPlanner planner(cfg);
    const cp::PathPlanningInput input = makeInput();
    const cp::TimedJointTrajectory trajectory = planner.planTrajectory(input);

    ASSERT_FALSE(trajectory.empty());
    ASSERT_EQ(trajectory.joint_targets.front().size(), kDof);
    ASSERT_EQ(trajectory.joint_targets.back().size(), kDof);
    EXPECT_TRUE(trajectory.joint_targets.front().isApprox(*input.q_start_seed));
    EXPECT_TRUE(trajectory.joint_targets.back().isApprox(input.q_goal_candidates.front()));
    EXPECT_EQ(trajectory.segment_durations.size(), trajectory.joint_targets.size() - 1);
    EXPECT_EQ(trajectory.cumulative_times.size(), trajectory.joint_targets.size());
    EXPECT_GT(trajectory.total_duration, 0.0);
}

TEST(BaseGuidedWholeBodyPlannerTest, RejectsWhenSegmentValidatorFails) {
    cp::PlannerCommonConfig cfg;
    gp::BaseGuidedWholeBodyPlanner planner(cfg);
    cp::PathPlanningInput input = makeInput();
    input.joint_segment_validator =
        [](const Eigen::VectorXd&,
           const Eigen::VectorXd&,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            if (diag != nullptr) {
                diag->collision_free = false;
                diag->min_margin = -0.01;
                diag->reason = "blocked";
            }
            return false;
        };

    const cp::TimedJointTrajectory trajectory = planner.planTrajectory(input);
    EXPECT_TRUE(trajectory.empty());
}

TEST(BaseGuidedWholeBodyPlannerTest, LayeredGraphCanUseNonlinearArmCandidate) {
    cp::PlannerCommonConfig cfg;
    cfg.path_resolution = 0.08;
    cfg.default_segment_speed = 0.20;

    cp::PathPlanningInput input;
    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd q_goal = Eigen::VectorXd::Zero(kDof);
    q_goal[0] = 0.96;
    q_goal[1] = 0.0;
    q_goal[2] = 0.0;
    q_goal.segment(3, 12).setConstant(0.40);
    input.q_start_seed = q_start;
    input.q_goal_candidates.push_back(q_goal);
    input.safe_distance = 0.05;
    input.feasibility_safe_distance = 0.03;
    input.joint_state_validator =
        [](const Eigen::VectorXd& q,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool ok = q.size() == kDof && q.allFinite();
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.10 : -0.10;
                diag->reason = ok ? "ok" : "invalid";
            }
            return ok;
        };
    input.joint_segment_validator =
        [](const Eigen::VectorXd& from,
           const Eigen::VectorXd& to,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool finite = from.size() == kDof && to.size() == kDof &&
                                from.allFinite() && to.allFinite();
            const bool long_direct_edge = std::abs(to[0] - from[0]) > 0.35;
            const bool both_linear_mid =
                std::abs(from[3] - 0.20) < 0.03 &&
                std::abs(to[3] - 0.20) < 0.03;
            const bool ok = finite && !long_direct_edge && !both_linear_mid;
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.10 : -0.01;
                diag->reason = ok ? "ok" : "layered_test_blocked";
            }
            return ok;
        };
    input.mobile_base_state_validator =
        [](double, double, double, double) {
            return true;
        };
    input.mobile_base_segment_validator =
        [](double, double, double, double, double, double, double) {
            return true;
        };

    gp::BaseGuidedWholeBodyPlanner planner(cfg);
    const cp::TimedJointTrajectory trajectory = planner.planTrajectory(input);

    ASSERT_FALSE(trajectory.empty());
    bool used_nonlinear_midpoint = false;
    for (const Eigen::VectorXd& q : trajectory.joint_targets) {
        if (q[0] > 0.24 && q[0] < 0.72 && std::abs(q[3] - 0.20) > 0.03) {
            used_nonlinear_midpoint = true;
        }
    }
    EXPECT_TRUE(used_nonlinear_midpoint);
}

TEST(BaseGuidedWholeBodyPlannerTest, BuildsLongerWholeBodyRouteWithLayerSearch) {
    cp::PlannerCommonConfig cfg;
    cfg.path_resolution = 0.08;
    cfg.default_segment_speed = 0.20;

    cp::PathPlanningInput input;
    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd q_goal = Eigen::VectorXd::Zero(kDof);
    q_goal[0] = 0.96;
    q_goal[3] = 0.48;
    input.q_start_seed = q_start;
    input.q_goal_candidates.push_back(q_goal);
    input.safe_distance = 0.05;
    input.feasibility_safe_distance = 0.03;
    input.joint_state_validator =
        [](const Eigen::VectorXd& q,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool ok = q.size() == kDof && q.allFinite();
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.10 : -0.10;
                diag->reason = ok ? "ok" : "invalid";
            }
            return ok;
        };
    input.joint_segment_validator =
        [](const Eigen::VectorXd& from,
           const Eigen::VectorXd& to,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool finite = from.size() == kDof && to.size() == kDof &&
                                from.allFinite() && to.allFinite();
            const double dx = std::abs(to[0] - from[0]);
            const double dq = std::abs(to[3] - from[3]);
            const bool dp_edge = dx > 1e-6 && std::abs(dq / dx - 0.50) < 0.08;
            const bool rrt_small_edge = dx > 1e-6 && dx < 0.21 && dq < 0.13;
            const bool endpoint_edge =
                (std::abs(from[0]) < 1e-6 && std::abs(from[3]) < 1e-6) ||
                (std::abs(to[0] - 0.96) < 1e-6 && std::abs(to[3] - 0.48) < 1e-6);
            const bool ok = finite && (!dp_edge || rrt_small_edge || endpoint_edge);
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.10 : -0.01;
                diag->reason = ok ? "ok" : "dp_blocked_rrt_repair_allowed";
            }
            return ok;
        };
    input.mobile_base_state_validator =
        [](double, double, double, double) {
            return true;
        };
    input.mobile_base_segment_validator =
        [](double, double, double, double, double, double, double) {
            return true;
        };

    gp::BaseGuidedWholeBodyPlanner planner(cfg);
    const cp::TimedJointTrajectory trajectory = planner.planTrajectory(input);

    ASSERT_FALSE(trajectory.empty());
    EXPECT_TRUE(trajectory.joint_targets.front().isApprox(q_start));
    EXPECT_TRUE(trajectory.joint_targets.back().isApprox(q_goal));
    EXPECT_GT(trajectory.joint_targets.size(), 3u);
}

TEST(BaseGuidedWholeBodyPlannerTest, GoalGuidedLayersApproachGoalBeforeFinalLayer) {
    cp::PlannerCommonConfig cfg;
    cfg.path_resolution = 0.08;
    cfg.default_segment_speed = 0.20;

    cp::PathPlanningInput input;
    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd q_goal = Eigen::VectorXd::Zero(kDof);
    q_goal[0] = 1.60;
    q_goal[3] = 1.20;
    input.q_start_seed = q_start;
    input.q_goal_candidates.push_back(q_goal);
    input.safe_distance = 0.05;
    input.feasibility_safe_distance = 0.03;
    input.joint_state_validator =
        [](const Eigen::VectorXd& q,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool ok = q.size() == kDof && q.allFinite();
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.10 : -0.10;
                diag->reason = ok ? "ok" : "invalid";
            }
            return ok;
        };
    input.joint_segment_validator =
        [](const Eigen::VectorXd& from,
           const Eigen::VectorXd& to,
           double,
           cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            const bool finite = from.size() == kDof && to.size() == kDof &&
                                from.allFinite() && to.allFinite();
            const bool ok = finite && std::abs(to[3] - from[3]) <= 0.11;
            if (diag != nullptr) {
                diag->collision_free = ok;
                diag->min_margin = ok ? 0.10 : -0.01;
                diag->reason = ok ? "ok" : "joint_step_too_large";
            }
            return ok;
        };
    input.mobile_base_state_validator =
        [](double, double, double, double) {
            return true;
        };
    input.mobile_base_segment_validator =
        [](double, double, double, double, double, double, double) {
            return true;
        };

    gp::BaseGuidedWholeBodyPlanner planner(cfg);
    const cp::TimedJointTrajectory trajectory = planner.planTrajectory(input);

    ASSERT_FALSE(trajectory.empty());
    ASSERT_GE(trajectory.joint_targets.size(), 3u);
    EXPECT_TRUE(trajectory.joint_targets.back().isApprox(q_goal));
    const Eigen::VectorXd& before_goal =
        trajectory.joint_targets[trajectory.joint_targets.size() - 2u];
    EXPECT_LE(std::abs(q_goal[3] - before_goal[3]), 0.11);
}
