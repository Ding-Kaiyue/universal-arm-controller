#include <gtest/gtest.h>

#include <algorithm>

#include "algorithm/global_planner/ompl_rrt_connect_global_planner.hpp"
#include "controller/reactive_task/local_planner/reactive_task_local_planner.hpp"
#include "reactive_task_real_arm_test_helpers.hpp"

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace gp = arm_controller::algorithm::global_planner;
namespace rt = arm_controller::controller::reactive_task;

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

TEST(TrajOptToNeoReferenceIntegrationTest, RrtConnectAndTrajOptAvoidObservedObstacleForNeoReferences) {
    const reactive_task_test::RealArmModel arm = reactive_task_test::RealArmModel::load();
    const Eigen::VectorXd q_current = reactive_task_test::defaultStartQ();
    const Eigen::VectorXd q_global_goal = reactive_task_test::defaultGoalQ();

    ASSERT_TRUE(validateEeClearanceState(arm, q_current, nullptr));
    ASSERT_TRUE(validateEeClearanceState(arm, q_global_goal, nullptr));
    EXPECT_FALSE(validateEeClearanceSegment(arm, q_current, q_global_goal, nullptr));

    cp::PathPlanningInput global_input;
    global_input.q_start_seed = q_current;
    global_input.q_goal_candidates = {q_global_goal};
    global_input.q_min = arm.q_min;
    global_input.q_max = arm.q_max;
    global_input.safe_distance = kRequiredEeClearance;
    global_input.feasibility_safe_distance = kRequiredEeClearance;
    global_input.joint_state_validator =
        [&arm](const Eigen::VectorXd& q, double,
               cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            return validateEeClearanceState(arm, q, diag);
        };
    global_input.joint_segment_validator =
        [&arm](const Eigen::VectorXd& q_from, const Eigen::VectorXd& q_to,
               double, cp::PathPlanningInput::WholeBodyPoseDiagnostic* diag) {
            return validateEeClearanceSegment(arm, q_from, q_to, diag);
        };
    global_input.joint_to_pose_fn =
        [&arm](const Eigen::VectorXd& q, cp::CartesianWaypoint& wp) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            if (!arm.computePose(q, &pose)) {
                return false;
            }
            wp.position = pose.translation();
            wp.orientation = pose.linear();
            return true;
        };

    cp::PlannerCommonConfig global_cfg;
    global_cfg.joint_space_sampling_step_rad = 0.25;
    global_cfg.joint_space_sampling_time_budget_sec = 2.0;
    global_cfg.joint_space_shortcut_trials = 24;
    global_cfg.joint_space_preferred_min_margin_m = kRequiredEeClearance;
    global_cfg.enable_joint_trajectory_post_optimization = false;

    gp::OmplRrtConnectGlobalPlanner global_planner(global_cfg, cp::SmoothingConfig{});
    const cp::TimedJointTrajectory global_reference =
        global_planner.planTrajectory(global_input);
    ASSERT_FALSE(global_reference.empty());
    ASSERT_GE(global_reference.joint_targets.size(), 2u);

    const Eigen::VectorXd q_trajopt_goal = global_reference.joint_targets.back();
    Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
    ASSERT_TRUE(arm.computePose(q_current, &current_pose));

    rt::ArmLocalPlanner::Config local_cfg;
    local_cfg.horizon_steps = 8;
    local_cfg.dt_sec = 0.05;
    local_cfg.update_period_sec = 0.05;
    local_cfg.enable_collision_cost = true;
    local_cfg.enable_collision_constraint = true;
    rt::ArmLocalPlanner local_planner(local_cfg);

    rt::ArmLocalPlanner::Input local_input;
    local_input.current_pose = current_pose;
    local_input.joint_names = arm.joint_names;
    local_input.q_current = q_current;
    local_input.q_goal = q_trajopt_goal;
    local_input.q_goal_valid = true;
    local_input.robot_type = arm.robot_type;
    local_input.planning_group = arm.planning_group;
    local_input.base_link = arm.base_link;
    local_input.tip_link = arm.tip_link;
    local_input.urdf_path = arm.urdf_path;
    local_input.srdf_path = arm.srdf_path;
    local_input.joint_to_pose =
        [&arm](const Eigen::VectorXd& q, Eigen::Isometry3d* pose) {
            return arm.computePose(q, pose);
        };
    for (std::size_t i = 1; i < global_reference.joint_targets.size(); ++i) {
        cp::TimedCartesianSample sample;
        sample.ik_joint_target = global_reference.joint_targets[i];
        sample.has_ik_joint_target = true;
        sample.is_cartesian_tracking_target = false;
        if (i < global_reference.cumulative_times.size()) {
            sample.time_from_start = global_reference.cumulative_times[i];
        }
        local_input.reference_samples.push_back(std::move(sample));
    }
    local_input.sphere_obstacles.push_back(rt::ArmLocalPlanner::SphereObstacle{
        "camera_observed_obstacle",
        kObservedObstacleCenter,
        kObservedObstacleRadius});

    rt::ArmLocalPlanner::Output local_output;
    ASSERT_TRUE(local_planner.compute(local_input, &local_output))
        << local_output.error;
    ASSERT_TRUE(local_output.ok) << local_output.error;
    ASSERT_TRUE(local_output.used);
    ASSERT_TRUE(local_output.has_optimized_joint_target);
    ASSERT_GE(local_output.optimized_joint_trajectory.size(), 2u);
    ASSERT_EQ(local_output.target_poses.size(),
              local_output.optimized_joint_trajectory.size());
    ASSERT_EQ(local_output.target_twists.size(),
              local_output.optimized_joint_trajectory.size());

    EXPECT_GT(local_output.trajectory_dt_sec, 0.0);
    EXPECT_TRUE(local_output.target_pose.matrix().allFinite());
    EXPECT_TRUE(local_output.target_twist.allFinite());
    EXPECT_EQ(local_output.target_twist.size(), 6);
    EXPECT_TRUE(local_output.optimized_joint_target.allFinite());
    EXPECT_EQ(local_output.optimized_joint_target.size(), q_current.size());

    for (std::size_t i = 0; i < local_output.target_poses.size(); ++i) {
        EXPECT_TRUE(local_output.target_poses[i].matrix().allFinite());
        EXPECT_TRUE(local_output.target_twists[i].allFinite());
        EXPECT_TRUE(local_output.optimized_joint_trajectory[i].allFinite());
        EXPECT_GE(eeObstacleMargin(arm, local_output.optimized_joint_trajectory[i]), -1e-9);
    }
    for (std::size_t i = 1; i < local_output.optimized_joint_trajectory.size(); ++i) {
        EXPECT_TRUE(validateEeClearanceSegment(
            arm,
            local_output.optimized_joint_trajectory[i - 1],
            local_output.optimized_joint_trajectory[i],
            nullptr));
    }

    const Eigen::Vector3d current_p = current_pose.translation();
    const Eigen::Vector3d final_p = local_output.target_poses.back().translation();
    EXPECT_GT((final_p - current_p).norm(), 1e-4);
}
