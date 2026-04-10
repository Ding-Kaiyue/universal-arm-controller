#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

#include "algorithm/cartesian_path_planner/config/astar_config.hpp"
#include "algorithm/cartesian_path_planner/config/planner_common_config.hpp"
#include "algorithm/cartesian_path_planner/config/smoothing_config.hpp"
#include "algorithm/cartesian_path_planner/core/cartesian_path_planner.hpp"
#include "algorithm/cartesian_path_planner/map/dummy_distance_field.hpp"
#include "algorithm/cartesian_path_planner/replanning/replanner_manager.hpp"

namespace cp = arm_controller::algorithm::cartesian_path_planner;

namespace {

std::shared_ptr<cp::CartesianPathPlanner> makePlanner() {
    const Eigen::Vector3d map_min(0.0, 0.0, 0.0);
    const Eigen::Vector3d map_max(1.0, 1.0, 1.0);
    auto map = std::make_shared<cp::DummyDistanceField>(map_min, map_max);

    cp::PlannerCommonConfig common_cfg;
    common_cfg.default_segment_speed = 0.2;

    cp::AStarConfig astar_cfg;
    astar_cfg.use_se3_search = true;
    astar_cfg.voxel_resolution = 0.05;
    astar_cfg.neighbor_mode = 26;
    astar_cfg.orientation_bin_size_rad = 0.5235987756;  // 30 deg
    astar_cfg.orientation_goal_tolerance_rad = 1.2;
    astar_cfg.max_iterations = 300000;
    astar_cfg.max_planning_time_sec = 1.0;
    astar_cfg.edge_check_step = 0.01;

    cp::SmoothingConfig smoothing_cfg;
    smoothing_cfg.max_shortcut_trials = 0;

    return std::make_shared<cp::CartesianPathPlanner>(
        common_cfg, astar_cfg, smoothing_cfg, map, map_min);
}

cp::PathPlanningInput makeInput(
    const Eigen::Vector3d& p_start,
    const Eigen::Vector3d& p_goal) {
    cp::PathPlanningInput in;
    in.p_start = p_start;
    in.p_goal = p_goal;
    in.R_start = Eigen::Matrix3d::Identity();
    in.R_goal = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    in.safe_distance = 0.03;
    in.goal_tolerance = 0.08;
    return in;
}

}  // namespace

TEST(CartesianReplannerManagerTest, ReplanTickCadenceWorks) {
    auto planner = makePlanner();
    cp::ReplannerManager replanner(planner);

    cp::ReplannerConfig cfg;
    cfg.segment_sample_step_m = 0.03;
    cfg.replan_every_control_ticks = 10;
    cfg.control_cycle_sec = 0.004;
    cfg.prediction_horizon_ticks = 10;
    cfg.planning_latency_sec = 0.06;
    replanner.setConfig(cfg);

    for (int tick = 1; tick <= 30; ++tick) {
        const bool should = replanner.shouldReplanAtControlTick(tick);
        if (tick % 10 == 0) {
            EXPECT_TRUE(should) << "tick=" << tick;
        } else {
            EXPECT_FALSE(should) << "tick=" << tick;
        }
    }
}

TEST(CartesianReplannerManagerTest, PlanFromFeedbackPoseUsesFeedbackAsNewStart) {
    auto planner = makePlanner();
    cp::ReplannerManager replanner(planner);

    cp::ReplannerConfig cfg;
    cfg.segment_sample_step_m = 0.03;
    cfg.replan_every_control_ticks = 10;
    cfg.control_cycle_sec = 0.004;
    cfg.prediction_horizon_ticks = 10;
    cfg.planning_latency_sec = 0.06;
    replanner.setConfig(cfg);

    cp::PathPlanningInput in = makeInput(
        Eigen::Vector3d(0.1, 0.1, 0.1),
        Eigen::Vector3d(0.9, 0.8, 0.7));

    std::string err;
    ASSERT_TRUE(replanner.start(in, &err)) << err;
    ASSERT_TRUE(replanner.hasActiveTrajectory());
    EXPECT_GE(replanner.activeSegmentPointCount(), 2);

    Eigen::Isometry3d T_feedback = Eigen::Isometry3d::Identity();
    T_feedback.translation() = Eigen::Vector3d(0.35, 0.22, 0.18);

    cp::PathPlanningInput next = in;
    next.p_goal = Eigen::Vector3d(0.8, 0.2, 0.8);
    ASSERT_TRUE(replanner.planFromFeedbackPose(next, T_feedback, &err)) << err;

    cp::TimedCartesianSample s0;
    ASSERT_TRUE(replanner.sample(0, s0));
    EXPECT_NEAR(s0.T_target.translation().x(), T_feedback.translation().x(), 1e-9);
    EXPECT_NEAR(s0.T_target.translation().y(), T_feedback.translation().y(), 1e-9);
    EXPECT_NEAR(s0.T_target.translation().z(), T_feedback.translation().z(), 1e-9);
}

TEST(CartesianReplannerManagerTest, PlanFromJointFeedbackProviderWorksWithFakeData) {
    auto planner = makePlanner();
    cp::ReplannerManager replanner(planner);

    cp::ReplannerConfig cfg;
    cfg.segment_sample_step_m = 0.03;
    cfg.replan_every_control_ticks = 10;
    cfg.control_cycle_sec = 0.004;
    cfg.prediction_horizon_ticks = 10;
    cfg.planning_latency_sec = 0.06;
    replanner.setConfig(cfg);

    cp::PathPlanningInput in = makeInput(
        Eigen::Vector3d(0.1, 0.1, 0.1),
        Eigen::Vector3d(0.9, 0.8, 0.7));

    std::string err;
    ASSERT_TRUE(replanner.start(in, &err)) << err;

    const auto q_provider = []() {
        return std::vector<double>{0.42, 0.21, 0.33, 0.0, 0.0, 0.0};
    };

    const auto fk_callback =
        [](const std::vector<double>& q, Eigen::Isometry3d& T_out) -> bool {
            if (q.size() < 3) {
                return false;
            }
            T_out = Eigen::Isometry3d::Identity();
            T_out.translation() = Eigen::Vector3d(q[0], q[1], q[2]);
            return true;
        };

    cp::PathPlanningInput next = in;
    next.p_goal = Eigen::Vector3d(0.8, 0.2, 0.8);
    ASSERT_TRUE(replanner.planFromJointFeedbackProvider(next, q_provider, fk_callback, &err)) << err;

    cp::TimedCartesianSample s0;
    ASSERT_TRUE(replanner.sample(0, s0));
    EXPECT_NEAR(s0.T_target.translation().x(), 0.42, 1e-9);
    EXPECT_NEAR(s0.T_target.translation().y(), 0.21, 1e-9);
    EXPECT_NEAR(s0.T_target.translation().z(), 0.33, 1e-9);
}

TEST(CartesianReplannerManagerTest, DistanceSamplingGeneratesMorePointsForLongerPath) {
    auto planner = makePlanner();
    cp::ReplannerManager replanner(planner);

    cp::ReplannerConfig cfg;
    cfg.segment_sample_step_m = 0.03;
    cfg.replan_every_control_ticks = 10;
    cfg.control_cycle_sec = 0.004;
    cfg.prediction_horizon_ticks = 10;
    cfg.planning_latency_sec = 0.06;
    replanner.setConfig(cfg);

    cp::PathPlanningInput short_in = makeInput(
        Eigen::Vector3d(0.10, 0.10, 0.10),
        Eigen::Vector3d(0.30, 0.10, 0.10));

    cp::PathPlanningInput long_in = short_in;
    long_in.p_goal = Eigen::Vector3d(0.90, 0.80, 0.70);

    std::string err;
    ASSERT_TRUE(replanner.start(short_in, &err)) << err;
    const int short_points = replanner.activeSegmentPointCount();
    EXPECT_GE(short_points, 2);

    ASSERT_TRUE(replanner.planSegment(long_in, &err)) << err;
    const int long_points = replanner.activeSegmentPointCount();
    EXPECT_GT(long_points, short_points);
}

TEST(CartesianReplannerManagerTest, PlanFromPredictedActiveTrajectoryUsesFuturePoint) {
    auto planner = makePlanner();
    cp::ReplannerManager replanner(planner);

    cp::ReplannerConfig cfg;
    cfg.segment_sample_step_m = 0.02;
    cfg.replan_every_control_ticks = 10;
    cfg.control_cycle_sec = 0.004;
    cfg.prediction_horizon_ticks = 10;
    cfg.planning_latency_sec = 10.0;
    replanner.setConfig(cfg);

    cp::PathPlanningInput in = makeInput(
        Eigen::Vector3d(0.1, 0.1, 0.1),
        Eigen::Vector3d(0.9, 0.9, 0.9));

    std::string err;
    ASSERT_TRUE(replanner.start(in, &err)) << err;
    ASSERT_GE(replanner.activeSegmentPointCount(), 3);

    // Very large latency should clamp prediction to the last point.
    const int end_idx = replanner.activeSegmentPointCount() - 1;
    cp::TimedCartesianSample s_now;
    cp::TimedCartesianSample s_end;
    ASSERT_TRUE(replanner.sample(0, s_now));
    ASSERT_TRUE(replanner.sample(end_idx, s_end));

    cp::PathPlanningInput next = in;
    next.p_goal = Eigen::Vector3d(0.8, 0.2, 0.8);
    ASSERT_TRUE(replanner.planFromPredictedActiveTrajectory(next, 0, &err)) << err;

    cp::TimedCartesianSample s0_new;
    ASSERT_TRUE(replanner.sample(0, s0_new));

    // New start should match predicted end point (clamped).
    const double d_to_now = (s0_new.T_target.translation() - s_now.T_target.translation()).norm();
    const double d_to_end = (s0_new.T_target.translation() - s_end.T_target.translation()).norm();
    EXPECT_GT(d_to_now, 1e-4);
    EXPECT_LT(d_to_end, 1e-9);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
