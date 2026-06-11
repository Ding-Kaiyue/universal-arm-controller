#include <gtest/gtest.h>

#include "algorithm/cartesian_path_planner/base/kino_astar_base_planner.hpp"

#include <cmath>

namespace cp = arm_controller::algorithm::cartesian_path_planner;

namespace {

bool circleFree(
    const cp::KinoAstarBasePlanner::BaseState& state,
    const double clearance) {
    const double obstacle_x = 1.0;
    const double obstacle_y = 0.0;
    const double obstacle_radius = 0.32 + clearance;
    return std::hypot(state.x - obstacle_x, state.y - obstacle_y) >
           obstacle_radius;
}

bool segmentFree(
    const cp::KinoAstarBasePlanner::BaseState& from,
    const cp::KinoAstarBasePlanner::BaseState& to,
    const double clearance) {
    constexpr int kSamples = 12;
    for (int i = 0; i <= kSamples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(kSamples);
        cp::KinoAstarBasePlanner::BaseState sample;
        sample.x = from.x + t * (to.x - from.x);
        sample.y = from.y + t * (to.y - from.y);
        sample.yaw = from.yaw + t * (to.yaw - from.yaw);
        if (!circleFree(sample, clearance)) {
            return false;
        }
    }
    return true;
}

cp::KinoAstarBasePlanner makePlanner() {
    cp::KinoAstarBasePlanner::Config cfg;
    cfg.xy_resolution = 0.08;
    cfg.yaw_resolution = 0.30;
    cfg.primitive_arc_length = 0.16;
    cfg.max_steer_angle = 0.75;
    cfg.goal_xy_tolerance = 0.12;
    cfg.goal_yaw_tolerance = 0.35;
    cfg.map_min_x = -0.5;
    cfg.map_max_x = 2.5;
    cfg.map_min_y = -1.5;
    cfg.map_max_y = 1.5;
    cfg.clearance = 0.05;
    cfg.max_expansions = 20000;
    return cp::KinoAstarBasePlanner(cfg);
}

}  // namespace

TEST(KinoAstarBasePlannerTest, PlansAroundCircularObstacle) {
    const cp::KinoAstarBasePlanner planner = makePlanner();
    cp::KinoAstarBasePlanner::Input input;
    input.start = {0.0, 0.0, 0.0};
    input.goal = {2.0, 0.0, 0.0};
    input.state_validator = circleFree;
    input.segment_validator = segmentFree;

    const cp::KinoAstarBasePlanner::Result result = planner.plan(input);

    ASSERT_TRUE(result.success);
    ASSERT_GE(result.path.size(), 3u);
    EXPECT_EQ(result.segment_times.size(), result.path.size() - 1u);
    EXPECT_EQ(result.singularities.size(), result.path.size() - 1u);
    EXPECT_GE(result.dense_check_path.size(), result.path.size());
    EXPECT_NEAR(result.path.front().x, input.start.x, 1e-9);
    EXPECT_NEAR(result.path.front().y, input.start.y, 1e-9);
    EXPECT_NEAR(result.path.back().x, input.goal.x, 1e-9);
    EXPECT_NEAR(result.path.back().y, input.goal.y, 1e-9);

    bool deviated_around_obstacle = false;
    for (const auto& state : result.path) {
        EXPECT_TRUE(circleFree(state, planner.config().clearance));
        deviated_around_obstacle =
            deviated_around_obstacle || std::abs(state.y) > 0.20;
    }
    for (double dt : result.segment_times) {
        EXPECT_GT(dt, 0.0);
    }
    for (int singularity : result.singularities) {
        EXPECT_TRUE(singularity == 1 || singularity == -1);
    }
    EXPECT_TRUE(deviated_around_obstacle);
}

TEST(KinoAstarBasePlannerTest, OmnidirectionalModelMovesLaterallyWithTimedSegments) {
    cp::KinoAstarBasePlanner::Config cfg;
    cfg.kinematic_model = cp::KinoAstarBasePlanner::KinematicModel::Omnidirectional;
    cfg.xy_resolution = 0.08;
    cfg.yaw_resolution = 0.30;
    cfg.primitive_duration = 0.25;
    cfg.max_velocity = 0.40;
    cfg.max_lateral_velocity = 0.40;
    cfg.max_yaw_rate = 0.60;
    cfg.goal_xy_tolerance = 0.10;
    cfg.goal_yaw_tolerance = 0.20;
    cfg.map_min_x = -0.5;
    cfg.map_max_x = 1.5;
    cfg.map_min_y = -0.5;
    cfg.map_max_y = 1.5;
    cfg.max_expansions = 10000;
    cfg.oneshot_range = 0.0;

    const cp::KinoAstarBasePlanner planner(cfg);
    cp::KinoAstarBasePlanner::Input input;
    input.start = {0.0, 0.0, 0.0};
    input.goal = {0.0, 0.8, 0.0};

    const cp::KinoAstarBasePlanner::Result result = planner.plan(input);

    ASSERT_TRUE(result.success);
    ASSERT_GE(result.path.size(), 2u);
    ASSERT_EQ(result.segment_times.size(), result.path.size() - 1u);
    EXPECT_NEAR(result.path.front().x, input.start.x, 1e-9);
    EXPECT_NEAR(result.path.front().y, input.start.y, 1e-9);
    EXPECT_NEAR(result.path.back().x, input.goal.x, 1e-9);
    EXPECT_NEAR(result.path.back().y, input.goal.y, 1e-9);

    bool has_lateral_motion = false;
    for (std::size_t i = 1; i < result.path.size(); ++i) {
        EXPECT_GT(result.segment_times[i - 1], 0.0);
        has_lateral_motion =
            has_lateral_motion ||
            std::abs(result.path[i].y - result.path[i - 1].y) > 0.02;
    }
    EXPECT_TRUE(has_lateral_motion);
}

TEST(KinoAstarBasePlannerTest, RejectsBlockedStartOrGoal) {
    const cp::KinoAstarBasePlanner planner = makePlanner();
    cp::KinoAstarBasePlanner::Input input;
    input.start = {1.0, 0.0, 0.0};
    input.goal = {2.0, 0.0, 0.0};
    input.state_validator = circleFree;
    input.segment_validator = segmentFree;

    const cp::KinoAstarBasePlanner::Result result = planner.plan(input);
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.path.empty());
}
