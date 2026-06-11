#include <gtest/gtest.h>

#include "algorithm/cartesian_path_planner/base/kino_astar_base_planner.hpp"
#include "algorithm/cartesian_path_planner/collision/base_footprint_collision_checker.hpp"

#include <cmath>
#include <memory>

namespace cp = arm_controller::algorithm::cartesian_path_planner;

namespace {

class CylinderDistanceField final : public cp::DistanceFieldInterface {
public:
    bool isInsideMap(const Eigen::Vector3d& p) const override {
        return std::isfinite(p.x()) && std::isfinite(p.y()) &&
               p.x() >= -1.0 && p.x() <= 3.0 && p.y() >= -2.5 &&
               p.y() <= 2.5 && p.z() >= -0.2 && p.z() <= 1.2;
    }

    double getDistance(const Eigen::Vector3d& p) const override {
        const double dx = p.x() - 1.0;
        const double dy = p.y();
        return std::hypot(dx, dy) - 0.28;
    }

    Eigen::Vector3d getGradient(const Eigen::Vector3d& p) const override {
        Eigen::Vector3d g(p.x() - 1.0, p.y(), 0.0);
        const double n = g.norm();
        if (n < 1e-9) {
            return Eigen::Vector3d::UnitX();
        }
        return g / n;
    }
};

cp::BaseFootprintCollisionChecker makeChecker() {
    cp::BaseFootprintCollisionChecker::Config cfg;
    cfg.center_in_base = Eigen::Vector3d(0.0, 0.0, 0.22);
    cfg.size = Eigen::Vector3d(0.64, 0.64, 0.24);
    cfg.footprint_sample_resolution = 0.08;
    cfg.segment_sample_resolution = 0.06;
    cfg.yaw_sample_resolution = 0.20;
    cfg.unknown_is_free = true;
    return cp::BaseFootprintCollisionChecker(
        std::make_shared<CylinderDistanceField>(), cfg);
}

}  // namespace

TEST(BaseFootprintCollisionCheckerTest, RejectsBaseFootprintNearCylinder) {
    const cp::BaseFootprintCollisionChecker checker = makeChecker();

    EXPECT_FALSE(checker.isStateCollisionFree({1.0, 0.0, 0.0}, 0.05));
    EXPECT_TRUE(checker.isStateCollisionFree({1.0, 0.90, 0.0}, 0.05));
}

TEST(BaseFootprintCollisionCheckerTest, KinoAstarUsesFootprintToRouteAroundObstacle) {
    const cp::BaseFootprintCollisionChecker checker = makeChecker();

    cp::KinoAstarBasePlanner::Config cfg;
    cfg.xy_resolution = 0.08;
    cfg.yaw_resolution = 0.30;
    cfg.primitive_arc_length = 0.16;
    cfg.max_steer_angle = 0.80;
    cfg.goal_xy_tolerance = 0.12;
    cfg.goal_yaw_tolerance = 0.35;
    cfg.map_min_x = -0.4;
    cfg.map_max_x = 2.4;
    cfg.map_min_y = -2.2;
    cfg.map_max_y = 2.2;
    cfg.clearance = 0.05;
    cfg.max_expansions = 24000;

    cp::KinoAstarBasePlanner::Input input;
    input.start = {0.0, -0.55, 0.0};
    input.goal = {2.0, -0.55, 0.0};
    input.state_validator =
        [&checker](const cp::KinoAstarBasePlanner::BaseState& state,
                   const double clearance) {
            return checker.isStateCollisionFree(
                {state.x, state.y, state.yaw}, clearance);
        };
    input.segment_validator =
        [&checker](const cp::KinoAstarBasePlanner::BaseState& from,
                   const cp::KinoAstarBasePlanner::BaseState& to,
                   const double clearance) {
            return checker.isSegmentCollisionFree(
                {from.x, from.y, from.yaw},
                {to.x, to.y, to.yaw},
                clearance);
        };

    const cp::KinoAstarBasePlanner planner(cfg);
    const cp::KinoAstarBasePlanner::Result result = planner.plan(input);

    ASSERT_TRUE(result.success);
    bool deviated = false;
    for (const auto& state : result.path) {
        EXPECT_TRUE(checker.isStateCollisionFree(
            {state.x, state.y, state.yaw}, cfg.clearance));
        deviated = deviated || std::abs(state.y) > 0.30;
    }
    EXPECT_TRUE(deviated);
}
