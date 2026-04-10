#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include "algorithm/cartesian_path_planner/config/astar_config.hpp"
#include "algorithm/cartesian_path_planner/config/planner_common_config.hpp"
#include "algorithm/cartesian_path_planner/config/smoothing_config.hpp"
#include "algorithm/cartesian_path_planner/core/cartesian_path_planner.hpp"
#include "algorithm/cartesian_path_planner/map/dummy_distance_field.hpp"
#include "algorithm/cartesian_path_planner/map/obstacle_primitives.hpp"

namespace cp = arm_controller::algorithm::cartesian_path_planner;
namespace fs = std::filesystem;

namespace {

std::string getVizDir() {
    const char* env = std::getenv("ASTAR_TEST_VIZ_DIR");
    if (env == nullptr || std::string(env).empty()) {
        return "";
    }
    return std::string(env);
}

void dumpCaseJson(
    const std::string& case_name,
    const Eigen::Vector3d& map_min,
    const Eigen::Vector3d& map_max,
    const cp::PathPlanningInput& in,
    const cp::PathPlanningOutput& out,
    const std::vector<cp::SphereObstacle>& spheres,
    const std::vector<cp::BoxObstacle>& boxes) {
    const std::string dir = getVizDir();
    if (dir.empty()) {
        return;
    }

    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        return;
    }

    const fs::path path = fs::path(dir) / (case_name + ".json");
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        return;
    }

    ofs << std::fixed << std::setprecision(6);
    ofs << "{\n";
    ofs << "  \"case\": \"" << case_name << "\",\n";
    ofs << "  \"success\": " << (out.success ? "true" : "false") << ",\n";
    ofs << "  \"map_min\": [" << map_min.x() << ", " << map_min.y() << ", " << map_min.z() << "],\n";
    ofs << "  \"map_max\": [" << map_max.x() << ", " << map_max.y() << ", " << map_max.z() << "],\n";
    ofs << "  \"start\": [" << in.p_start.x() << ", " << in.p_start.y() << ", " << in.p_start.z() << "],\n";
    ofs << "  \"goal\": [" << in.p_goal.x() << ", " << in.p_goal.y() << ", " << in.p_goal.z() << "],\n";

    ofs << "  \"spheres\": [\n";
    for (size_t i = 0; i < spheres.size(); ++i) {
        const auto& s = spheres[i];
        ofs << "    {\"center\": [" << s.center.x() << ", " << s.center.y() << ", " << s.center.z()
            << "], \"radius\": " << s.radius << "}";
        ofs << (i + 1 == spheres.size() ? "\n" : ",\n");
    }
    ofs << "  ],\n";

    ofs << "  \"boxes\": [\n";
    for (size_t i = 0; i < boxes.size(); ++i) {
        const auto& b = boxes[i];
        ofs << "    {\"min\": [" << b.min_corner.x() << ", " << b.min_corner.y() << ", " << b.min_corner.z()
            << "], \"max\": [" << b.max_corner.x() << ", " << b.max_corner.y() << ", " << b.max_corner.z() << "]}";
        ofs << (i + 1 == boxes.size() ? "\n" : ",\n");
    }
    ofs << "  ],\n";

    ofs << "  \"path\": [\n";
    for (size_t i = 0; i < out.path.waypoints.size(); ++i) {
        const auto& p = out.path.waypoints[i].position;
        ofs << "    [" << p.x() << ", " << p.y() << ", " << p.z() << "]";
        ofs << (i + 1 == out.path.waypoints.size() ? "\n" : ",\n");
    }
    ofs << "  ],\n";

    ofs << "  \"path_orientations\": [\n";
    for (size_t i = 0; i < out.path.waypoints.size(); ++i) {
        const auto& R = out.path.waypoints[i].orientation;
        ofs << "    ["
            << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2) << ", "
            << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2) << ", "
            << R(2, 0) << ", " << R(2, 1) << ", " << R(2, 2) << "]";
        ofs << (i + 1 == out.path.waypoints.size() ? "\n" : ",\n");
    }
    ofs << "  ]\n";
    ofs << "}\n";
}

}  // namespace

TEST(CartesianPathPlannerAStarTest, FindsPathInEmptyMap) {
    const Eigen::Vector3d map_min(0.0, 0.0, 0.0);
    const Eigen::Vector3d map_max(1.0, 1.0, 1.0);
    auto map = std::make_shared<cp::DummyDistanceField>(
        map_min, map_max);

    cp::PlannerCommonConfig common_cfg;
    cp::AStarConfig astar_cfg;
    astar_cfg.use_se3_search = true;
    astar_cfg.voxel_resolution = 0.05;
    astar_cfg.neighbor_mode = 26;
    astar_cfg.orientation_bin_size_rad = 0.5235987756;  // 30 deg
    astar_cfg.orientation_goal_tolerance_rad = 1.2;
    astar_cfg.max_iterations = 300000;
    astar_cfg.max_planning_time_sec = 1.0;

    cp::SmoothingConfig smoothing_cfg;

    cp::CartesianPathPlanner planner(
        common_cfg, astar_cfg, smoothing_cfg, map, Eigen::Vector3d(0.0, 0.0, 0.0));

    cp::PathPlanningInput in;
    in.p_start = Eigen::Vector3d(0.10, 0.10, 0.10);
    in.p_goal = Eigen::Vector3d(0.90, 0.90, 0.90);
    in.R_start = Eigen::Matrix3d::Identity();
    in.R_goal = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    in.safe_distance = 0.02;
    in.goal_tolerance = 0.08;

    const auto out = planner.planPath(in);
    dumpCaseJson("case_empty_map", map_min, map_max, in, out, {}, {});
    ASSERT_TRUE(out.success);
    ASSERT_GE(out.path.waypoints.size(), 2u);
    EXPECT_GT(out.path.length, 0.0);
}

TEST(CartesianPathPlannerAStarTest, FindsDetourAroundSphereObstacle) {
    const Eigen::Vector3d map_min(0.0, 0.0, 0.0);
    const Eigen::Vector3d map_max(1.0, 1.0, 1.0);
    auto map = std::make_shared<cp::DummyDistanceField>(
        map_min, map_max);

    cp::SphereObstacle obs;
    obs.center = Eigen::Vector3d(0.50, 0.50, 0.50);
    obs.radius = 0.20;
    map->addSphere(obs);

    cp::PlannerCommonConfig common_cfg;
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

    cp::CartesianPathPlanner planner(
        common_cfg, astar_cfg, smoothing_cfg, map, Eigen::Vector3d(0.0, 0.0, 0.0));

    cp::PathPlanningInput in;
    in.p_start = Eigen::Vector3d(0.10, 0.10, 0.10);
    in.p_goal = Eigen::Vector3d(0.90, 0.90, 0.90);
    in.R_start = Eigen::Matrix3d::Identity();
    in.R_goal = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    in.safe_distance = 0.03;
    in.goal_tolerance = 0.08;

    const auto out = planner.planPath(in);
    dumpCaseJson("case_sphere_detour", map_min, map_max, in, out, {obs}, {});
    ASSERT_TRUE(out.success);
    ASSERT_GE(out.path.waypoints.size(), 2u);

    const double straight = (in.p_goal - in.p_start).norm();
    EXPECT_GE(out.path.length, straight);
}

TEST(CartesianPathPlannerAStarTest, ReportsNoPathWhenWallBlocksSpace) {
    const Eigen::Vector3d map_min(0.0, 0.0, 0.0);
    const Eigen::Vector3d map_max(1.0, 1.0, 1.0);
    auto map = std::make_shared<cp::DummyDistanceField>(
        map_min, map_max);

    cp::BoxObstacle wall;
    wall.min_corner = Eigen::Vector3d(0.0, 0.45, 0.0);
    wall.max_corner = Eigen::Vector3d(1.0, 0.55, 1.0);
    map->addBox(wall);

    cp::PlannerCommonConfig common_cfg;
    cp::AStarConfig astar_cfg;
    astar_cfg.use_se3_search = true;
    astar_cfg.voxel_resolution = 0.05;
    astar_cfg.neighbor_mode = 26;
    astar_cfg.orientation_bin_size_rad = 0.5235987756;  // 30 deg
    astar_cfg.orientation_goal_tolerance_rad = 1.2;
    astar_cfg.max_iterations = 200000;
    astar_cfg.max_planning_time_sec = 0.5;

    cp::SmoothingConfig smoothing_cfg;

    cp::CartesianPathPlanner planner(
        common_cfg, astar_cfg, smoothing_cfg, map, Eigen::Vector3d(0.0, 0.0, 0.0));

    cp::PathPlanningInput in;
    in.p_start = Eigen::Vector3d(0.20, 0.20, 0.20);
    in.p_goal = Eigen::Vector3d(0.80, 0.80, 0.80);
    in.R_start = Eigen::Matrix3d::Identity();
    in.R_goal = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    in.safe_distance = 0.02;
    in.goal_tolerance = 0.08;

    const auto out = planner.planPath(in);
    dumpCaseJson("case_wall_blocked", map_min, map_max, in, out, {}, {wall});
    EXPECT_FALSE(out.success);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
