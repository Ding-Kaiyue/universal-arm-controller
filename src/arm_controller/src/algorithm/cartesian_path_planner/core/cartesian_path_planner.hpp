#pragma once

#include "algorithm/cartesian_path_planner/astar/astar_planner.hpp"
#include "algorithm/cartesian_path_planner/config/astar_config.hpp"
#include "algorithm/cartesian_path_planner/config/planner_common_config.hpp"
#include "algorithm/cartesian_path_planner/config/smoothing_config.hpp"
#include "algorithm/cartesian_path_planner/core/trajectory_parameterizer.hpp"
#include "algorithm/cartesian_path_planner/smoothing/shortcut_smoother.hpp"
#include "algorithm/cartesian_path_planner/types.hpp"

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"

#include <memory>

namespace arm_controller::algorithm::cartesian_path_planner {

class CartesianPathPlanner {
public:
    CartesianPathPlanner(
        const PlannerCommonConfig& common_cfg,
        const AStarConfig& astar_cfg,
        const SmoothingConfig& smoothing_cfg,
        std::shared_ptr<const DistanceFieldInterface> distance_field,
        const Eigen::Vector3d& map_min);

    PathPlanningOutput planPath(const PathPlanningInput& input);
    TimedCartesianTrajectory planTrajectory(const PathPlanningInput& input);

private:
    PlannerCommonConfig common_cfg_;
    std::shared_ptr<const DistanceFieldInterface> distance_field_;
    AStarPlanner astar_;
    ShortcutSmoother smoother_;
    TrajectoryParameterizer parameterizer_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
