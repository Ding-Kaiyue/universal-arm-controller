#pragma once

#include "algorithm/cartesian_path_planner/config/astar_config.hpp"
#include "algorithm/cartesian_path_planner/collision/cartesian_collision_checker.hpp"
#include "algorithm/cartesian_path_planner/collision/clearance_evaluator.hpp"
#include "algorithm/cartesian_path_planner/types.hpp"

#include <memory>
#include <optional>

namespace arm_controller::algorithm::cartesian_path_planner {

class AStarPlanner final {
public:
    AStarPlanner(
        const AStarConfig& cfg,
        std::shared_ptr<const DistanceFieldInterface> distance_field,
        const Eigen::Vector3d& map_min);

    PathPlanningOutput plan(const PathPlanningInput& input);

private:
    AStarConfig cfg_;
    Eigen::Vector3d map_min_;

    std::shared_ptr<const DistanceFieldInterface> distance_field_;
    CartesianCollisionChecker collision_checker_;
    ClearanceEvaluator clearance_evaluator_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
