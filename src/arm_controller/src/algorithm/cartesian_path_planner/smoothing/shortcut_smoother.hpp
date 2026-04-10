#pragma once

#include "algorithm/cartesian_path_planner/config/smoothing_config.hpp"
#include "algorithm/cartesian_path_planner/collision/cartesian_collision_checker.hpp"
#include "algorithm/cartesian_path_planner/smoothing/path_smoother_interface.hpp"

#include <random>

namespace arm_controller::algorithm::cartesian_path_planner {

class ShortcutSmoother final : public PathSmootherInterface {
public:
    ShortcutSmoother(
        const SmoothingConfig& cfg, 
        std::shared_ptr<const DistanceFieldInterface> distance_field);

    CartesianPath smooth(
        const CartesianPath& raw_path,
        const PathPlanningInput& input) override;

private:
    SmoothingConfig cfg_;
    CartesianCollisionChecker collision_checker_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
