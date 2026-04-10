#pragma once

#include "path_smoother_interface.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class BsplineSmoother final : public PathSmootherInterface {
public:
    CartesianPath smooth(
        const CartesianPath& path,
        const PathPlanningInput& input) override;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
