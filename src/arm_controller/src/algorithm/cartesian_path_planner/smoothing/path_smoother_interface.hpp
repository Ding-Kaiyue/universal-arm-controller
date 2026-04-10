#pragma once

#include "algorithm/cartesian_path_planner/types.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class PathSmootherInterface {
public:
    virtual ~PathSmootherInterface() = default;
    virtual CartesianPath smooth(
        const CartesianPath& raw_path,
        const PathPlanningInput& input) = 0;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
