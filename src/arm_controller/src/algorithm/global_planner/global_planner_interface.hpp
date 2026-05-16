#pragma once

#include "algorithm/cartesian_path_planner/types.hpp"

namespace arm_controller::algorithm::global_planner {

namespace cp = arm_controller::algorithm::cartesian_path_planner;

class GlobalPlannerInterface {
public:
    virtual ~GlobalPlannerInterface() = default;

    virtual cp::TimedJointTrajectory planTrajectory(
        const cp::PathPlanningInput& input) = 0;
};

}  // namespace arm_controller::algorithm::global_planner
