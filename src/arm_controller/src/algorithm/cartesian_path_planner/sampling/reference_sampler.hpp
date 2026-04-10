#pragma once

#include "algorithm/cartesian_path_planner/types.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class ReferenceSampler {
public:
    TimedCartesianSample sample(
        const TimedCartesianTrajectory& traj,
        double t_query) const;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner