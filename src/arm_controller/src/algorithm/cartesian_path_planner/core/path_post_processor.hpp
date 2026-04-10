#pragma once

#include "../types.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class PathPostProcessor {
public:
    CartesianPath process(const CartesianPath& in) const;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
