#pragma once

#include <string>

#include "../types.hpp"

namespace arm_controller::algorithm::cartesian_path_planner::utils {

inline std::string summarizePath(const CartesianPath& path) {
    return "waypoints=" + std::to_string(path.waypoints.size()) + ", length=" +
           std::to_string(path.length);
}

}  // namespace arm_controller::algorithm::cartesian_path_planner::utils
