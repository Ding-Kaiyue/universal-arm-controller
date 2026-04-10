#pragma once

#include <algorithm>

namespace arm_controller::algorithm::cartesian_path_planner::utils {

template <typename T>
inline T clamp(T v, T lo, T hi) {
    return std::max(lo, std::min(v, hi));
}

}  // namespace arm_controller::algorithm::cartesian_path_planner::utils
