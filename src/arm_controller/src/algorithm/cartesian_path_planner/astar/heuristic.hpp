#pragma once

#include <cmath>

#include "algorithm/cartesian_path_planner/astar/grid_index.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

inline double euclideanHeuristic(const GridIndex& a, const GridIndex& b) {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    const double dz = static_cast<double>(a.z - b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double se3GridHeuristic(
    const PoseGridIndex& a,
    const PoseGridIndex& b,
    double voxel_resolution,
    double orientation_bin_size_rad,
    double orientation_heuristic_weight) {
    const double dx = static_cast<double>(a.x - b.x) * voxel_resolution;
    const double dy = static_cast<double>(a.y - b.y) * voxel_resolution;
    const double dz = static_cast<double>(a.z - b.z) * voxel_resolution;
    const double h_xyz = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double h_ori = orientationBinDistanceRad(a, b, orientation_bin_size_rad);
    return h_xyz + orientation_heuristic_weight * h_ori;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
