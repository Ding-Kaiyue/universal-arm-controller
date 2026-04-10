#pragma once

#include <vector>

#include "algorithm/cartesian_path_planner/astar/grid_index.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

inline std::vector<GridIndex> getNeighborOffsets(int neighbor_mode) {
    std::vector<GridIndex> offsets;
    if (neighbor_mode != 6 && neighbor_mode != 18 && neighbor_mode != 26) {
        neighbor_mode = 26;
    }

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }

                const int non_zero = (dx != 0) + (dy != 0) + (dz != 0);
                if (neighbor_mode == 6 && non_zero != 1) {
                    continue;
                }
                if (neighbor_mode == 18 && non_zero == 3) {
                    continue;
                }
                offsets.push_back(GridIndex{dx, dy, dz});
            }
        }
    }

    return offsets;
}

inline std::vector<PoseGridIndex> getPoseNeighborOffsets(
    int neighbor_mode,
    bool force_axis_translation_neighbors,
    bool enable_inplace_rotation_neighbors) {
    std::vector<PoseGridIndex> offsets;
    const int trans_mode = force_axis_translation_neighbors ? 6 : neighbor_mode;
    const auto trans = getNeighborOffsets(trans_mode);
    offsets.reserve(trans.size() + 6);

    for (const auto& t : trans) {
        offsets.push_back(PoseGridIndex{t.x, t.y, t.z, 0, 0, 0});
    }
    if (enable_inplace_rotation_neighbors) {
        offsets.push_back(PoseGridIndex{0, 0, 0, +1, 0, 0});
        offsets.push_back(PoseGridIndex{0, 0, 0, -1, 0, 0});
        offsets.push_back(PoseGridIndex{0, 0, 0, 0, +1, 0});
        offsets.push_back(PoseGridIndex{0, 0, 0, 0, -1, 0});
        offsets.push_back(PoseGridIndex{0, 0, 0, 0, 0, +1});
        offsets.push_back(PoseGridIndex{0, 0, 0, 0, 0, -1});
    }
    return offsets;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
