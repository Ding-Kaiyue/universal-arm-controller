#pragma once

#include "algorithm/cartesian_path_planner/astar/grid_index.hpp"

#include <Eigen/Core>

namespace arm_controller::algorithm::cartesian_path_planner {

struct AStarNode {
    PoseGridIndex index;
    double g{0.0};
    double h{0.0};
    PoseGridIndex parent;
    bool has_parent{false};
    Eigen::VectorXd q;
    bool has_q{false};

    double f() const { return g + h; }
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
