#include "bspline_smoother.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

CartesianPath BsplineSmoother::smooth(
    const CartesianPath& path,
    const PathPlanningInput& input) {
    // Placeholder: keep identity behavior first.
    (void)input;
    return path;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
