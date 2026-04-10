#include "path_post_processor.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

CartesianPath PathPostProcessor::process(const CartesianPath& in) const {
    // Placeholder: keep identity behavior first, then add pruning/smoothing pipeline.
    return in;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
