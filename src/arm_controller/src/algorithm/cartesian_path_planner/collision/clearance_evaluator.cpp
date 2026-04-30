#include "algorithm/cartesian_path_planner/collision/clearance_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::algorithm::cartesian_path_planner {

ClearanceEvaluator::ClearanceEvaluator(
    std::shared_ptr<const DistanceFieldInterface> distance_field)
    : distance_field_(std::move(distance_field)) {}

double ClearanceEvaluator::obstaclePenalty(
    const Eigen::Vector3d& p,
    const double desired_clearance) const {
    if (!distance_field_) {
        return 1e6;
    }

    const DistanceFieldQueryResult query =
        distance_field_->queryDistanceAndGradient(p);
    if (!query.observed || !query.distance_valid) {
        return 0.0;
    }
    if (desired_clearance <= 1e-6) {
        return 0.0;
    }

    const double shell_band = std::max(1e-3, desired_clearance);
    const double delta = std::abs(query.distance - desired_clearance);
    const double ratio = delta / shell_band;
    if (ratio <= 1.0) {
        return ratio * ratio;
    }
    return 1.0 + 0.25 * (ratio - 1.0);
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
