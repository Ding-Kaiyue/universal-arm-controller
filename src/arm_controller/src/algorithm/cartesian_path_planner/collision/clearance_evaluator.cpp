#include "algorithm/cartesian_path_planner/collision/clearance_evaluator.hpp"

#include <algorithm>

namespace arm_controller::algorithm::cartesian_path_planner {

ClearanceEvaluator::ClearanceEvaluator(
    std::shared_ptr<const DistanceFieldInterface> distance_field)
    : distance_field_(std::move(distance_field)) {}

double ClearanceEvaluator::obstaclePenalty(
    const Eigen::Vector3d& p, 
    double safe_distance) const {

    if (!distance_field_ || !distance_field_->isInsideMap(p)) {
        return 1e6;
    }

    const double d = distance_field_->getDistance(p);
    if (d < safe_distance) {
        return 1e6;  // Very high penalty for being inside the unsafe zone
    }
    constexpr double eps = 1e-6;

    return 1.0 / (d - safe_distance + eps);  // Penalty grows as we get closer to the unsafe zone
}



}  // namespace arm_controller::algorithm::cartesian_path_planner
