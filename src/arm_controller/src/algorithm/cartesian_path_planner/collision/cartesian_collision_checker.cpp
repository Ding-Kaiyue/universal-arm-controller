#include "algorithm/cartesian_path_planner/collision/cartesian_collision_checker.hpp"

#include <algorithm>
#include <cmath>

namespace arm_controller::algorithm::cartesian_path_planner {

CartesianCollisionChecker::CartesianCollisionChecker(
    std::shared_ptr<const DistanceFieldInterface> distance_field)
    : distance_field_(std::move(distance_field)) {}

bool CartesianCollisionChecker::isStateValid(
    const Eigen::Vector3d& p,
    double safe_distance) const {
    if (!distance_field_) {
        return false;
    }

    const DistanceFieldQueryResult query =
        distance_field_->queryDistanceAndGradient(p);
    if (!query.observed || !query.distance_valid) {
        return true;
    }
    if (query.distance < safe_distance) {
        return false;
    }
    return true;
}

bool CartesianCollisionChecker::isSegmentValid(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    double safe_distance,
    double step) const {
    if (!distance_field_) {
        return false;
    }
    const double len = (p1 - p0).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    std::vector<Eigen::Vector3d> sample_points;
    sample_points.reserve(static_cast<std::size_t>(n + 1));
    for (int i = 0; i <= n; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(n);
        sample_points.push_back((1.0 - s) * p0 + s * p1);
    }

    const std::vector<DistanceFieldQueryResult> queries =
        distance_field_->queryDistanceAndGradientBatch(sample_points);
    for (std::size_t i = 0; i < sample_points.size(); ++i) {
        const DistanceFieldQueryResult& query = queries[i];
        if (!query.observed || !query.distance_valid) {
            continue;
        }
        if (query.distance < safe_distance) {
            return false;
        }
    }
    return true;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
