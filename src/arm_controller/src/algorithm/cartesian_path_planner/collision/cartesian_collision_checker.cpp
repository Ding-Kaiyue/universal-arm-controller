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
    return isStateValid(p, safe_distance, {});
}

bool CartesianCollisionChecker::isStateValid(
    const Eigen::Vector3d& p,
    const double safe_distance,
    const std::vector<PathPlanningInput::ForbiddenSphere>& forbidden_spheres) const {
    if (!distance_field_ || !distance_field_->isInsideMap(p)) {
        return false;  // Treat out-of-map as invalid
    }
    if (distance_field_->getDistance(p) < safe_distance) {
        return false;
    }
    for (const auto& forbidden : forbidden_spheres) {
        if (forbidden.radius > 0.0 && (p - forbidden.center).norm() < forbidden.radius) {
            return false;
        }
    }
    return true;
}

bool CartesianCollisionChecker::isSegmentValid(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    double safe_distance,
    double step) const {
    return isSegmentValid(p0, p1, safe_distance, step, {});
}

bool CartesianCollisionChecker::isSegmentValid(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const double safe_distance,
    const double step,
    const std::vector<PathPlanningInput::ForbiddenSphere>& forbidden_spheres) const {
    const double len = (p1 - p0).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int i = 0; i <= n; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(n);
        const Eigen::Vector3d p = (1.0 - s) * p0 + s * p1;
        if (!isStateValid(p, safe_distance, forbidden_spheres)) {
            return false;
        }
    }
    return true;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
