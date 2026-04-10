#include "algorithm/cartesian_path_planner/map/dummy_distance_field.hpp"

#include <limits>

namespace arm_controller::algorithm::cartesian_path_planner {

DummyDistanceField::DummyDistanceField(
    const Eigen::Vector3d& map_min,
    const Eigen::Vector3d& map_max)
    : map_min_(map_min), map_max_(map_max) {}

void DummyDistanceField::addSphere(const SphereObstacle& obs) {
    spheres_.push_back(obs);
}

void DummyDistanceField::addBox(const BoxObstacle& obs) {
    boxes_.push_back(obs);
}

bool DummyDistanceField::isInsideMap(const Eigen::Vector3d& p) const {
    return (p.array() >= map_min_.array()).all() && 
           (p.array() <= map_max_.array()).all();
}

double DummyDistanceField::getDistance(const Eigen::Vector3d& p) const {
    if (!isInsideMap(p)) {
        return -1.0;  // Outside map
    }
    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& s : spheres_) {
        min_dist = std::min(min_dist, distanceToSphere(p, s));
    }
    for (const auto& b : boxes_) {
        min_dist = std::min(min_dist, distanceToAABB(p, b));
    }
    if (std::isinf(min_dist)) {
        return 10.0;  // Invalid distance, treat as inside
    }
    return min_dist;
}

Eigen::Vector3d DummyDistanceField::getGradient(const Eigen::Vector3d& p) const {
    constexpr double eps = 1e-3;
    const Eigen::Vector3d dx(eps, 0, 0);
    const Eigen::Vector3d dy(0, eps, 0);
    const Eigen::Vector3d dz(0, 0, eps);

    const double gx = (getDistance(p + dx) - getDistance(p - dx)) / (2 * eps);
    const double gy = (getDistance(p + dy) - getDistance(p - dy)) / (2 * eps);
    const double gz = (getDistance(p + dz) - getDistance(p - dz)) / (2 * eps);

    return Eigen::Vector3d(gx, gy, gz);
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
