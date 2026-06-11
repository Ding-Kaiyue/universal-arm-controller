#include "algorithm/cartesian_path_planner/map/static_pillar_distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace arm_controller::algorithm::cartesian_path_planner {

StaticPillarDistanceField::StaticPillarDistanceField(
    std::vector<Pillar> pillars,
    const Eigen::Vector3d& map_min,
    const Eigen::Vector3d& map_max)
    : pillars_(std::move(pillars)),
      map_min_(map_min),
      map_max_(map_max) {}

std::shared_ptr<StaticPillarDistanceField>
StaticPillarDistanceField::makeDefaultPillarWorld() {
    std::vector<Pillar> pillars;
    pillars.reserve(9u);
    const double radius = 0.075;
    const double z_min = 0.0;
    const double z_max = 1.2;
    const Eigen::Vector2d centers[] = {
        {1.10, 0.55},
        {1.45, -0.75},
        {1.95, 0.20},
        {2.35, -1.05},
        {2.65, 0.75},
        {3.20, -0.25},
        {3.65, 1.05},
        {4.05, -0.85},
        {4.55, 0.35},
    };
    for (const Eigen::Vector2d& center : centers) {
        pillars.push_back(Pillar{center, radius, z_min, z_max});
    }
    return std::make_shared<StaticPillarDistanceField>(
        std::move(pillars),
        Eigen::Vector3d(-2.0, -2.0, -0.2),
        Eigen::Vector3d(6.0, 2.0, 2.0));
}

bool StaticPillarDistanceField::isInsideMap(const Eigen::Vector3d& p) const {
    return (p.array() >= map_min_.array()).all() &&
           (p.array() <= map_max_.array()).all();
}

double StaticPillarDistanceField::getDistance(const Eigen::Vector3d& p) const {
    return queryDistanceAndGradient(p).distance;
}

Eigen::Vector3d StaticPillarDistanceField::getGradient(
    const Eigen::Vector3d& p) const {
    return queryDistanceAndGradient(p).gradient;
}

DistanceFieldQueryResult StaticPillarDistanceField::queryDistanceAndGradient(
    const Eigen::Vector3d& p) const {
    DistanceFieldQueryResult result;
    if (!isInsideMap(p) || pillars_.empty()) {
        return result;
    }

    double best_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_gradient = Eigen::Vector3d::UnitX();
    for (const Pillar& pillar : pillars_) {
        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
        const double distance = signedDistanceToPillar(p, pillar, &gradient);
        if (distance < best_distance) {
            best_distance = distance;
            best_gradient = gradient;
        }
    }

    result.observed = true;
    result.distance = best_distance;
    result.distance_valid = std::isfinite(best_distance);
    result.gradient = best_gradient;
    result.gradient_valid = best_gradient.allFinite();
    return result;
}

double StaticPillarDistanceField::signedDistanceToPillar(
    const Eigen::Vector3d& p,
    const Pillar& pillar,
    Eigen::Vector3d* gradient) {
    const Eigen::Vector2d d_xy = p.head<2>() - pillar.center_xy;
    const double r_xy = d_xy.norm();
    const double outside_xy = r_xy - pillar.radius;
    const double outside_z =
        std::max(std::max(pillar.z_min - p.z(), p.z() - pillar.z_max), 0.0);

    if (outside_xy > 0.0 || outside_z > 0.0) {
        const double dx = std::max(outside_xy, 0.0);
        const double dz = outside_z;
        const double dist = std::hypot(dx, dz);
        Eigen::Vector3d g = Eigen::Vector3d::Zero();
        if (dist > 1e-9) {
            if (dx > 0.0 && r_xy > 1e-9) {
                g.head<2>() = (dx / dist) * (d_xy / r_xy);
            }
            if (dz > 0.0) {
                g.z() = (p.z() > pillar.z_max ? 1.0 : -1.0) * dz / dist;
            }
        } else if (r_xy > 1e-9) {
            g.head<2>() = d_xy / r_xy;
        } else {
            g = Eigen::Vector3d::UnitX();
        }
        if (gradient != nullptr) {
            *gradient = g;
        }
        return dist;
    }

    const double inside_radial = pillar.radius - r_xy;
    const double inside_bottom = p.z() - pillar.z_min;
    const double inside_top = pillar.z_max - p.z();
    double min_inside = inside_radial;
    Eigen::Vector3d g = r_xy > 1e-9
                            ? Eigen::Vector3d(d_xy.x() / r_xy, d_xy.y() / r_xy, 0.0)
                            : Eigen::Vector3d::UnitX();
    if (inside_bottom < min_inside) {
        min_inside = inside_bottom;
        g = -Eigen::Vector3d::UnitZ();
    }
    if (inside_top < min_inside) {
        min_inside = inside_top;
        g = Eigen::Vector3d::UnitZ();
    }
    if (gradient != nullptr) {
        *gradient = g;
    }
    return -min_inside;
}

}  // namespace arm_controller::algorithm::cartesian_path_planner
