#pragma once

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "algorithm/cartesian_path_planner/types.hpp"
#include <memory>
#include <Eigen/Core>

namespace arm_controller::algorithm::cartesian_path_planner {

class CartesianCollisionChecker {
public:
    explicit CartesianCollisionChecker(
        std::shared_ptr<const DistanceFieldInterface> distance_field);

    bool isStateValid(const Eigen::Vector3d& p, double safe_distance) const;
    bool isStateValid(
        const Eigen::Vector3d& p,
        double safe_distance,
        const std::vector<PathPlanningInput::ForbiddenSphere>& forbidden_spheres) const;
    bool isSegmentValid(
        const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1,
        double safe_distance,
        double step) const;
    bool isSegmentValid(
        const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1,
        double safe_distance,
        double step,
        const std::vector<PathPlanningInput::ForbiddenSphere>& forbidden_spheres) const;

private:
    std::shared_ptr<const DistanceFieldInterface> distance_field_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
