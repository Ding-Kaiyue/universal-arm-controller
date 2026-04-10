#pragma once

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"

#include <memory>
#include <Eigen/Core>


namespace arm_controller::algorithm::cartesian_path_planner {

class ClearanceEvaluator {
public:
    explicit ClearanceEvaluator(
        std::shared_ptr<const DistanceFieldInterface> distance_field);

    double obstaclePenalty(const Eigen::Vector3d& p, double safe_distance) const;

private:
    std::shared_ptr<const DistanceFieldInterface> distance_field_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
