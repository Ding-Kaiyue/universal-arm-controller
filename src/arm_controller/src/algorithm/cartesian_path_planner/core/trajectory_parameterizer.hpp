#pragma once

#include <memory>
#include <vector>

#include "algorithm/cartesian_path_planner/config/planner_common_config.hpp"
#include "algorithm/cartesian_path_planner/collision/cartesian_collision_checker.hpp"
#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "algorithm/cartesian_path_planner/types.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class TrajectoryParameterizer {
public:
    explicit TrajectoryParameterizer(
        const PlannerCommonConfig& cfg,
        std::shared_ptr<const DistanceFieldInterface> distance_field);

    TimedCartesianTrajectory parameterize(
        const CartesianPath& path,
        const Eigen::Matrix3d& R_start,
        const Eigen::Matrix3d& R_goal,
        double safe_distance
    ) const;
private:
    TimedCartesianTrajectory smoothWithInterpolator(
        const TimedCartesianTrajectory& in_traj,
        double safe_distance) const;

    bool isTrajectoryCollisionFree(
        const TimedCartesianTrajectory& traj,
        double safe_distance) const;

    PlannerCommonConfig cfg_;
    std::shared_ptr<const DistanceFieldInterface> distance_field_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
