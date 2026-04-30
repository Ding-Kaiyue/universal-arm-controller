#pragma once

#include <optional>
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
        double hard_clearance
    ) const;
private:
    TimedCartesianTrajectory smoothWithInterpolator(
        const TimedCartesianTrajectory& in_traj,
        double hard_clearance) const;

    std::optional<TimedCartesianTrajectory> tryMinimumSnapOptimization(
        const TimedCartesianTrajectory& in_traj,
        const TimedCartesianTrajectory& anchor_traj,
        double hard_clearance) const;

    TimedCartesianTrajectory buildResampledTrajectory(
        const TimedCartesianTrajectory& source_traj,
        const TimedCartesianTrajectory& anchor_traj) const;

    CartesianWaypoint sampleWaypointAtTime(
        const TimedCartesianTrajectory& traj,
        double time_from_start) const;

    bool isTrajectoryCollisionFree(
        const TimedCartesianTrajectory& traj,
        double hard_clearance) const;

    PlannerCommonConfig cfg_;
    std::shared_ptr<const DistanceFieldInterface> distance_field_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
