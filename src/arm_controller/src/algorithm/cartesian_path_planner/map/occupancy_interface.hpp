#pragma once

#include <Eigen/Core>

namespace arm_controller::algorithm::cartesian_path_planner {

class OccupancyInterface {
public:
    virtual ~OccupancyInterface() = default;
    virtual bool isOccupied(const Eigen::Vector3d& p) const = 0;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
