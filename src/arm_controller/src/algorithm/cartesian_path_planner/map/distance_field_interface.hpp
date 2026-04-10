#pragma once

#include <Eigen/Core>

namespace arm_controller::algorithm::cartesian_path_planner {

class DistanceFieldInterface {
public:
    virtual ~DistanceFieldInterface() = default;
    
    virtual bool isInsideMap(const Eigen::Vector3d& p) const = 0;
    virtual double getDistance(const Eigen::Vector3d& p) const = 0;
    virtual Eigen::Vector3d getGradient(const Eigen::Vector3d& p) const = 0;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
