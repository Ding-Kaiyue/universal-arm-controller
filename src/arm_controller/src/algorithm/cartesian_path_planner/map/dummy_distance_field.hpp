#pragma once

#include <vector>

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"
#include "algorithm/cartesian_path_planner/map/obstacle_primitives.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class DummyDistanceField final : public DistanceFieldInterface {
public:
    DummyDistanceField(
        const Eigen::Vector3d& map_min,
        const Eigen::Vector3d& map_max);

    void addSphere(const SphereObstacle& obs);
    void addBox(const BoxObstacle& obs);
    
    bool isInsideMap(const Eigen::Vector3d& p) const override;
    double getDistance(const Eigen::Vector3d& p) const override;
    Eigen::Vector3d getGradient(const Eigen::Vector3d& p) const override;

private:
    Eigen::Vector3d map_min_;
    Eigen::Vector3d map_max_;
    std::vector<SphereObstacle> spheres_;
    std::vector<BoxObstacle> boxes_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
