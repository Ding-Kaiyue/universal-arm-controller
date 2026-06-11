#pragma once

#include <memory>
#include <vector>

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class StaticPillarDistanceField final : public DistanceFieldInterface {
public:
    struct Pillar {
        Eigen::Vector2d center_xy{Eigen::Vector2d::Zero()};
        double radius{0.075};
        double z_min{0.0};
        double z_max{1.2};
    };

    StaticPillarDistanceField(
        std::vector<Pillar> pillars,
        const Eigen::Vector3d& map_min,
        const Eigen::Vector3d& map_max);

    static std::shared_ptr<StaticPillarDistanceField> makeDefaultPillarWorld();

    bool isInsideMap(const Eigen::Vector3d& p) const override;
    double getDistance(const Eigen::Vector3d& p) const override;
    Eigen::Vector3d getGradient(const Eigen::Vector3d& p) const override;
    DistanceFieldQueryResult queryDistanceAndGradient(
        const Eigen::Vector3d& p) const override;

private:
    static double signedDistanceToPillar(
        const Eigen::Vector3d& p,
        const Pillar& pillar,
        Eigen::Vector3d* gradient);

    std::vector<Pillar> pillars_;
    Eigen::Vector3d map_min_;
    Eigen::Vector3d map_max_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
