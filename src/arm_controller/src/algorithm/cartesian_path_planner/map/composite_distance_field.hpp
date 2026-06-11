#pragma once

#include <memory>
#include <vector>

#include "algorithm/cartesian_path_planner/map/distance_field_interface.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class CompositeDistanceField final : public DistanceFieldInterface {
public:
    void addField(std::shared_ptr<const DistanceFieldInterface> field);

    bool isInsideMap(const Eigen::Vector3d& p) const override;
    double getDistance(const Eigen::Vector3d& p) const override;
    Eigen::Vector3d getGradient(const Eigen::Vector3d& p) const override;
    DistanceFieldQueryResult queryDistanceAndGradient(
        const Eigen::Vector3d& p) const override;
    DistanceFieldQueryResultList queryDistanceAndGradientBatch(
        const Vector3dList& positions) const override;

private:
    std::vector<std::shared_ptr<const DistanceFieldInterface>> fields_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
