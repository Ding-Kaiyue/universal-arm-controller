#pragma once

#include <memory>

#include "distance_field_interface.hpp"

namespace arm_controller::algorithm::cartesian_path_planner {

class MapAdapter {
public:
    explicit MapAdapter(std::shared_ptr<DistanceFieldInterface> distance_field)
        : distance_field_(std::move(distance_field)) {}

    const std::shared_ptr<DistanceFieldInterface>& getDistanceField() const {
        return distance_field_;
    }

private:
    std::shared_ptr<DistanceFieldInterface> distance_field_;
};

}  // namespace arm_controller::algorithm::cartesian_path_planner
