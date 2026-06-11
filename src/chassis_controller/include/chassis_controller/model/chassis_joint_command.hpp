#pragma once

#include <string>
#include <vector>

namespace chassis_controller {

struct ChassisJointCommand {
    std::string name;
    bool has_position{false};
    bool has_velocity{false};
    double position{0.0};
    double velocity{0.0};
};

struct ChassisJointCommandSet {
    std::vector<ChassisJointCommand> joints;

    bool empty() const;
};

}  // namespace chassis_controller
