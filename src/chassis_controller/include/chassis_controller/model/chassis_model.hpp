#pragma once

#include <string>

namespace chassis_controller {

enum class ChassisType {
    Fixed,
    Differential,
    Ackermann,
    Omnidirectional,
    Mecanum,
};

const char* toString(ChassisType type);
ChassisType chassisTypeFromString(const std::string& value);

struct ChassisModel {
    ChassisType type{ChassisType::Fixed};
    std::string name{"fixed"};

    bool supportsLongitudinalVelocity() const;
    bool supportsLateralVelocity() const;
    bool supportsYawRate() const;
};

}  // namespace chassis_controller
