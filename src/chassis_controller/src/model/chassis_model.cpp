#include "chassis_controller/model/chassis_model.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace chassis_controller {

const char* toString(const ChassisType type) {
    switch (type) {
        case ChassisType::Fixed:
            return "fixed";
        case ChassisType::Differential:
            return "differential";
        case ChassisType::Ackermann:
            return "ackermann";
        case ChassisType::Omnidirectional:
            return "omnidirectional";
        case ChassisType::Mecanum:
            return "mecanum";
    }
    return "fixed";
}

ChassisType chassisTypeFromString(const std::string& input) {
    std::string value = input;
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "fixed") {
        return ChassisType::Fixed;
    }
    if (value == "differential" || value == "diff") {
        return ChassisType::Differential;
    }
    if (value == "ackermann") {
        return ChassisType::Ackermann;
    }
    if (value == "omnidirectional" || value == "omni") {
        return ChassisType::Omnidirectional;
    }
    if (value == "mecanum") {
        return ChassisType::Mecanum;
    }
    throw std::invalid_argument("unknown chassis_type: " + value);
}

bool ChassisModel::supportsLongitudinalVelocity() const {
    return type != ChassisType::Fixed;
}

bool ChassisModel::supportsLateralVelocity() const {
    return type == ChassisType::Omnidirectional || type == ChassisType::Mecanum;
}

bool ChassisModel::supportsYawRate() const {
    return type != ChassisType::Fixed;
}

}  // namespace chassis_controller
