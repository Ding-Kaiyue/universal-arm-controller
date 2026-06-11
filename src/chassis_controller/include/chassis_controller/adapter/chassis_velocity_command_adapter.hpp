#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <string>

#include "chassis_controller/model/chassis_velocity_command.hpp"
#include "chassis_controller/model/chassis_model.hpp"

namespace chassis_controller {

struct CommandValidation {
    bool ok{true};
    std::string error;
};

class ChassisVelocityCommandAdapter {
public:
    explicit ChassisVelocityCommandAdapter(ChassisModel model);

    const ChassisModel& model() const;

    CommandValidation validate(const ChassisVelocityCommand& command) const;
    ChassisVelocityCommand fromTwist(const geometry_msgs::msg::Twist& twist) const;
    geometry_msgs::msg::Twist toTwist(const ChassisVelocityCommand& command) const;

private:
    ChassisModel model_;
};

}  // namespace chassis_controller
