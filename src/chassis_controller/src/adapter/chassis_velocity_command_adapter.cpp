#include "chassis_controller/adapter/chassis_velocity_command_adapter.hpp"

#include <type_traits>

namespace chassis_controller {
namespace {

template <typename T>
bool isVariant(const ChassisVelocityCommand& command) {
    return std::holds_alternative<T>(command);
}

}  // namespace

ChassisVelocityCommandAdapter::ChassisVelocityCommandAdapter(ChassisModel model) : model_(std::move(model)) {}

const ChassisModel& ChassisVelocityCommandAdapter::model() const {
    return model_;
}

CommandValidation ChassisVelocityCommandAdapter::validate(const ChassisVelocityCommand& command) const {
    switch (model_.type) {
        case ChassisType::Fixed:
            if (isVariant<FixedChassisVelocityCommand>(command)) {
                return {};
            }
            return {false, "fixed chassis only accepts FixedChassisVelocityCommand"};
        case ChassisType::Differential:
            if (isVariant<DifferentialCommand>(command)) {
                return {};
            }
            return {false, "differential chassis expects DifferentialCommand(vx, omega)"};
        case ChassisType::Ackermann:
            if (isVariant<AckermannCommand>(command)) {
                return {};
            }
            return {false, "ackermann chassis expects AckermannCommand(vx, omega)"};
        case ChassisType::Omnidirectional:
            if (isVariant<OmnidirectionalCommand>(command)) {
                return {};
            }
            return {false,
                    "omnidirectional chassis expects OmnidirectionalCommand(vx, "
                    "vy, omega)"};
        case ChassisType::Mecanum:
            if (isVariant<MecanumCommand>(command)) {
                return {};
            }
            return {false, "mecanum chassis expects MecanumCommand(vx, vy, omega)"};
    }
    return {false, "unknown chassis model"};
}

geometry_msgs::msg::Twist ChassisVelocityCommandAdapter::toTwist(const ChassisVelocityCommand& command) const {
    geometry_msgs::msg::Twist twist;

    std::visit(
        [&](const auto& typed_command) {
            using CommandT = std::decay_t<decltype(typed_command)>;
            if constexpr (std::is_same_v<CommandT, FixedChassisVelocityCommand>) {
                return;
            } else if constexpr (std::is_same_v<CommandT, DifferentialCommand> ||
                                 std::is_same_v<CommandT, AckermannCommand>) {
                twist.linear.x = typed_command.vx;
                twist.angular.z = typed_command.omega;
            } else {
                twist.linear.x = typed_command.vx;
                twist.linear.y = typed_command.vy;
                twist.angular.z = typed_command.omega;
            }
        },
        command);

    return twist;
}

ChassisVelocityCommand ChassisVelocityCommandAdapter::fromTwist(const geometry_msgs::msg::Twist& twist) const {
    switch (model_.type) {
        case ChassisType::Fixed:
            return FixedChassisVelocityCommand{};
        case ChassisType::Differential:
            return DifferentialCommand{twist.linear.x, twist.angular.z};
        case ChassisType::Ackermann:
            return AckermannCommand{twist.linear.x, twist.angular.z};
        case ChassisType::Omnidirectional:
            return OmnidirectionalCommand{twist.linear.x, twist.linear.y, twist.angular.z};
        case ChassisType::Mecanum:
            return MecanumCommand{twist.linear.x, twist.linear.y, twist.angular.z};
    }
    return FixedChassisVelocityCommand{};
}

CommandValidation validateCommandForModel(const ChassisModel& model,
                                          const ChassisVelocityCommand& command) {
    return ChassisVelocityCommandAdapter(model).validate(command);
}

geometry_msgs::msg::Twist toTwist(const ChassisVelocityCommand& command) {
    ChassisModel model;
    std::visit(
        [&](const auto& typed_command) {
            using CommandT = std::decay_t<decltype(typed_command)>;
            if constexpr (std::is_same_v<CommandT, DifferentialCommand>) {
                model.type = ChassisType::Differential;
            } else if constexpr (std::is_same_v<CommandT, AckermannCommand>) {
                model.type = ChassisType::Ackermann;
            } else if constexpr (std::is_same_v<CommandT, OmnidirectionalCommand>) {
                model.type = ChassisType::Omnidirectional;
            } else if constexpr (std::is_same_v<CommandT, MecanumCommand>) {
                model.type = ChassisType::Mecanum;
            }
        },
        command);
    return ChassisVelocityCommandAdapter(model).toTwist(command);
}

ChassisVelocityCommand velocityCommandFromTwist(const ChassisModel& model, const geometry_msgs::msg::Twist& twist) {
    return ChassisVelocityCommandAdapter(model).fromTwist(twist);
}

}  // namespace chassis_controller
