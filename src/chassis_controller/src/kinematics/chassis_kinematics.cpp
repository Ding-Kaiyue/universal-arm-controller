#include "chassis_controller/kinematics/chassis_kinematics.hpp"

#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace chassis_controller {
namespace {

constexpr double kMinWheelRadius = 1e-6;

double requireWheelRadius(const double wheel_radius) {
    if (wheel_radius <= kMinWheelRadius) {
        throw std::invalid_argument("wheel_radius must be positive");
    }
    return wheel_radius;
}

double square(const double value) {
    return value * value;
}

double norm(const double x, const double y) {
    return std::sqrt(square(x) + square(y));
}

}  // namespace

ChassisKinematics::ChassisKinematics(ChassisKinematicsConfig config)
    : config_(std::move(config)) {}

ChassisJointCommandSet ChassisKinematics::computeJointCommands(
    const ChassisVelocityCommand& command) const {
    const ChassisVelocityState velocity = toVelocityState(command);

    switch (config_.model.type) {
        case ChassisType::Fixed:
            return {};
        case ChassisType::Differential:
            return computeDifferentialCommands(velocity);
        case ChassisType::Ackermann:
            return computeSteerableModuleCommands(velocity);
        case ChassisType::Omnidirectional:
            return computeOmnidirectionalCommands(velocity);
        case ChassisType::Mecanum:
            return computeMecanumCommands(velocity);
    }

    return {};
}

ChassisVelocityState ChassisKinematics::toVelocityState(const ChassisVelocityCommand& command) const {
    ChassisVelocityState velocity;

    std::visit(
        [&](const auto& typed_command) {
            using CommandT = std::decay_t<decltype(typed_command)>;
            if constexpr (std::is_same_v<CommandT, FixedChassisVelocityCommand>) {
                return;
            } else if constexpr (std::is_same_v<CommandT, DifferentialCommand> ||
                                 std::is_same_v<CommandT, AckermannCommand>) {
                velocity.vx = typed_command.vx;
                velocity.omega = typed_command.omega;
            } else {
                velocity.vx = typed_command.vx;
                velocity.vy = typed_command.vy;
                velocity.omega = typed_command.omega;
            }
        },
        command);

    return velocity;
}

ChassisJointCommandSet ChassisKinematics::computeDifferentialCommands(
    const ChassisVelocityState& velocity) const {
    ChassisJointCommandSet output;
    const double wheel_radius = requireWheelRadius(config_.wheel_radius);
    const double half_track = 0.5 * config_.track_width;

    for (const ChassisModuleGeometry& module : config_.modules) {
        if (module.wheel_joint.empty()) {
            continue;
        }

        const double wheel_linear_velocity = velocity.vx - velocity.omega * module.y;
        appendWheelVelocity(&output, module, wheel_linear_velocity / wheel_radius);
    }

    if (output.empty() && config_.modules.size() >= 2 && half_track > 0.0) {
        const ChassisModuleGeometry& left_module = config_.modules[0];
        const ChassisModuleGeometry& right_module = config_.modules[1];
        appendWheelVelocity(&output, left_module,
                            (velocity.vx - velocity.omega * half_track) / wheel_radius);
        appendWheelVelocity(&output, right_module,
                            (velocity.vx + velocity.omega * half_track) / wheel_radius);
    }

    return output;
}

ChassisJointCommandSet ChassisKinematics::computeOmnidirectionalCommands(
    const ChassisVelocityState& velocity) const {
    ChassisJointCommandSet output;
    const double wheel_radius = requireWheelRadius(config_.wheel_radius);

    for (const ChassisModuleGeometry& module : config_.modules) {
        if (module.wheel_joint.empty()) {
            continue;
        }

        const double direction_norm = norm(module.drive_direction_x, module.drive_direction_y);
        if (direction_norm <= 1e-6) {
            throw std::invalid_argument("omnidirectional wheel drive direction must be non-zero");
        }

        const double drive_x = module.drive_direction_x / direction_norm;
        const double drive_y = module.drive_direction_y / direction_norm;
        const double module_vx = velocity.vx - velocity.omega * module.y;
        const double module_vy = velocity.vy + velocity.omega * module.x;
        const double wheel_velocity = (module_vx * drive_x + module_vy * drive_y) / wheel_radius;
        appendWheelVelocity(&output, module, wheel_velocity);
    }

    return output;
}

ChassisJointCommandSet ChassisKinematics::computeMecanumCommands(
    const ChassisVelocityState& velocity) const {
    ChassisJointCommandSet output;
    const double wheel_radius = requireWheelRadius(config_.wheel_radius);
    const double geometry_scale =
        config_.wheel_base > 0.0 || config_.track_width > 0.0
            ? 0.5 * (config_.wheel_base + config_.track_width)
            : 1.0;

    for (const ChassisModuleGeometry& module : config_.modules) {
        if (module.wheel_joint.empty()) {
            continue;
        }

        const double lateral_sign = module.y >= 0.0 ? -1.0 : 1.0;
        const double yaw_sign = module.x * module.y >= 0.0 ? -1.0 : 1.0;
        const double wheel_velocity =
            (velocity.vx + lateral_sign * velocity.vy +
             yaw_sign * geometry_scale * velocity.omega) /
            wheel_radius;
        appendWheelVelocity(&output, module, wheel_velocity);
    }

    return output;
}

ChassisJointCommandSet ChassisKinematics::computeSteerableModuleCommands(
    const ChassisVelocityState& velocity) const {
    ChassisJointCommandSet output;
    const double wheel_radius = requireWheelRadius(config_.wheel_radius);

    for (const ChassisModuleGeometry& module : config_.modules) {
        const double module_vx = velocity.vx - velocity.omega * module.y;
        const double module_vy = velocity.vy + velocity.omega * module.x;
        const double steering_position = std::atan2(module_vy, module_vx);
        const double wheel_velocity =
            std::sqrt(square(module_vx) + square(module_vy)) / wheel_radius;

        appendSteeringPosition(&output, module, steering_position);
        appendWheelVelocity(&output, module, wheel_velocity);
    }

    return output;
}

void ChassisKinematics::appendWheelVelocity(ChassisJointCommandSet* output,
                                            const ChassisModuleGeometry& module,
                                            const double wheel_velocity) const {
    if (module.wheel_joint.empty()) {
        return;
    }

    output->joints.push_back(
        ChassisJointCommand{module.wheel_joint, false, true, 0.0,
                            module.wheel_axis_sign * wheel_velocity});
}

void ChassisKinematics::appendSteeringPosition(ChassisJointCommandSet* output,
                                               const ChassisModuleGeometry& module,
                                               const double steering_position) const {
    if (module.steering_joint.empty()) {
        return;
    }

    output->joints.push_back(
        ChassisJointCommand{module.steering_joint, true, false,
                            module.steering_axis_sign * steering_position, 0.0});
}

}  // namespace chassis_controller
