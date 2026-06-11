#pragma once

#include <string>
#include <vector>

#include "chassis_controller/model/chassis_velocity_command.hpp"
#include "chassis_controller/model/chassis_joint_command.hpp"
#include "chassis_controller/model/chassis_model.hpp"

namespace chassis_controller {

struct ChassisModuleGeometry {
    std::string name;
    std::string steering_joint;
    std::string wheel_joint;
    double x{0.0};
    double y{0.0};
    double drive_direction_x{1.0};
    double drive_direction_y{0.0};
    double steering_axis_sign{1.0};
    double wheel_axis_sign{1.0};
};

struct ChassisKinematicsConfig {
    ChassisModel model;
    double wheel_radius{0.05};
    double track_width{0.0};
    double wheel_base{0.0};
    std::vector<ChassisModuleGeometry> modules;
};

class ChassisKinematics {
public:
    explicit ChassisKinematics(ChassisKinematicsConfig config);

    ChassisJointCommandSet computeJointCommands(const ChassisVelocityCommand& command) const;

private:
    ChassisVelocityState toVelocityState(const ChassisVelocityCommand& command) const;
    ChassisJointCommandSet computeDifferentialCommands(const ChassisVelocityState& velocity) const;
    ChassisJointCommandSet computeOmnidirectionalCommands(
        const ChassisVelocityState& velocity) const;
    ChassisJointCommandSet computeMecanumCommands(const ChassisVelocityState& velocity) const;
    ChassisJointCommandSet computeSteerableModuleCommands(
        const ChassisVelocityState& velocity) const;
    void appendWheelVelocity(ChassisJointCommandSet* output, const ChassisModuleGeometry& module,
                             double wheel_velocity) const;
    void appendSteeringPosition(ChassisJointCommandSet* output,
                                const ChassisModuleGeometry& module,
                                double steering_position) const;

    ChassisKinematicsConfig config_;
};

}  // namespace chassis_controller
