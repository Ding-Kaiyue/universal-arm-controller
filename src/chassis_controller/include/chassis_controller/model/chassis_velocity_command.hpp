#pragma once

#include <variant>

namespace chassis_controller {

struct FixedChassisVelocityCommand {};

struct DifferentialCommand {
    double vx{0.0};
    double omega{0.0};
};

struct AckermannCommand {
    double vx{0.0};
    double omega{0.0};
};

struct OmnidirectionalCommand {
    double vx{0.0};
    double vy{0.0};
    double omega{0.0};
};

struct MecanumCommand {
    double vx{0.0};
    double vy{0.0};
    double omega{0.0};
};

using ChassisVelocityCommand = std::variant<FixedChassisVelocityCommand, DifferentialCommand,
                                            AckermannCommand, OmnidirectionalCommand,
                                            MecanumCommand>;

struct ChassisVelocityState {
    double vx{0.0};
    double vy{0.0};
    double omega{0.0};
};

}  // namespace chassis_controller
