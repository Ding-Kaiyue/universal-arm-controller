#ifndef __MOTOR_MODE_HPP__
#define __MOTOR_MODE_HPP__

#include <cstdint>

enum class MotorControlMode : uint8_t {
    EFFORT_MODE         = 0x02,
    MIT_MODE             = 0x03,
    SPEED_MODE           = 0x04,
    POSITION_ABS_MODE    = 0x05,
    POSITION_INC_MODE    = 0x06
};

#endif // __MOTOR_MODE_HPP__