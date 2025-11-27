#ifndef ROBOT_SDK_COMMANDS_HPP
#define ROBOT_SDK_COMMANDS_HPP

#include <cstdint>

namespace robot_sdk {

// SDK命令枚举
enum SdkCommand : uint8_t {
    SDK_BACK_TO_START = 0,      // 回到初始位置
    SDK_DISABLE = 1,            // 失能电机
    SDK_JOINT_CONTROL = 2,      // 单关节控制
    SDK_CARTESIAN = 3,          // 笛卡尔控制
    SDK_MOVEJ = 4,              // 关节空间运动
    SDK_MOVEL = 5,              // 笛卡尔空间运动
    SDK_MOVEC = 6,              // 圆弧运动
    SDK_TEACH_START = 7,        // 开始示教录制
    SDK_TEACH_REPEAT = 8,       // 开始轨迹复现
    SDK_STATE_OPERATION = 9,    // 状态操作
    SDK_LOAD_STATE = 10,        // 加载状态
    SDK_BACK_TO_INITIAL = 11,   // 回到初始关节位置
    SDK_MOTOR_ZERO_SET = 12,    // 电机零位设置
    SDK_GRIPPER_CONTROL = 13,   // 夹爪控制
    SDK_USER_PARAM_SET = 14,    // 用户参数设置
    SDK_USER_PARAM_GET = 15,    // 用户参数获取
    SDK_TEACH_STOP = 16,        // 停止示教录制
    SDK_GET_POSE = 17,          // 获取当前位姿
    SDK_JOINT_STOP = 18,        // 停止关节运动
    SDK_SPEED_CONFIG = 19,      // 速度配置
    SDK_HOLD_STATE = 20,        // 保持当前状态
};

// 控制器模式名称映射
inline const char* get_mode_name(uint8_t command) {
    switch (command) {
        case SDK_DISABLE:         return "Disable";
        case SDK_TEACH_START:     return "TrajectoryRecord";
        case SDK_TEACH_REPEAT:    return "TrajectoryReplay";
        case SDK_MOVEJ:           return "MoveJ";
        case SDK_MOVEL:           return "MoveL";
        case SDK_MOVEC:           return "MoveC";
        case SDK_JOINT_CONTROL:   return "JointVelocity";
        case SDK_CARTESIAN:       return "CartesianVelocity";
        case SDK_HOLD_STATE:      return "HoldState";
        case SDK_BACK_TO_START:   return "Move2Start";
        case SDK_BACK_TO_INITIAL: return "Move2Initial";
        default:                  return nullptr;
    }
}

}  // namespace robot_sdk

#endif  // ROBOT_SDK_COMMANDS_HPP
