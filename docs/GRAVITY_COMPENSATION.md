# 重力补偿功能实现文档

## 概述

本文档描述了使用 C++ Pinocchio 库实现的重力力矩补偿功能。该功能用于在 MIT 模式控制下补偿机械臂各关节的重力力矩，使机械臂能够在任意姿态下保持稳定。

## 依赖

- **Pinocchio**: 机器人动力学计算库
- **URDF**: 机械臂模型文件

## 实现架构

```
┌─────────────────────────────────────────────────────────────────┐
│                      HardwareManager                             │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  GravityCompensator (独立类)                             │    │
│  │  - loadModel(): 加载 URDF 到 Pinocchio                   │    │
│  │  - computeGravityTorques(): 计算重力力矩                 │    │
│  │  - 支持多个 mapping 的模型管理                           │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────────┐
│JointVelocity  │   │CartesianVel   │   │TrajectoryRecord   │
│Controller     │   │Controller     │   │Controller         │
│               │   │               │   │                   │
│实时计算重力矩 │   │实时计算重力矩 │   │定时器计算重力矩   │
│发送MIT命令    │   │发送MIT命令    │   │(100Hz)            │
└───────────────┘   └───────────────┘   └───────────────────┘
                              │
                              ▼
                    ┌───────────────────┐
                    │  TrajectoryExec   │
                    │  (轨迹执行)        │
                    │                   │
                    │  预计算每个点的   │
                    │  重力力矩存入     │
                    │  TrajectoryPoint  │
                    │  .efforts 字段    │
                    └───────────────────┘
```

## 修改的文件

### 1. GravityCompensator 类 (新增)
**文件路径**:
- `src/arm_controller/include/arm_controller/dynamics/gravity_compensator.hpp`
- `src/arm_controller/src/dynamics/gravity_compensator.cpp`

**职责**: 封装 Pinocchio 库，负责重力力矩计算

**接口**:
```cpp
class GravityCompensator {
public:
    // 加载 URDF 模型
    bool loadModel(const std::string& mapping, const std::string& urdf_path);

    // 检查模型是否已加载
    bool hasModel(const std::string& mapping) const;

    // 计算重力力矩
    std::vector<double> computeGravityTorques(const std::string& mapping,
                                              const std::vector<double>& joint_positions);

    // 获取关节数量
    size_t getNumJoints(const std::string& mapping) const;
};
```

### 2. CMakeLists.txt
**文件路径**: `src/arm_controller/CMakeLists.txt`

**修改内容**:
- 添加 Pinocchio 依赖
- 添加 gravity_compensator.cpp 到编译源文件
```cmake
find_package(pinocchio REQUIRED)

add_library(arm_controller_lib
  ...
  src/dynamics/gravity_compensator.cpp
  ...
)

target_link_libraries(arm_controller_lib
  ...
  pinocchio::pinocchio
)
```

### 3. HardwareManager 头文件
**文件路径**: `src/arm_controller/include/arm_controller/hardware/hardware_manager.hpp`

**修改内容**: 使用 GravityCompensator 替代直接使用 Pinocchio
```cpp
#include "arm_controller/dynamics/gravity_compensator.hpp"

// 成员变量
std::shared_ptr<arm_controller::dynamics::GravityCompensator> gravity_compensator_;

// 接口保持不变
std::vector<double> compute_gravity_torques(const std::string& mapping);
std::vector<double> compute_gravity_torques(const std::string& mapping,
                                            const std::vector<double>& joint_positions);
```

### 4. HardwareManager 实现
**文件路径**: `src/arm_controller/src/hardware/hardware_manager.cpp`

**修改内容**: 通过 GravityCompensator 计算重力矩
```cpp
// parse_mapping() 中加载模型
if (!gravity_compensator_) {
    gravity_compensator_ = std::make_shared<arm_controller::dynamics::GravityCompensator>();
}
gravity_compensator_->loadModel(mapping_name, urdf_path);

// compute_gravity_torques() 调用 GravityCompensator
std::vector<double> HardwareManager::compute_gravity_torques(
    const std::string& mapping,
    const std::vector<double>& joint_positions) {

    if (!gravity_compensator_ || !gravity_compensator_->hasModel(mapping)) {
        return std::vector<double>(joint_positions.size(), 0.0);
    }
    return gravity_compensator_->computeGravityTorques(mapping, joint_positions);
}
```

**executeTrajectory() 和 execute_trajectory_async() 中预计算重力力矩**:
```cpp
// 预计算每个轨迹点的重力力矩
Trajectory trajectory_with_efforts = trajectory;
for (auto& point : trajectory_with_efforts.points) {
    std::vector<double> positions_rad;
    for (double pos_deg : point.positions) {
        positions_rad.push_back(pos_deg * M_PI / 180.0);
    }
    point.efforts = compute_gravity_torques(mapping, positions_rad);
}
```

### 4. JointVelocityController
**文件路径**: `src/arm_controller/src/controller/joint_velocity/joint_velocity_controller.cpp`

**修改内容**: 在发送 MIT 命令前计算重力力矩
```cpp
// 获取重力补偿力矩
std::vector<double> gravity_torques = hardware_manager_->compute_gravity_torques(mapping);

// 发送带重力补偿的 MIT 命令
for (size_t i = 0; i < motor_ids.size(); ++i) {
    double effort = (i < gravity_torques.size()) ? gravity_torques[i] : 0.0;
    robot_hardware->control_motor_in_mit_mode(interface, motor_ids[i],
                                               position, velocity, effort, kp, kd);
}
```

### 5. CartesianVelocityController
**文件路径**: `src/arm_controller/src/controller/cartesian_velocity/cartesian_velocity_controller.cpp`

**修改内容**: 与 JointVelocityController 类似，在发送 MIT 命令前计算重力力矩。

### 6. TrajectoryRecordController
**文件路径**: `src/arm_controller/src/controller/trajectory_record/trajectory_record_controller.cpp`

**修改内容**: 使用定时器 (100Hz) 定期计算并发送重力补偿力矩
```cpp
// 创建重力补偿定时器
gravity_compensation_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&TrajectoryRecordController::gravity_compensation_timer_callback, this)
);

void TrajectoryRecordController::gravity_compensation_timer_callback() {
    if (!is_recording_) return;

    std::vector<double> gravity_torques = hardware_manager_->compute_gravity_torques(mapping);
    // 发送重力补偿力矩...
}
```

### 7. TrajectoryPoint 结构体
**文件路径**:
- `src/trajectory_interpolator/include/trajectory_interpolator/moveit_spline_adapter.hpp`
- `src/hardware_driver/include/hardware_driver/interface/robot_hardware.hpp`

**修改内容**: 添加 efforts 字段
```cpp
struct TrajectoryPoint {
    double time_from_start;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    std::vector<double> efforts;  // 重力补偿力矩 (可选，用于MIT模式控制)
};
```

### 8. RobotHardware 轨迹执行
**文件路径**: `src/hardware_driver/src/interface/robot_hardware.cpp`

**修改内容**: 在轨迹执行时使用预计算的 efforts
```cpp
// 使用预计算的重力补偿力矩
float effort = (i < point.efforts.size()) ? static_cast<float>(point.efforts[i]) : 0.0f;
efforts[i] = effort;
```

## URDF 配置

URDF 文件路径: `src/trajectory_planning/robot_description/urdf/arm620.urdf`

在 mapping 配置中指定 URDF 路径，HardwareManager 会自动加载 Pinocchio 模型。

## 使用场景

### 1. 速度控制模式 (JointVelocity / CartesianVelocity)
- 实时计算当前关节位置的重力力矩
- 每次发送 MIT 命令时附带重力补偿

### 2. 示教模式 (TrajectoryRecord)
- 100Hz 定时器持续计算重力力矩
- 保持机械臂在任意姿态下的稳定

### 3. 轨迹执行 (TrajectoryExecution)
- 轨迹执行前预计算所有轨迹点的重力力矩
- 存储在 TrajectoryPoint.efforts 字段中
- 执行时直接使用预计算值，减少实时计算开销

## 测试方法

### 第一步：启动系统

```bash
# 启动完整的机械臂控制系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

### 第二步：测试 JointVelocity 模式的重力补偿

在另一个终端中：

```bash
# 切换到 JointVelocity 模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'JointVelocity', mapping: 'single_arm'}"

# 发送零速度命令（仅靠重力补偿维持位置）
ros2 topic pub /controller_api/joint_velocity_action/single_arm sensor_msgs/msg/JointState \
  "{velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**验证方法**: 发送零速度后，观察二轴是否能在当前位置保持住而不下坠。如果重力补偿生效，手臂应该能"悬停"。

**动态测试（二轴起升测试）**: 先将手臂移动到二轴接近最低点的位置，然后：

```bash
# 给二轴一个小的正速度让它起来
ros2 topic pub --once /controller_api/joint_velocity_action/single_arm sensor_msgs/msg/JointState \
  "{velocity: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0]}"

# 然后发送零速度，观察是否能停住
ros2 topic pub /controller_api/joint_velocity_action/single_arm sensor_msgs/msg/JointState \
  "{velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 第三步：测试 CartesianVelocity 模式的重力补偿

```bash
# 切换到 CartesianVelocity 模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'CartesianVelocity', mapping: 'single_arm'}"

# 发送零笛卡尔速度命令
ros2 topic pub /controller_api/cartesian_velocity_action/single_arm geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

**验证方法**: 同样观察手臂是否能在零速度命令下保持当前位置不下坠。

### 第四步：测试 MoveIt 轨迹执行的重力补偿

```bash
# 切换到 MoveJ 模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'MoveJ', mapping: 'single_arm'}"

# 将二轴移动到最低点位置（根据实际关节限位调整）
ros2 topic pub /controller_api/movej_action/single_arm sensor_msgs/msg/JointState \
  "{position: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]}"
```

**验证方法**: 当轨迹执行完成后，观察二轴在最低点位置是否会往下掉。如果重力补偿生效，手臂应该能稳定保持在目标位置。

### 预期结果

| 测试项 | 有重力补偿 | 无重力补偿 |
|--------|-----------|-----------|
| 零速度悬停 | 手臂稳定保持位置 | 手臂缓慢下坠 |
| 二轴起升 | 平稳起升，能用较小速度克服重力 | 需要更大速度才能起升 |
| 停止命令 | 立即停止并保持位置 | 因惯性和重力继续下坠 |
| MoveIt 到位 | 稳定保持目标位置 | 到位后下坠 |

## 注意事项

1. **单位转换**: URDF 使用弧度，部分接口使用角度，需注意转换
2. **线程安全**: pinocchio_mutex_ 保护模型和数据的并发访问
3. **模型加载**: 确保 URDF 路径正确，模型加载失败会返回空力矩向量
4. **性能**: RNEA 算法计算复杂度为 O(n)，适合实时控制
