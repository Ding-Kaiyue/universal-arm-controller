# 配置指南

本文档详细说明 Arm Controller 的所有配置选项。

## 📋 目录

- [配置文件结构](#配置文件结构)
- [map和配置信息绑定](#map和配置信息绑定)
- [控制器配置](#控制器配置)
- [安全配置](#安全配置)
- [插值配置](#插值配置)
- [ROS2配置](#ros2配置)
- [调试配置](#调试配置)

---

## 配置文件结构

```
config/
├── hardware_config.yaml      # map和配置信息绑定
├── config.yaml               # 通用系统配置
├── arm620_joint_limits.yaml  # 某型号机器人关节限制
├── arm380_joint_limits.yaml  # 某型号机器人关节限制
└── interpolator_config.yaml  # 插值配置
```

---

## map和配置信息绑定

### hardware_config.yaml

#### 单臂配置

```yaml
hardware_interfaces:
  single_arm:
    # 机器人型号
    robot_type: "arm620"              # arm380 / arm620 / custom

    # CAN接口
    interface: "can0"                 # CAN接口名称

    # 电机配置
    motors: [1, 2, 3, 4, 5, 6]        # 电机ID列表

    # 关节命名
    joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    # 坐标系
    frame_id: "base_link"             # 基座坐标系

    # 控制器名称
    controller_name: "arm_controller"

    # MoveIt规划组
    planning_group: "arm"

    # 初始位置 (弧度)
    initial_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # 启动位置 (弧度)
    start_position: [0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0]
```

#### 双臂配置

```yaml
hardware_interfaces:
  left_arm:
    robot_type: "arm620"
    interface: "can0"
    motors: [1, 2, 3, 4, 5, 6]
    joint_names: ["left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6"]
    frame_id: "left_base_link"
    controller_name: "left_arm_controller"
    planning_group: "left_arm"
    initial_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    start_position: [0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0]

  right_arm:
    robot_type: "arm620"
    interface: "can1"
    motors: [1, 2, 3, 4, 5, 6]
    joint_names: ["right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6"]
    frame_id: "right_base_link"
    controller_name: "right_arm_controller"
    planning_group: "right_arm"
    initial_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    start_position: [0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0]
```

#### 参数说明

| 参数 | 类型 | 说明 | 默认值 |
|-----|------|-----|--------|
| `robot_type` | string | 机器人型号 | arm620 |
| `interface` | string | CAN接口名称 | can0 |
| `motors` | int[] | 电机ID列表 | [1,2,3,4,5,6] |
| `joint_names` | string[] | 关节名称列表 | 必须提供 |
| `frame_id` | string | TF坐标系名称 | base_link |
| `controller_name` | string | 控制器名称 | arm_controller |
| `planning_group` | string | MoveIt规划组 | arm |
| `initial_position` | double[] | 初始位置(弧度) | 全零 |
| `start_position` | double[] | 启动位置(弧度) | 必须提供 |

---

## 控制器配置

### config.yaml

#### 通用接口配置(common)

```yaml
common:
  - key: controller_mode_service
    name: /controller_api/controller_mode
    type: controller_interfaces/srv/WorkMode
    kind: service

  - key: running_status
    name: /controller_api/running_status
    type: std_msgs/msg/String
    kind: input_topic
```

| 参数 | 类型 | 说明 | 示例 |
|-----|------|-----|------|
| `key` | string | 唯一键名，用于在系统内引用该接口 | controller_mode_service |
| `name` | string | ROS 话题或服务名 | /controller_api/controller_mode |
| `type` | string | ROS 消息或服务类型 | controller_interfaces/srv/WorkMode |
| `kind` | string | 接口类别（service / input_topic / output_topic） | service |

#### 控制器配置(controllers)

```yaml
controllers:
  - key: Disable
    class: DisableController

  - key: HoldState
    class: HoldStateController

  - key: JointVelocity
    class: JointVelocityController
    input_topic:
      name: /controller_api/joint_velocity_action
      type: sensor_msgs/msg/JointState

  - key: Move2Initial
    class: Move2InitialController

  - key: Move2Start
    class: Move2StartController

  - key: ROS2ActionControl
    class: ROS2ActionControlController

  - key: MoveC
    class: MoveCController
    input_topic: 
      name: /controller_api/movec_action
      type: geometry_msgs/msg/PoseArray

  - key: MoveJ
    class: MoveJController
    input_topic: 
      name: /controller_api/movej_action
      type: sensor_msgs/msg/JointState

  - key: MoveL
    class: MoveLController
    input_topic: 
      name: /controller_api/movel_action
      type: geometry_msgs/msg/Pose
```
| 参数 | 类型 | 说明 | 示例 | 
| --- | --- | --- | --- |
| key | string | 控制器唯一标识符 | MoveJ |
| class | string | 控制器类名(需在注册宏中定义) | MoveJController |
| input_topic.name | string | 控制器订阅的ROS话题名称 | /controller_api/movej_action |
| input_topic.type | string | 控制器订阅的ROS话题类型 | sensor_msgs/msg/JointState |

#### 已注册的控制器一览
| 控制器Key | 控制器类名 | 输入话题 | 消息类型 | 说明 |
| --- | --- | --- | --- | --- |
| Disable | DisableController | - | - | 整机失能 |
| HoldState | HoldStateController | - | - | 保持当前姿态 |
| JointVelocity | JointVelocityController | /controller_api/joint_velocity_action | sensor_msgs/msg/JointState | 关节速度控制 |
| Move2Initial | Move2InitialController | - | - | 移动到初始位置 |
| Move2Start | Move2StartController | - | - | 移动到启动位置
| ROS2ActionControl | ROS2ActionController | - | - | ROS2Action控制层 |
| MoveC | MoveCController | /controller_api/movec_action | geometry_msgs/msg/PoseArray | 圆弧运动控制 |
| MoveJ | MoveJController | /controller_api/movej_action | sensor_msgs/msg/JointState | 关节空间运动控制 |
| MoveL | MoveLController | /controller_api/movel_action | geometry_msgs/msg/Pose | 笛卡尔空间直线运动控制(支持智能回退) |
---

#### 扩展控制器配置示例
```yaml
- key: TrajectoryReplay
  class: TrajectoryReplayController
  input_topic:
    name: /controller_api/trajectory_replay_action
    type: std_msgs/msg/String
```

## 插值配置

```yaml
interpolation:
  default:
    # 时间步长
    target_dt: 0.01           # 插值时间步长 (秒)
    max_velocity: 180.0       # 最大速度 (度/秒)
    max_acceleration: 360.0   # 最大加速度 (度/秒^2)
    max_jerk: 720.0           # 最大加加速度 (度/秒^3)
```

此处仅列出常用配置项, 其他参数使用默认. 更多配置项请参考 [interpolation](https://github.com/Ding-Kaiyue/trajectory-interpolator)仓库的使用说明.

---

## 配置验证

### 验证工具

```bash
# 检查YAML语法
yamllint config/hardware_config.yaml

# 验证配置有效性
ros2 run arm_controller validate_config --config config/hardware_config.yaml
```

### 常见配置错误

#### 1. 关节数量不匹配

```yaml
# ❌ 错误: motors有6个,joint_names只有5个
motors: [1, 2, 3, 4, 5, 6]
joint_names: ["j1", "j2", "j3", "j4", "j5"]

# ✅ 正确: 数量一致
motors: [1, 2, 3, 4, 5, 6]
joint_names: ["j1", "j2", "j3", "j4", "j5", "j6"]
```

#### 2. 限位配置错误

```yaml
# ❌ 错误: upper < lower
position_limits:
  lower: [0.0, -2.0, -2.0, 0.0, -2.0, 0.0]
  upper: [-3.0, 2.0, 2.0, 3.0, 2.0, 3.0]  # upper[0] < lower[0]

# ✅ 正确
position_limits:
  lower: [-3.0, -2.0, -2.0, -3.0, -2.0, -3.0]
  upper: [3.0, 2.0, 2.0, 3.0, 2.0, 3.0]
```

#### 3. 规划组名称不匹配

```yaml
# ❌ 错误: planning_group与SRDF不匹配
planning_group: "manipulator"  # SRDF中定义的是"arm"

# ✅ 正确: 与SRDF一致
planning_group: "arm"
```

### 自定义机器人模板

```yaml
# config/custom_robot_config.yaml
hardware_interfaces:
  my_robot:
    robot_type: "custom"
    interface: "can0"
    motors: [1, 2, 3, 4, 5, 6, 7]  # 7自由度
    joint_names: ["joint1", ..., "joint7"]
    # ... 其他参数
```

---

## 相关文档

- [快速开始](QUICKSTART.md) - 基本配置示例
- [硬件驱动](https://github.com/Ding-Kaiyue/hardware-driver) - CAN配置
- [轨迹规划](https://github.com/Ding-Kaiyue/trajectory-planning) - MoveIt配置
