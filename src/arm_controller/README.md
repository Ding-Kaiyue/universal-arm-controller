# Arm Controller

[![ROS Version](https://img.shields.io/badge/ROS-ROS2%20Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()

**通用机械臂控制器** - Universal Arm Controller 的核心控制组件,提供多模式控制、状态管理和安全监控功能。

> 📖 **完整文档**: 查看 [docs/README.md](docs/README.md) 获取详细的文档中心,包括架构设计、控制器详解、开发指南等。

## 🚀 特性

- **多控制模式**: MoveJ、MoveL、MoveC、JointVelocity、CartesianVelocity 等多种控制模式
- **双臂支持**: 原生支持单臂和双臂配置,通过 mapping 机制灵活管理
- **实时安全**: 集成安全钩子状态(HoldState)和多层安全检查机制
- **MoveIt2 集成**: 深度集成 MoveIt2 进行碰撞检测和路径规划
- **模块化架构**: 双节点架构设计,ControllerManager 和 TrajectoryController 并行运行
- **ROS2 Action Server**: 完整的 FollowJointTrajectory 动作服务器支持

## 📦 安装

### 系统要求

- **操作系统**: Ubuntu 22.04 或更高版本
- **ROS 版本**: ROS2 Humble 或更高版本
- **编译器**: 支持 C++17 的 GCC/Clang
- **依赖项**: MoveIt2, yaml-cpp, Eigen3, rclcpp

### 快速安装

arm_controller 是 [Universal Arm Controller](https://github.com/Ding-Kaiyue/universal-arm-controller) 项目的核心组件:

```bash
# 1. 克隆主仓库
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller

# 2. 获取依赖组件
sudo apt install python3-vcstool
cd src
vcs import < ../deps.repos

# 3. 安装 ROS2 依赖
cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. 设置环境
source install/setup.bash
```

### 依赖组件

arm_controller 依赖以下外部组件:

#### 1. [trajectory_planning](https://github.com/Ding-Kaiyue/trajectory-planning)
- **功能**: 轨迹规划库
- **版本**: planning-only 分支
- **用途**: MoveIt2 集成、多种规划策略、碰撞检测

#### 2. [trajectory_interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator)
- **功能**: 轨迹插值库
- **版本**: master 分支
- **用途**: 样条插值算法、动力学约束满足

#### 3. [hardware_driver](https://github.com/Ding-Kaiyue/hardware-driver)
- **功能**: 硬件驱动库
- **版本**: master 分支
- **用途**: CAN-FD 通信、实时电机控制

## 🚀 快速开始

### 启动系统

```bash
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

### 控制器类型

系统支持三类控制器:

#### 1. 轨迹控制器 (Trajectory Controllers)
- **MoveJ**: 关节空间点对点运动
- **MoveL**: 笛卡尔空间直线运动
- **MoveC**: 圆弧/圆周轨迹运动

#### 2. 速度控制器 (Velocity Controllers)
- **JointVelocity**: 关节空间速度控制
- **CartesianVelocity**: 末端执行器速度控制(开发中)

#### 3. 实用控制器 (Utility Controllers)
- **HoldState**: 安全保持状态
- **Move2Start**: 移动到启动位置
- **Move2Initial**: 移动到初始位置
- **ROS2ActionControl**: MoveIt 轨迹执行
- **Disable**: 机械臂失能

### 基本使用

```bash
# 切换控制模式
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'MoveJ', mapping: 'single_arm'}"

# 发送关节目标
ros2 topic pub --once /controller_api/movej_action sensor_msgs/msg/JointState \
  "{position: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 发送笛卡尔目标
ros2 topic pub --once /controller_api/movel_action geometry_msgs/msg/Pose \
  "{position: {x: 0.19, y: 0.0, z: 0.63}, orientation: {x: -0.4546, y: 0.4546, z: -0.5417, w: 0.5417}}"
```

## 📋 配置

### 硬件配置 (`config/hardware_config.yaml`)

```yaml
hardware_interfaces:
  single_arm:
    robot_type: "arm620"
    interface: can0
    motors: [1, 2, 3, 4, 5, 6]
    joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    planning_group: "arm"
    initial_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    start_position: [0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0]
```

## 📚 文档

本README提供了快速开始指南。更多详细信息请访问:

- **[📖 文档中心](docs/README.md)** - 完整的项目文档门户
- **[🏗️ 系统架构](docs/ARCHITECTURE.md)** - 双节点架构设计
- **[🎮 控制器详解](docs/CONTROLLERS.md)** - 所有控制器的详细说明
- **[👨‍💻 开发者指南](docs/DEVELOPER.md)** - 深度开发指南
- **[🔒 安全机制](docs/SAFETY.md)** - 安全钩子和限位保护
- **[📋 代码规范](docs/CODE_STYLE.md)** - 代码风格指南

## 🛠️ 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- MoveIt 2
- CAN-FD 支持
- GCC 10+ (C++17)

## 🔍 故障排除

### CAN接口配置
```bash
# 配置CAN接口
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 \
  sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on
```

### 查看系统状态
```bash
# 查看控制模式
ros2 topic echo /controller_api/running_status

# 查看关节状态
ros2 topic echo /joint_states
```

更多问题排查请查看 [故障排除文档](docs/TROUBLESHOOTING.md)。

## 📄 许可证

待定

## 📞 联系方式

- **维护者**: Ding Kaiyue
- **邮箱**: kaiyue.ding@raysense.com
- **GitHub**: [universal-arm-controller](https://github.com/Ding-Kaiyue/universal-arm-controller)

## 🔗 相关项目

- **[trajectory-planning](https://github.com/Ding-Kaiyue/trajectory-planning)** - 轨迹规划库
- **[trajectory-interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator)** - 轨迹插值库
- **[hardware-driver](https://github.com/Ding-Kaiyue/hardware-driver)** - 硬件驱动库

---

⭐ **如果这个项目对你有帮助,请给我们一个星标!**
