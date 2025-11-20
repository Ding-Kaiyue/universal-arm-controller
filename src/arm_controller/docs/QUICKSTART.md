# 快速开始

5分钟上手 Arm Controller!

## 前提条件

- ✅ 已完成安装(参考 [README.md](../README.md#安装))
- ✅ CAN 接口已配置
- ✅ 机械臂硬件已连接

## 第一步:配置 CAN 接口

```bash
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on loopback off restart-ms 100
```

## 第二步:启动系统

```bash
# source 工作空间
source ~/robotic_arm_ws/install/setup.bash

# 启动控制系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

## 第三步:测试基本功能

### 1. 移动到启动位置

```bash
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'Move2Start', mapping: 'single_arm'}"
```

### 2. MoveJ 测试

```bash
# 切换到 MoveJ 模式
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'MoveJ', mapping: 'single_arm'}"

# 发送目标位置
ros2 topic pub --once /controller_api/movej_action/single_arm sensor_msgs/msg/JointState \
  "{position: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 3. 轨迹控制测试

在轨迹执行过程中，可以实时暂停、恢复或取消:

```bash
# 暂停轨迹
ros2 topic pub --once /trajectory_control controller_interfaces/msg/TrajectoryControl \
  "{mapping: 'single_arm', action: 'Pause'}"

# 恢复轨迹
ros2 topic pub --once /trajectory_control controller_interfaces/msg/TrajectoryControl \
  "{mapping: 'single_arm', action: 'Resume'}"

# 取消轨迹
ros2 topic pub --once /trajectory_control controller_interfaces/msg/TrajectoryControl \
  "{mapping: 'single_arm', action: 'Cancel'}"
```

### 4. MoveL 测试

```bash
# 切换到 MoveL 模式
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'MoveL', mapping: 'single_arm'}"

# 发送目标位姿
ros2 topic pub --once /controller_api/movel_action/single_arm geometry_msgs/msg/Pose \
  "{position: {x: 0.19, y: 0.0, z: 0.63}, orientation: {x: -0.4546, y: 0.4546, z: -0.5417, w: 0.5417}}"
```

## 第四步:监控状态

```bash
# 查看当前模式
ros2 topic echo /controller_api/running_status

# 查看关节状态
ros2 topic echo /joint_states
```

## 下一步

- 📖 阅读 [控制器详解](CONTROLLERS.md) 了解所有控制模式
- ⚙️ 查看 [配置指南](CONFIGURATION.md) 自定义配置
- 🏗️ 学习 [系统架构](ARCHITECTURE.md) 深入理解系统

## 常见问题

**Q: 机械臂不动?**
A: 检查 CAN 接口是否配置,电机是否上电。

**Q: 规划失败?**
A: 检查目标位置是否在工作空间内,查看 MoveIt 日志。

**Q: 模式切换卡住?**
A: 等待 HoldState 完成安全检查,通常需要几秒钟。