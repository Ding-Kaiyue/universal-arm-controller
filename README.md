# Universal Arm Controller

[![ROS Version](https://img.shields.io/badge/ROS-ROS2%20Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/Ding-Kaiyue/universal-arm-controller/actions/workflows/colcon-build.yml/badge.svg?branch=master)](https://github.com/Ding-Kaiyue/universal-arm-controller/actions/workflows/colcon-build.yml)

完整的机械臂控制系统解决方案。基于 ROS2 的模块化架构，集成轨迹规划、轨迹插值、硬件驱动等核心功能，为工业机械臂提供高效、可靠的运动控制。

## 🚀 核心特性

### 运动控制

- **13+ 控制模式**: MoveJ、MoveL、MoveC、JointVelocity、CartesianVelocity、PointRecord、PointReplay、TrajectoryRecord、TrajectoryReplay 等
- **全 6D 方向控制**: 完整的末端执行器位姿控制，支持任意方向
- **双臂原生支持**: 原生单臂/双臂协同控制，支持对称运动
- **动态速度缩放**: MoveJ/MoveL/MoveC 动态速度调整，无需重新规划

### 性能与可靠性

- **微秒级控制延迟**: <200μs 实时控制响应
- **高速硬件通信**: CAN-FD 5000 kbit/s，事件驱动监控
- **工业级安全**: 多层安全检查、关节限位保护、紧急停止机制
- **线程安全**: CPU 亲和性优化，多线程高效执行
- **实时性保证**: 500Hz 高频更新，确保运动平滑

### 架构设计

- **模块化设计**: 清晰三层架构，组件独立开发维护
- **插件式控制器**: 易于扩展新的控制模式
- **事件驱动**: 观察者模式，硬件状态实时通知
- **安全状态机**: 多层状态转换钩子，确保系统安全

## 📦 安装

### 系统要求

- **OS**: Ubuntu 22.04 LTS+
- **ROS**: ROS2 Humble+
- **编译器**: GCC 10+ (C++17)
- **工具**: colcon, vcstool

### 快速安装

```bash
# 1. 创建工作空间
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src

# 2. 克隆仓库与依赖
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller/src
sudo apt install python3-vcstool
vcs import < ../deps.repos --recursive

# 3. 编译
cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 前置说明

详见 [文档中心](docs/README.md) 中的配置与故障排除部分。

## 🚀 快速开始

### 启动系统

```bash
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

### 控制示例

```bash
# 切换控制模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'MoveJ'}"

# 发送关节空间目标
ros2 topic pub /controller_api/movej_action/single_arm sensor_msgs/msg/JointState \
  "{position: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## 📚 文档中心

访问 **[文档中心](docs/README.md)** 了解详细信息。

## 📦 核心组件

### 本仓库维护的组件

| 组件 | 功能 | 描述 |
|------|------|------|
| **arm_controller** | 运动控制核心 | 13+ 控制模式、双节点架构、状态管理、安全机制 |
| **controller_interfaces** | ROS2 消息/服务定义 | 工作模式切换、轨迹控制、系统状态消息 |
| **robotic_arm_bringup** | 系统启动配置 | ROS2 启动文件、YAML 配置、参数管理 |

### VCS 导入的依赖组件

| 组件 | 功能 | 特性 |
|------|------|------|
| **hardware_driver** | CAN-FD 硬件驱动 | 5000 kbit/s 高速通信、实时电机控制、事件驱动监控 |
| **trajectory_interpolator** | 样条轨迹插值 | 3次样条曲线、动力学约束、实时轨迹生成 |
| **trajectory_planning** | MoveIt2 规划集成 | TracIK 逆运动学、碰撞检测、多种规划策略 |
| **csaps** | C++ 样条曲线库 | 自适应样条拟合、轨迹平滑、录制数据后处理 |

### 支持的机器人配置

| 配置 | 关节数 | 用途 |
|------|--------|------|
| **ARM380** | 6 轴 | 工业机械臂 |
| **ARM620** | 6 轴 | 工业机械臂 |
| **Dual Arm** | 12 轴 | 双臂协同控制 |

## 🔧 开发

### 编译特定组件

```bash
colcon build --packages-select arm_controller
```

### 更新依赖

```bash
cd ~/robotic_arm_ws/src/universal-arm-controller/src
vcs pull < ../deps.repos
```

### 项目结构

```
src/
├── arm_controller/          # 控制核心
├── controller_interfaces/   # 消息定义
├── robotic_arm_bringup/     # 系统启动
├── trajectory_planning/     # 规划库 (VCS)
├── trajectory_interpolator/ # 插值库 (VCS)
└── hardware_driver/         # 驱动库 (VCS)
```

## 🔗 依赖库

- **[trajectory-planning](https://github.com/Ding-Kaiyue/trajectory-planning)** - 轨迹规划
- **[trajectory-interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator)** - 轨迹插值
- **[hardware-driver](https://github.com/Ding-Kaiyue/hardware-driver)** - 硬件驱动

## 📄 许可证

MIT License - 详见 [LICENSE](LICENSE) 文件

## 📞 联系方式

- **GitHub Issues**: [提交问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues)
  - 使用预定义的 Issue 模板报告 Bug、功能请求或安全问题
- **Email**: <kaiyue.ding@raysense.com>

## 🤝 贡献

欢迎贡献！详见 [CONTRIBUTING.md](.github/CONTRIBUTING.md)

---

⭐ **如果这个项目对你有帮助，请给我们一个星标！**
