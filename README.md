# Universal Arm Controller

一个**完整的机械臂控制系统解决方案**，基于 ROS2 的模块化多组件架构，集成了轨迹规划、轨迹插值、硬件驱动等核心功能，为工业机械臂提供高效、可靠的运动控制。

> 📖 **快速导航**: 首次使用请先参考 [arm_controller 使用文档](src/arm_controller/README.md) 了解系统详细功能。

## 🚀 特性

- **模块化设计**: 系统采用清晰的分层架构，各组件职责明确，可独立开发和维护
- **四种运动模式**: MoveJ(关节空间) + MoveL(直线) + MoveC(圆弧) + 速度控制模式
- **高性能集成**: 基于 MoveIt2 + TracIK 的快速轨迹生成，微秒级控制延迟
- **实时反馈**: 事件驱动架构，实时电机状态监控，支持观察者模式和事件总线
- **双臂支持**: 原生支持单臂/双臂协同控制
- **工业级可靠性**: CAN-FD 高速通信，线程安全设计，CPU亲和性绑定

## 📦 系统组件

本项目采用 **VCS 管理架构**，集成三个高性能的独立库：

### 核心组件（内部维护）

| 组件 | 功能 | 位置 |
|------|------|------|
| **arm_controller** | 运动控制核心 | `src/arm_controller/` |
| **controller_interfaces** | ROS2 消息/服务定义 | `src/controller_interfaces/` |
| **robotic_arm_bringup** | 系统启动和配置 | `src/robotic_arm_bringup/` |

### 集成库（通过 deps.repos 获取）

| 库 | 功能 | 仓库 |
|-----|------|------|
| **hardware_driver** | CAN-FD硬件控制 | [hardware-driver](https://github.com/Ding-Kaiyue/hardware-driver) |
| **trajectory_interpolator** | 样条轨迹插值 | [trajectory-interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator) |
| **trajectory_planning** | MoveIt2 规划集成 | [trajectory-planning](https://github.com/Ding-Kaiyue/trajectory-planning) |

## ⚡ 快速开始

### 系统要求

- **Ubuntu 22.04** LTS 或更高版本
- **ROS2 Humble** 或更高版本
- **GCC 10+** (C++17 支持)
- **工具**: colcon, vcstool

### 前置准备

安装 TracIK (唯一需要源码编译的依赖)：

```bash
mkdir -p ~/trac_ik_ws/src
cd ~/trac_ik_ws/src
git clone https://github.com/aprotyas/trac_ik.git
cd ~/trac_ik_ws
colcon build
source install/setup.bash

# 将 TracIK 环境写入系统配置（可选但推荐）
echo "source ~/trac_ik_ws/install/setup.bash" >> ~/.bashrc
```

### 快速安装

```bash
# 1. 创建工作空间
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src

# 2. 克隆主仓库和依赖
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller/src
sudo apt install python3-vcstool
vcs import < ../deps.repos --recursive

# 3. 更新依赖到最新版本
vcs pull < ../deps.repos

# 4. 编译和安装
cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 启动系统

```bash
# 启动完整的机械臂控制系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

### 硬件配置

编辑 `src/arm_controller/config/hardware_config.yaml`：

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

## 📚 使用文档

### 完整文档索引

- **[arm_controller 使用指南](src/arm_controller/README.md)** - 核心控制器详细文档
  - ✓ 多模式控制 (MoveJ/MoveL/MoveC)
  - ✓ ROS2 接口详解
  - ✓ 配置和参数管理
  - ✓ 常见问题排查

### 组件文档

每个集成库都有完整的独立文档：

| 组件 | 文档链接 | 功能 |
|------|---------|------|
| Hardware Driver | [📖 文档](https://github.com/Ding-Kaiyue/hardware-driver#readme) | CAN-FD 硬件控制、电机驱动 |
| Trajectory Interpolator | [📖 文档](https://github.com/Ding-Kaiyue/trajectory-interpolator#readme) | 样条插值算法、实时轨迹生成 |
| Trajectory Planning | [📖 文档](https://github.com/Ding-Kaiyue/trajectory-planning#readme) | MoveIt2 集成、运动规划策略 |

## 🔧 开发指南

### 依赖管理

```bash
# 更新所有依赖
cd ~/robotic_arm_ws/src/universal-arm-controller/src
vcs pull < ../deps.repos

# 修改依赖版本：编辑 deps.repos，然后重新导入
vcs import < ../deps.repos --force
```

### 编译选项

```bash
# 编译特定组件
colcon build --packages-select arm_controller

# 查看编译详情
colcon build --event-handlers console_direct+
```

### 项目架构

```
┌──────────────────────────── Universal Arm Controller ────────────────────────────┐
│                                                                                  │
│  ┌─────────────────────┐          ┌──────────────────────┐                      │
│  │   arm_controller    │          │ TrajectoryController │                      │
│  │  (Control Manager)  │          │ (Planning & Exec)    │                      │
│  └──────────┬──────────┘          └──────────┬───────────┘                      │
│             └─────────────┬──────────────────┘                                  │
│                           │                                                     │
│            ┌──────────────▼──────────────┐                                      │
│            │    HardwareManager          │                                      │
│            │   (hardware_driver lib)     │                                      │
│            └──────────────┬──────────────┘                                      │
│                           │                                                     │
│      ┌────────────────────┼────────────────────┐                                │
│      │                    │                    │                                │
│  ┌───▼────────┐   ┌──────▼────────┐   ┌──────▼──────┐                          │
│  │ Trajectory │   │ Trajectory    │   │   CAN-FD    │                          │
│  │ Planning   │   │ Interpolator  │   │   Driver    │                          │
│  └────────────┘   └───────────────┘   └──────┬──────┘                          │
└────────────────────────────────────────────────┼──────────────────────────────┘
                                                 │
                                          ┌──────▼───────┐
                                          │   Hardware   │
                                          │   (Motors)   │
                                          └──────────────┘
```

## 🔍 故障排除

### vcs import 失败

```bash
# 确保已安装 vcstool
sudo apt install python3-vcstool

# 检查网络连接
ping github.com
```

### 编译错误："找不到依赖"

```bash
# 重新导入依赖
cd ~/robotic_arm_ws/src/universal-arm-controller/src
vcs import < ../deps.repos --recursive

# 安装 ROS 依赖
cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 只编译特定组件

```bash
colcon build --packages-select arm_controller
```

### 依赖组件位置

通过 VCS 导入的依赖会被放在与 `arm_controller` 平级的目录：

```
src/universal_arm_controller/src/
├── arm_controller/          # ✓ 本仓库维护
├── controller_interfaces/   # ✓ 本仓库维护
├── robotic_arm_bringup/     # ✓ 本仓库维护
├── trajectory_planning/     # ◆ VCS 导入（可独立更新）
├── trajectory_interpolator/ # ◆ VCS 导入（可独立更新）
└── hardware_driver/         # ◆ VCS 导入（可独立更新）
```

## 💡 更多帮助

### 获取帮助

如在使用过程中遇到问题：

- **GitHub Issues**: [提交问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues)
- **Email**: kaiyue.ding@raysense.com

### 贡献指南

欢迎贡献代码！步骤如下：

1. Fork 本仓库和相关依赖仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

对于外部依赖组件的修改，请向对应仓库提交 PR。

## 📄 许可证

MIT License - 详见 [LICENSE](LICENSE) 文件

## 相关仓库

- [Hardware Driver](https://github.com/Ding-Kaiyue/hardware-driver) - CAN-FD 硬件驱动库
- [Trajectory Interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator) - 轨迹插值库
- [Trajectory Planning](https://github.com/Ding-Kaiyue/trajectory-planning) - 轨迹规划库

---

⭐ **如果这个项目对你有帮助，请给我们一个星标！**
