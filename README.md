# Universal Arm Controller

通用机械臂控制系统 - 基于 ROS2 的模块化多组件机器人控制框架

## 项目简介

Universal Arm Controller 是一个完整的机械臂控制系统解决方案，由多个独立的 ROS2 组件构成。系统采用模块化设计，各组件职责明确，可独立开发和维护。

## 项目结构

本项目包含以下核心组件：

```
universal_arm_controller/
├── deps.repos                          # 依赖组件配置文件
├── src/
│   ├── arm_controller/                 # 核心：机械臂控制器（状态管理、模式切换）
│   ├── controller_interfaces/          # 自定义服务和消息接口
│   ├── robotic_arm_bringup/            # 启动文件和配置
│   ├── trajectory_planning/            # 外部依赖：轨迹规划库
│   ├── trajectory_interpolator/        # 外部依赖：轨迹插值库
│   └── hardware_driver/                # 外部依赖：硬件驱动（CAN-FD）
└── README.md                           # 本文件
```

### 组件说明

#### 内部组件（本仓库维护）

- **arm_controller** - 核心控制器组件
  - 多模式控制器（MoveJ/MoveL/MoveC/速度控制等）
  - 状态管理和安全监控
  - 双臂协同控制支持
  - [详细文档](src/arm_controller/README.md)

- **controller_interfaces** - 自定义 ROS2 接口
  - WorkMode.srv - 工作模式切换服务
  - 其他自定义消息类型

- **robotic_arm_bringup** - 系统启动
  - 单臂/双臂系统启动文件
  - 配置文件管理

#### 外部依赖组件（通过 deps.repos 获取）

- **trajectory_planning** - 轨迹规划库
  - MoveIt2 集成和适配
  - 多种规划策略（MoveJ/MoveL/MoveC）
  - 碰撞检测和避障
  - 仓库：https://github.com/Ding-Kaiyue/trajectory-planning.git

- **trajectory_interpolator** - 轨迹插值库
  - 多种样条插值算法（Linear/Cubic Spline/Cubic Hermite）
  - 可配置边界条件
  - 动力学约束满足
  - 仓库：https://github.com/Ding-Kaiyue/trajectory-interpolator.git

- **hardware_driver** - 硬件驱动库
  - CAN-FD 协议通信
  - 实时电机控制
  - 状态反馈和监控
  - 仓库：https://github.com/Ding-Kaiyue/hardware-driver.git

## 快速开始

### 系统要求

- **操作系统**：Ubuntu 22.04 或更高版本
- **ROS 版本**：ROS2 Humble 或更高版本
- **编译器**：支持 C++17 的 GCC/Clang
- **工具**：
  - colcon
  - vcstool (用于处理 deps.repos)

### 安装步骤

#### 1. 创建工作空间并克隆主仓库

```bash
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller
```

#### 2. 使用 vcstool 获取依赖组件

```bash
# 安装 vcstool（如果尚未安装）
sudo apt install python3-vcstool

# 进入 universal-arm-controller 的 src 目录
cd ~/robotic_arm_ws/src/universal-arm-controller/src

# 使用 deps.repos 获取所有依赖（deps.repos 在上一级目录）
vcs import < ../deps.repos --recursive
```

这将自动克隆以下依赖到当前目录（与 `arm_controller` 平级）：
- `trajectory_planning/` (planning-only 分支)
- `hardware_driver/` (master 分支)
- `trajectory_interpolator/` (master 分支)

#### 3. 安装 ROS2 依赖

```bash
cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y
```

#### 4. 编译项目

```bash
cd ~/robotic_arm_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 5. 设置环境

```bash
source ~/robotic_arm_ws/install/setup.bash
```

### 配置

#### 硬件配置

编辑 `src/arm_controller/config/hardware_config.yaml` 配置您的机械臂硬件参数：

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

### 启动系统

```bash
# 启动完整的机械臂控制系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

## 使用文档

详细的使用文档请参考各组件的 README：

- **[arm_controller 使用文档](src/arm_controller/README.md)** - 核心控制器的详细使用说明
  - 所有控制模式的使用方法
  - ROS2 话题和服务接口
  - 配置选项和参数
  - 常见问题排查

## 开发指南

### 更新依赖组件

如果外部依赖组件有更新，可以使用以下命令更新：

```bash
cd ~/robotic_arm_ws/src/universal-arm-controller/src
vcs pull < ../deps.repos
```

### 修改依赖版本

如果需要使用不同版本的依赖组件，编辑根目录的 `deps.repos` 文件：

```yaml
repositories:
  trajectory_planning:
    type: git
    url: https://github.com/Ding-Kaiyue/trajectory-planning.git
    version: planning-only  # 修改为需要的分支或tag
```

然后重新导入：

```bash
cd ~/robotic_arm_ws/src/universal-arm-controller/src
vcs import < ../deps.repos --force
```

### 添加新的依赖

在 `deps.repos` 中添加新的组件：

```yaml
repositories:
  your_new_component:
    type: git
    url: https://github.com/your-org/your-component.git
    version: master
```

### 项目架构

```
┌────────────────────────────────────────────────────────────────┐
│                   Universal Arm Controller                     │
│                                                                │
│  ┌─────────────────────┐      ┌─────────────────────────┐      │
│  │ ControllerManager   │      │ TrajectoryController    │      │
│  │ Section             │      │ Section                 │      │
│  │ (arm_controller)    │      │ (arm_controller)        │      │
│  └──────────┬──────────┘      └───────────┬─────────────┘      │
│             │                             │                    │
│             └─────────────┬───────────────┘                    │
│                           │                                    │
│            ┌──────────────▼───────────────┐                    │
│            │      HardwareManager         │                    │
│            │    (hardware_driver)         │                    │
│            └──────────────┬───────────────┘                    │
│                           │                                    │
│         ┌─────────────────┼─────────────────┐                  │
│         │                 │                 │                  │
│  ┌──────▼────────┐ ┌──────▼────────┐ ┌─────▼──────┐            │
│  │ Trajectory    │ │ Trajectory    │ │  CAN-FD    │            │
│  │ Planning      │ │ Interpolator  │ │  Driver    │            │
│  └───────────────┘ └───────────────┘ └─────┬──────┘            │
└─────────────────────────────────────────────┼──────────────────┘
                                              │
                                       ┌──────▼───────┐
                                       │   Hardware   │
                                       │   (Motors)   │
                                       └──────────────┘
```

## 常见问题

### Q: vcs import 失败

**A:** 确保已安装 vcstool 并且网络连接正常：
```bash
sudo apt install python3-vcstool
```

### Q: 编译失败，提示找不到依赖

**A:** 确保已正确导入所有依赖：
```bash
cd ~/robotic_arm_ws/src/universal-arm-controller/src
vcs import < ../deps.repos --recursive
cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Q: 如何只编译特定组件

**A:** 使用 colcon 的 --packages-select 选项：
```bash
colcon build --packages-select arm_controller
```

### Q: 依赖组件在哪里

**A:** 通过 `vcs import` 导入的依赖会放在 `src/universal_arm_controller/src/` 目录下，与 `arm_controller` 平级：

```
~/robotic_arm_ws/src/universal_arm_controller/src/
├── arm_controller/              # 本仓库内容
├── controller_interfaces/       # 本仓库内容
├── robotic_arm_bringup/         # 本仓库内容
├── trajectory_planning/         # vcs 导入
├── trajectory_interpolator/     # vcs 导入
└── hardware_driver/             # vcs 导入
```

## 贡献指南

欢迎贡献代码！请遵循以下步骤：

1. Fork 本仓库和相关依赖仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

对于外部依赖组件的修改，请直接向对应仓库提交 PR。

## 许可证

待定

## 联系方式

- **维护者**：Ding Kaiyue
- **邮箱**：kaiyue.ding@raysense.com
## 相关仓库

- [trajectory-planning](https://github.com/Ding-Kaiyue/trajectory-planning) - 轨迹规划库
- [trajectory-interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator) - 轨迹插值库
- [hardware-driver](https://github.com/Ding-Kaiyue/hardware-driver) - 硬件驱动库

---

**提示**：首次使用请先阅读 [arm_controller 使用文档](src/arm_controller/README.md) 了解系统的详细功能和使用方法。
