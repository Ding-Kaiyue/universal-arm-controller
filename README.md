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

#### 方式 A: Docker（推荐，特别适合内存受限系统）

**最简单的方式，一条命令完成所有工作。特别推荐给：**
- ✅ 内存不足 16GB 的系统（避免本地编译卡死）
- ✅ 无编译环境的用户
- ✅ 希望快速部署的用户

```bash
# 克隆仓库
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller

# 一键启动（首次会自动构建，耗时 30-60 分钟）
docker-compose up -d

# 进入容器
docker-compose exec robotic-arm bash

# 在容器内启动系统（源代码已编译完成）
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

**为什么选择 Docker？**
| 对比项 | 本地编译 | Docker 构建 |
|--------|---------|------------|
| 构建时间 | 10-30 分钟 | 30-60 分钟（仅首次）|
| 内存需求 | **≥16GB RAM + swap** | 4-8GB（Docker 容器可限制）|
| 系统卡死风险 | ⚠️ 高（低内存系统） | ✅ 低 |
| 环境依赖冲突 | 可能 | 完全隔离 |
| 后续编译 | 快（10+ 分钟） | 快（在容器内） |

详见 [Docker 完整指南](docs/DOCKER.md)

#### 方式 B: 本地编译

适合需要修改代码的开发者：

```bash
# 1. 创建工作空间
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src

# 2. 克隆仓库与依赖
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller/src
sudo apt install python3-vcstool
vcs import < ../deps.repos --recursive

# 3. 编译（使用官方构建脚本）
cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y
./src/universal-arm-controller/build.sh
source install/setup.bash
```

**重要**: 请使用 `build.sh` 而不是直接运行 `colcon build`。构建脚本会检查系统资源并应用安全的构建配置。

## 🔨 构建指南

### 系统要求与性能

| 场景 | 最低配置 | 推荐配置 | 编译时间 |
|------|---------|---------|---------|
| 标准开发 | 4核 CPU, 16GB RAM | 8核+, 32GB RAM | 5-10 分钟 |
| 低配机器 | 4核 CPU, 8GB RAM + 8GB swap | 8GB RAM + 16GB swap | 20-30 分钟 |

**注意**: 编译过程中，MoveIt2、Pinocchio 等重型库会导致高内存占用。如果机器配置低，请使用单线程构建模式。

### 标准构建（推荐）

适合 16GB+ 内存的机器：

```bash
cd ~/robotic_arm_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 低配机器构建（≤16GB RAM）

如果编译过程中系统卡死，请使用以下命令：

```bash
# 方法 1: 单线程构建（最稳定）
colcon build \
  --executor sequential \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1 -DCMAKE_BUILD_TYPE=Release

# 方法 2: 限制并行度
export MAKEFLAGS="-j2"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### ⚠️ 常见问题与解决方案

#### 编译时系统卡死

**原因**: 并行编译线程过多，导致 CPU/内存耗尽和 swap 风暴。

**诊断步骤**:
```bash
# 在编译时打开另一个终端，实时监控资源
watch -n 1 'free -h && echo "---" && top -b -n 1 | head -15'
```

**解决方案**:
```bash
# 使用单线程构建（完全解决卡死问题，但编译时间较长）
colcon build \
  --executor sequential \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1 -DCMAKE_BUILD_TYPE=Release
```

#### Debug 模式编译导致内存爆炸

**问题**: 不要使用 Debug 模式，否则内存占用会增加 10 倍以上。

```bash
# ❌ 禁止使用 Debug 模式
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# ✅ 使用 RelWithDebInfo（保留符号信息，内存占用更低）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

#### 编译特定包以加快开发循环

```bash
# 只编译 arm_controller，速度快 10 倍
colcon build --packages-select arm_controller
```

#### 为什么有些用户会遇到卡死问题？

这个工程为了性能和可移植性，使用了密集的 C++ 模板、多层抽象和复杂的链接依赖。这导致：

- 编译期符号解析复杂度高
- 最终链接阶段瞬时内存峰值可达 10GB+
- 低配机器（≤8GB RAM，无 swap）无法安全完成

**这不是代码问题**，而是工程规模问题。大型项目如 MoveIt、LLVM 等也有同样的约束。

**解决方案**（优先级顺序）：

1. **使用官方构建脚本**（推荐）
   ```bash
   ./build.sh    # 自动检查资源并应用安全配置
   ```

2. **确保足够的内存**
   - 最小需求：16GB 总可用内存（RAM + swap）
   - 推荐配置：16GB RAM + 8GB swap 或更多

   启用 swap：
   ```bash
   sudo fallocate -l 16G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

   # 验证
   free -h
   ```

3. **使用 Docker**（最简单，无需本地编译）
   ```bash
   docker-compose build
   docker-compose up -d
   ```
   详见 [Docker 快速开始](docs/DOCKER.md)

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
