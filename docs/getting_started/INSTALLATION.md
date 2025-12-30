# 安装指南

详细的 Universal Arm Controller 安装步骤。

## 📋 目录

- [系统要求](#系统要求)
- [前置准备](#前置准备)
- [标准安装流程](#标准安装流程)
- [安装验证](#安装验证)
- [故障排除](#故障排除)

---

## 系统要求

### 操作系统

- **Ubuntu 22.04 LTS** 或更高版本（推荐 Ubuntu 22.04）
- 需要 Linux 内核 4.4+ 以支持 CAN

### 软件依赖

- **ROS2 Humble** 或更高版本
- **GCC 10+** 或 **Clang 12+**（支持 C++17）
- **CMake 3.10+**
- **Python 3.10+**

### 工具软件

```bash
# 必需工具
sudo apt install python3-vcstool python3-colcon-common-extensions
sudo apt install build-essential cmake git

# 依赖库
sudo apt install libyaml-cpp-dev libeigen3-dev libfmt-dev
```

### 硬件要求

- **CAN 接口**: 兼容 SocketCAN 的 CAN-FD 接口
- **内存**: 最少 4GB RAM（推荐 8GB）
- **CPU**: 四核或以上

---

## 前置准备

### 1. 安装 ROS2 Humble

如果还未安装 ROS2，按照 [ROS2 官方文档](https://docs.ros.org/en/humble/Installation.html) 安装。

```bash
# 验证 ROS2 安装
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2. 安装 MoveIt2

```bash
# 安装 MoveIt2
sudo apt install ros-humble-moveit ros-humble-moveit-servo
```

### 3. 安装 TracIK（必选）

```bash
# 创建工作空间
mkdir -p ~/trac_ik_ws/src
cd ~/trac_ik_ws/src

# 克隆 TracIK
git clone https://github.com/aprotyas/trac_ik.git

# 编译
cd ~/trac_ik_ws
colcon build
source install/setup.bash

# 添加到 bashrc
echo "source ~/trac_ik_ws/install/setup.bash" >> ~/.bashrc
```

### 4. 安装 QP 求解器（必选）

安装 OSQP 和 OsqpEigen（用于二次规划优化求解）：

```bash
# 创建临时工作目录
mkdir -p ~/osqp_build

# 编译 OSQP
cd ~/osqp_build
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig

# 编译 OsqpEigen
cd ~/osqp_build
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig

# 验证安装
pkg-config --modversion osqp
pkg-config --modversion osqp-eigen
```

---

## 标准安装流程

### 步骤 1：创建工作空间

```bash
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src
```

### 步骤 2：克隆仓库

```bash
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller
```

### 步骤 3：导入依赖

```bash
# 进入源码目录
cd src

# 安装 vcstool（如果还未安装）
sudo apt install python3-vcstool

# 导入依赖（包括 hardware_driver、trajectory_interpolator、trajectory_planning）
vcs import < ../deps.repos --recursive

# 验证导入
ls -la
# 应该看到：arm_controller, controller_interfaces, robotic_arm_bringup,
# hardware_driver, trajectory_interpolator, trajectory_planning
```

### 步骤 4：安装 ROS 依赖

```bash
cd ~/robotic_arm_ws

# 自动安装所有依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 步骤 5：编译

```bash
# 编译（Release 模式）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 或使用 Debug 模式（用于调试）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
```

### 步骤 6：环境配置

```bash
# 设置环境变量
source ~/robotic_arm_ws/install/setup.bash

# 添加到 bashrc（可选但推荐）
echo "source ~/robotic_arm_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 安装验证

### 检查编译结果

```bash
# 验证文件夹存在
ls ~/robotic_arm_ws/install/

# 应该看到：
# arm_controller  controller_interfaces  hardware_driver
# robotic_arm_bringup  trajectory_interpolator  trajectory_planning
```

### 验证 ROS 包

```bash
# 检查 arm_controller 是否可被 ROS 找到
ros2 pkg list | grep arm_controller

# 应该输出：
# arm_controller
# controller_interfaces
```

### 验证启动文件

```bash
# 检查启动文件是否存在
ls ~/robotic_arm_ws/install/robotic_arm_bringup/share/robotic_arm_bringup/launch/

# 应该看到 robotic_arm_real.launch.py
```

---

## 故障排除

### 错误：找不到 vcstool

```bash
sudo apt install python3-vcstool
```

### 错误：找不到 ROS 依赖

```bash
# 重新运行依赖安装
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 错误：编译失败，找不到头文件

```bash
# 清理并重新编译
cd ~/robotic_arm_ws
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 错误：CAN 接口无法启动

```bash
# 检查内核支持
uname -r

# 配置 CAN 接口
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 \
  dbitrate 5000000 dsample-point 0.75 fd on loopback off restart-ms 100

# 验证
ip link show can0
```

### 权限问题

```bash
# 添加用户到 dialout 组（处理 CAN 接口权限）
sudo usermod -a -G dialout $USER

# 重新登录或运行
newgrp dialout
```

---

## 下一步

- 👉 查看 [快速开始](QUICKSTART.md) 运行你的第一个程序
- 📖 学习 [系统概览](COMPONENTS.md)
- ⚙️ 参考 [Arm Controller 配置](../src/arm_controller/docs/CONFIGURATION.md)

---

**遇到问题？** 查看 [故障排除](TROUBLESHOOTING.md) 或提交 [GitHub Issue](https://github.com/Ding-Kaiyue/universal-arm-controller/issues)。
