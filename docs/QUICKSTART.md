# 快速开始

5分钟快速上手 Universal Arm Controller！

## 前提条件

- ✅ Ubuntu 22.04 LTS 或更高版本
- ✅ 根据 [安装指南](INSTALLATION.md) 配置好环境
- ✅ 网络连接正常

## 第一步：安装

```bash
# 1. 创建工作空间
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src

# 2. 克隆仓库与依赖
git clone -b feature/ipc-dual-arm https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller/src
sudo apt install python3-vcstool
vcs import < ../deps.repos --recursive

# 3. 编译
cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 第二步：配置 CAN 接口

```bash
# 配置 CAN0（根据你的硬件修改）
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 \
  dbitrate 5000000 dsample-point 0.75 fd on loopback off restart-ms 100

# 验证配置
ip link show can0
```

## 第三步：启动系统

```bash
# 启动完整的机械臂控制系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

## 第四步：编写控制程序

创建 C++ 程序使用 IPC API 进行控制（参考 [example_single_arm.cpp](../src/arm_controller/example/example_single_arm.cpp)）：

```cpp
#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/movel/movel_ipc_interface.hpp"
#include <iostream>

using namespace arm_controller;

int main() {
    // 初始化 IPC
    if (!IPCLifecycle::initialize()) {
        std::cerr << "初始化失败\n";
        return 1;
    }

    // 创建控制器接口
    movej::MoveJIPCInterface movej;
    movel::MoveLIPCInterface movel;

    // 单臂控制 - MoveJ (关节空间运动)
    std::cout << "执行 MoveJ 命令...\n";
    movej.execute({0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0}, "single_arm");

    // 单臂控制 - MoveL (笛卡尔空间直线运动)
    std::cout << "执行 MoveL 命令...\n";
    movel.execute(0.19, -0.5, 0.63, -0.4546, 0.4546, -0.5417, 0.5417, "single_arm");

    // 关闭 IPC
    IPCLifecycle::shutdown();
    return 0;
}
```

### 编译并运行

```bash
# 编译程序
cd ~/robotic_arm_ws
colcon build

# 运行控制程序
cd build/arm_controller/ && example_single_arm
```

### 多臂并发控制

```cpp
#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/joint_velocity/joint_velocity_ipc_interface.hpp"

using namespace arm_controller;

int main() {
    // 初始化 IPC
    IPCLifecycle::initialize();

    movej::MoveJIPCInterface movej;
    joint_velocity::JointVelocityIPCInterface joint_vel;

    // 臂 A 执行 MoveJ
    std::cout << "臂 A 执行 MoveJ...\n";
    movej.execute({0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0}, "left_arm");

    // 臂 B 同时执行关节速度控制（真并发！无需等待臂 A）
    std::cout << "臂 B 同时执行关节速度控制...\n";
    joint_vel.execute({0.3, 0.3, 0.0, 0.0, 0.0, 0.0}, "right_arm");

    std::cout << "两臂正在并发执行\n";

    // 关闭 IPC
    IPCLifecycle::shutdown();
    return 0;
}
```

> [!NOTE]
> 根据你的硬件配置，需要修改配置文件：
> - 机械臂的 URDF 模型路径
> - CAN 总线设备号（如 can0、can1）
> - 电机 ID 映射
> - 控制周期和速度限制
>
> 详见 [配置指南](../src/arm_controller/docs/CONFIGURATION.md) 和 [robotic_arm_bringup](../src/robotic_arm_bringup/) 配置文件

### 各控制模式详解

#### 1. MoveJ - 关节空间点到点运动
- 规划完整的关节轨迹
- 保证末端不碰撞
- 适合工业操作

#### 2. MoveL - 笛卡尔直线运动
- 末端直线路径
- 自动逆运动学求解
- 适合精密组装

#### 3. MoveC - 笛卡尔圆弧运动
- 末端圆弧路径
- 通过中间点定义圆弧
- 适合去毛刺、焊接

#### 4. JointVelocity - 关节速度控制
- 实时关节速度控制
- 快速响应
- 适合遥操作

#### 5. CartesianVelocity - 笛卡尔速度控制
- 实时笛卡尔速度控制
- 基于 QP 求解的逆速度雅可比
- 3层安全检查保障
- 适合视觉伺服

## 成功标志

- ✅ 系统启动无错误
- ✅ 程序成功初始化 IPC
- ✅ 命令成功入队
- ✅ 机械臂响应控制命令

## 下一步

- 📖 学习 [系统概览](COMPONENTS.md)
- ⚙️ 查看 [Arm Controller 配置指南](../src/arm_controller/docs/CONFIGURATION.md)
- 🏗️ 理解 [系统架构](ARCHITECTURE.md)

## 常见问题

**Q: 编译失败？**
A: 确保已安装所有依赖。运行 `rosdep install --from-paths src --ignore-src -r -y`

**Q: CAN 接口配置失败？**
A: 检查系统权限。可能需要 `sudo` 或将用户添加到 dialout 组。

**Q: 机械臂不动？**
A: 检查 CAN 接口是否启动，电机是否上电。查看 [故障排除](TROUBLESHOOTING.md)。

---

更多详细信息请查看 [完整文档中心](README.md)。
