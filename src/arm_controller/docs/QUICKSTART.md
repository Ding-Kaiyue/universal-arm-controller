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

## 第二步:编写 C++ 控制程序

Arm Controller 采用 IPC 命令队列架构，通过 C++ API 进行控制。参考完整示例：[example_single_arm.cpp](../example/example_single_arm.cpp)

```cpp
#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/movel/movel_ipc_interface.hpp"

using namespace arm_controller;

int main() {
    // 初始化 IPC
    if (!IPCLifecycle::initialize()) {
        return 1;
    }

    // 创建控制器接口
    movej::MoveJIPCInterface movej;
    movel::MoveLIPCInterface movel;

    // MoveJ 控制
    movej.execute({0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0}, "single_arm");

    // MoveL 控制
    movel.execute(0.19, -0.5, 0.63, -0.4546, 0.4546, -0.5417, 0.5417, "single_arm");

    IPCLifecycle::shutdown();
    return 0;
}
```

## 第三步:编译程序

```bash
# source 工作空间
source ~/robotic_arm_ws/install/setup.bash

# 编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 运行程序
./install/arm_controller/bin/example_single_arm
```

## 第四步:多臂并发控制

```cpp
#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/joint_velocity/joint_velocity_ipc_interface.hpp"

using namespace arm_controller;

int main() {
    IPCLifecycle::initialize();

    movej::MoveJIPCInterface movej;
    joint_velocity::JointVelocityIPCInterface joint_vel;

    // 左臂 MoveJ，右臂 JointVelocity（真并发！）
    movej.execute({0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0}, "left_arm");
    joint_vel.execute({0.3, 0.3, 0.0, 0.0, 0.0, 0.0}, "right_arm");

    IPCLifecycle::shutdown();
    return 0;
}
```

## 下一步

- 📖 阅读 [IPC 架构详解](IPC_ARCHITECTURE.md) 了解 IPC 通信机制
- 📖 阅读 [控制器详解](CONTROLLERS.md) 了解所有控制模式实现
- ⚙️ 查看 [配置指南](CONFIGURATION.md) 自定义配置
- 🏗️ 学习 [系统架构](ARCHITECTURE.md) 了解双节点架构和线程模型

## 常见问题

**Q: IPC 命令入队失败?**
A: 确保已调用 `IPCLifecycle::initialize()`，检查共享内存是否已创建。

**Q: 多臂不能真并发?**
A: 每臂内命令严格顺序执行，但不同臂之间完全并发（无需等待）。参考 [IPC_ARCHITECTURE.md](IPC_ARCHITECTURE.md)。

**Q: 如何添加新的控制模式?**
A: 参考 [CONTROLLERS.md](CONTROLLERS.md) 和 [DEVELOPER.md](DEVELOPER.md) 了解控制器扩展流程。