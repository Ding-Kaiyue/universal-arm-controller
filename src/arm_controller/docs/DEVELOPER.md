# 开发者指南

本文档面向 Arm Controller 的开发者。

## 📋 目录

- [开发环境搭建](#开发环境搭建)
- [项目结构](#项目结构)
- [构建系统](#构建系统)
- [开发流程](#开发流程)
- [测试](#测试)
- [调试](#调试)

---

## 开发环境搭建

### 必需工具

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# 开发工具
sudo apt install python3-vcstool python3-colcon-common-extensions

# 依赖库
sudo apt install libyaml-cpp-dev libeigen3-dev
```

### 克隆项目

```bash
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller/src
vcs import < ../deps.repos
```

### 编译

```bash
cd ~/robotic_arm_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
```

---

## 项目结构

```
arm_controller/
├── include/arm_controller/    # 头文件
│   ├── controller_base/       # 控制器基类
|   ├── controller_interface/  # 控制器接口
│   ├── hardware/              # 硬件接口
│   ├── utils/                 # 工具类
|   ├── controller_interface.hpp            # 控制器接口头文件
|   ├── controller_manager_section.hpp      # 控制器管理器头文件
|   └── trajectory_controller_section.hpp   # 轨迹控制器头文件
├── src/                       # 源文件
├── config/                    # 配置文件
├── test/                      # 测试
└── docs/                      # 文档
```

### 核心模块

| 模块 | 说明 | 关键文件 |
|-----|------|---------|
| ControllerManager | 控制器管理 | `controller_manager_section.hpp/cpp` |
| Controllers | 各种控制器 | `controller/*.hpp/cpp` |
| HardwareManager | 硬件接口 | `hardware/hardware_manager.hpp/cpp` |
| TrajectoryController | 轨迹执行 | `trajectory_controller_section.hpp/cpp` |

---

## 构建系统

### CMakeLists.txt 关键部分

```cmake
# 查找依赖
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# 添加库
add_library(${PROJECT_NAME} SHARED
  src/controller_manager.cpp
  src/controllers/movej_controller.cpp
  # ...
)

# 链接依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  # ...
)

# 安装
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
```

### 编译选项

```bash
# Debug 模式
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Release 模式
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 仅编译 arm_controller
colcon build --packages-select arm_controller

# 符号链接安装
colcon build --symlink-install
```

---

## 开发流程

### 1. 创建分支

```bash
git checkout -b feature/new-controller
```

### 2. 实现功能

#### 添加新控制器示例

Arm Controller 采用 **IPC 命令队列架构**。新控制器应实现以下两个关键方法：
- `move()` - C++ API 入口点，验证并将命令推送到 IPC 队列
- `command_queue_consumer_thread()` - 后台线程，从 IPC 队列消费命令并执行

参考现有实现：[MoveJController](../src/controller/movej/)、[JointVelocityController](../src/controller/joint_velocity/)

##### 示例 1: 轨迹控制器 (如 MyTrajectory)

```cpp
// src/controller/my_trajectory/my_trajectory_controller.hpp
#ifndef __MY_TRAJECTORY_CONTROLLER_HPP__
#define __MY_TRAJECTORY_CONTROLLER_HPP__

#include <controller_base/trajectory_controller_base.hpp>
#include "ipc/ipc_context.hpp"
#include "hardware/hardware_manager.hpp"
#include <thread>
#include <atomic>

namespace my_trajectory {

class MyTrajectoryController final : public TrajectoryControllerBase {
public:
    explicit MyTrajectoryController(const rclcpp::Node::SharedPtr& node);
    ~MyTrajectoryController() override;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

    // C++ IPC API - 供外部程序调用
    bool move(const std::vector<double>& target, const std::string& mapping);

private:
    // IPC 消费者线程
    void command_queue_consumer_thread();

    // 执行逻辑
    bool plan_and_execute(const std::string& mapping, const std::vector<double>& target);

    std::shared_ptr<HardwareManager> hardware_manager_;
    std::shared_ptr<arm_controller::ipc::IPCContext> ipc_context_;

    // 线程管理
    std::thread queue_consumer_;
    std::atomic<bool> consumer_running_{false};
};

}  // namespace my_trajectory

#endif  // __MY_TRAJECTORY_CONTROLLER_HPP__
```

```cpp
// src/controller/my_trajectory/my_trajectory_controller.cpp
#include "my_trajectory_controller.hpp"
#include <iostream>

namespace my_trajectory {

MyTrajectoryController::MyTrajectoryController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerBase("MyTrajectory", node)
{
    hardware_manager_ = HardwareManager::getInstance();
    ipc_context_ = arm_controller::ipc::IPCContext::getInstance();
}

MyTrajectoryController::~MyTrajectoryController() {
    if (consumer_running_) {
        consumer_running_ = false;
        if (queue_consumer_.joinable()) {
            queue_consumer_.join();
        }
    }
}

void MyTrajectoryController::start(const std::string& mapping) {
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::thread(&MyTrajectoryController::command_queue_consumer_thread, this);
    }
    is_active_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] MyTrajectoryController activated", mapping.c_str());
}

bool MyTrajectoryController::stop(const std::string& mapping) {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] MyTrajectoryController deactivated", mapping.c_str());
    return true;
}

bool MyTrajectoryController::move(const std::vector<double>& target, const std::string& mapping) {
    // 验证参数
    if (target.size() != 6) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Invalid target size: %zu", target.size());
        return false;
    }

    // 将命令推送到 IPC 队列
    try {
        ipc_context_->enqueueCommand(mapping, "MyTrajectory", target);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to enqueue command: %s", e.what());
        return false;
    }
}

void MyTrajectoryController::command_queue_consumer_thread() {
    while (consumer_running_) {
        // 从 IPC 队列消费该控制器的命令
        try {
            auto cmd = ipc_context_->dequeueCommand("MyTrajectory", std::chrono::milliseconds(100));
            if (cmd) {
                plan_and_execute(cmd->mapping, cmd->parameters);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Dequeue error: %s", e.what());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool MyTrajectoryController::plan_and_execute(
    const std::string& mapping,
    const std::vector<double>& target) {
    // 实现你的轨迹规划逻辑
    RCLCPP_INFO(node_->get_logger(), "[%s] Planning and executing trajectory", mapping.c_str());

    // 规划轨迹
    // auto trajectory = plan_trajectory(target);

    // 执行轨迹
    // hardware_manager_->executeTrajectory(mapping, trajectory);

    return true;
}

}  // namespace my_trajectory
```

##### 示例 2: 速度控制器 (如 JointVelocity)

```cpp
// src/controller/my_velocity/my_velocity_controller.hpp
#ifndef __MY_VELOCITY_CONTROLLER_HPP__
#define __MY_VELOCITY_CONTROLLER_HPP__

#include <controller_base/velocity_controller_base.hpp>
#include "ipc/ipc_context.hpp"
#include "hardware/hardware_manager.hpp"
#include <thread>
#include <atomic>

namespace my_velocity {

class MyVelocityController final : public VelocityControllerBase {
public:
    explicit MyVelocityController(const rclcpp::Node::SharedPtr& node);
    ~MyVelocityController() override;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

    // C++ IPC API
    bool move(const std::vector<double>& velocity, const std::string& mapping);

private:
    // IPC 消费者线程
    void command_queue_consumer_thread();

    // 执行逻辑
    bool execute_velocity(const std::string& mapping, const std::vector<double>& velocity);

    std::shared_ptr<HardwareManager> hardware_manager_;
    std::shared_ptr<arm_controller::ipc::IPCContext> ipc_context_;

    // 线程管理
    std::thread queue_consumer_;
    std::atomic<bool> consumer_running_{false};
};

}  // namespace my_velocity

#endif  // __MY_VELOCITY_CONTROLLER_HPP__
```

```cpp
// src/controller/my_velocity/my_velocity_controller.cpp
#include "my_velocity_controller.hpp"

namespace my_velocity {

MyVelocityController::MyVelocityController(const rclcpp::Node::SharedPtr& node)
    : VelocityControllerBase("MyVelocity", node)
{
    hardware_manager_ = HardwareManager::getInstance();
    ipc_context_ = arm_controller::ipc::IPCContext::getInstance();
}

MyVelocityController::~MyVelocityController() {
    if (consumer_running_) {
        consumer_running_ = false;
        if (queue_consumer_.joinable()) {
            queue_consumer_.join();
        }
    }
}

void MyVelocityController::start(const std::string& mapping) {
    if (!consumer_running_) {
        consumer_running_ = true;
        queue_consumer_ = std::thread(&MyVelocityController::command_queue_consumer_thread, this);
    }
    is_active_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] MyVelocityController activated", mapping.c_str());
}

bool MyVelocityController::stop(const std::string& mapping) {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] MyVelocityController deactivated", mapping.c_str());
    return true;
}

bool MyVelocityController::move(const std::vector<double>& velocity, const std::string& mapping) {
    // 验证参数
    if (velocity.size() != 6) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Invalid velocity size: %zu", velocity.size());
        return false;
    }

    // 将命令推送到 IPC 队列
    try {
        ipc_context_->enqueueCommand(mapping, "MyVelocity", velocity);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to enqueue command: %s", e.what());
        return false;
    }
}

void MyVelocityController::command_queue_consumer_thread() {
    while (consumer_running_) {
        try {
            auto cmd = ipc_context_->dequeueCommand("MyVelocity", std::chrono::milliseconds(100));
            if (cmd) {
                execute_velocity(cmd->mapping, cmd->parameters);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Dequeue error: %s", e.what());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool MyVelocityController::execute_velocity(
    const std::string& mapping,
    const std::vector<double>& velocity) {
    // 实现你的速度控制逻辑
    RCLCPP_INFO(node_->get_logger(), "[%s] Executing velocity control", mapping.c_str());

    // 执行速度控制
    // hardware_manager_->controlVelocity(mapping, velocity);

    return true;
}

}  // namespace my_velocity
```

#### 注册控制器

在 `src/controller/controller_registry.cpp` 中注册新控制器:

```cpp
#include "my_trajectory/my_trajectory_controller.hpp"
#include "my_velocity/my_velocity_controller.hpp"

std::unordered_map<std::string, ControllerFactory::Creator> get_available_controllers() {
    return {
        // ... 现有控制器 ...

        // 添加新控制器（IPC 模式）
        {"MyTrajectory", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<my_trajectory::MyTrajectoryController>(node); }},
        {"MyVelocity", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<my_velocity::MyVelocityController>(node); }}
    };
}
```

#### 使用新控制器（C++ IPC API）

```cpp
#include "arm_controller_api.hpp"
#include "controller/my_trajectory/my_trajectory_ipc_interface.hpp"

using namespace arm_controller;

int main() {
    // 初始化 IPC
    IPCLifecycle::initialize();

    // 创建接口实例
    my_trajectory::MyTrajectoryIPCInterface my_traj;

    // 调用 move() 方法推送命令到 IPC 队列
    std::vector<double> target = {0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0};
    if (my_traj.execute(target, "single_arm")) {
        std::cout << "✅ Command enqueued\n";
    }

    IPCLifecycle::shutdown();
    return 0;
}
```

#### IPC 接口包装类

为了简化 API 使用，建议为每个控制器创建一个 IPC 接口包装类：

```cpp
// src/controller/my_trajectory/my_trajectory_ipc_interface.hpp
#ifndef __MY_TRAJECTORY_IPC_INTERFACE_HPP__
#define __MY_TRAJECTORY_IPC_INTERFACE_HPP__

#include "ipc/ipc_context.hpp"

namespace my_trajectory {

class MyTrajectoryIPCInterface {
public:
    bool execute(const std::vector<double>& target, const std::string& mapping) {
        auto ipc = arm_controller::ipc::IPCContext::getInstance();
        try {
            ipc->enqueueCommand(mapping, "MyTrajectory", target);
            return true;
        } catch (...) {
            return false;
        }
    }

    std::string getLastError() const { return last_error_; }

private:
    std::string last_error_;
};

}  // namespace my_trajectory

#endif  // __MY_TRAJECTORY_IPC_INTERFACE_HPP__
```

### 3. 测试

```bash
# 运行测试
colcon test --packages-select arm_controller

# 查看测试结果
colcon test-result --verbose
```

### 4. 提交代码

```bash
git add .
git commit -m "feat: add new controller

- Implement MyController
- Add unit tests
- Update documentation"
git push origin feature/new-controller
```

### 5. CI

提交后稍等 5 分钟，CI 系统会自动运行测试，请保证测试通过。
---

## 测试

### 单元测试

项目已集成 gtest 单元测试框架，覆盖率约 34%。当前已有单元测试覆盖核心功能。

**现有测试**:
- ✅ IPC 命令队列入队/出队
- ✅ 控制器 move() 方法验证
- ✅ 多臂命令处理
- ✅ 基础硬件接口

**计划添加测试**:
- 📋 ControllerManager 的控制器注册和切换
- 📋 各个控制器的 start/stop 方法
- 📋 HoldState 的状态转移逻辑
- 📋 CartesianVelocity QP 求解器
- 📋 VelocityQPSolver 工具类

### 集成测试 (已验证)

参考完整的集成测试示例：[example_single_arm.cpp](../example/example_single_arm.cpp)

**运行集成测试**:

```bash
# 1. 编译项目
cd ~/robotic_arm_ws
colcon build --packages-select arm_controller --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. 运行集成测试程序
./install/arm_controller/bin/example_single_arm

# 3. 预期输出
# ===============================================================
# ARM Controller IPC 演示
# ===============================================================
# 📍 初始化 IPC...
# ✅ 初始化成功
#
# ========== MoveJ 演示 ==========
# 发送 MoveJ 命令 -> left_arm ...
# ✅ 已入队
#
# ========== MoveL 演示 ==========
# ...
```

**测试覆盖内容** (参考 example_single_arm.cpp):
- ✅ IPC 系统初始化和清理
- ✅ MoveJ 控制器命令入队
- ✅ MoveL 控制器命令入队
- ✅ MoveC 控制器命令入队
- ✅ 多臂单臂映射支持
- ✅ 多个命令的顺序保证
- ✅ 命令入队成功/失败处理

**创建自己的集成测试**:

参考 example_single_arm.cpp 的模式：

```cpp
#include "arm_controller/arm_controller_api.hpp"
#include "controller/movej/movej_ipc_interface.hpp"
#include "controller/movel/movel_ipc_interface.hpp"

using namespace arm_controller;

int main() {
    // 初始化 IPC
    if (!IPCLifecycle::initialize()) {
        std::cerr << "❌ 初始化失败\n";
        return 1;
    }

    // 创建控制器接口
    movej::MoveJIPCInterface movej;
    movel::MoveLIPCInterface movel;

    // 测试 MoveJ
    if (movej.execute({0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0}, "single_arm")) {
        std::cout << "✅ MoveJ 已入队\n";
    } else {
        std::cerr << "❌ MoveJ 失败: " << movej.getLastError() << "\n";
    }

    // 测试 MoveL
    if (movel.execute(0.19, -0.5, 0.63, -0.4546, 0.4546, -0.5417, 0.5417, "single_arm")) {
        std::cout << "✅ MoveL 已入队\n";
    } else {
        std::cerr << "❌ MoveL 失败: " << movel.getLastError() << "\n";
    }

    // 清理
    IPCLifecycle::shutdown();
    return 0;
}
```

---

## 代码审查

我们鼓励使用自动化工具检查代码质量，但代码格式问题不会阻止代码提交。重点是代码的**功能正确性**和**可维护性**。

### 自动检查工具

**代码格式检查** (推荐，但非强制):

```bash
# 检查代码风格
ament_cpplint src/

# 可选：自动格式化
ament_uncrustify --reformat src/
```

**静态分析** (推荐检查):

```bash
# 静态代码分析，查找潜在的逻辑错误
ament_cppcheck src/
```

### 提交前检查清单

**必须满足的要求** ✅:
- [ ] 代码通过编译，无编译错误和警告
- [ ] 通过集成测试验证
- [ ] 代码具有清晰的逻辑和注释
- [ ] 相关文档已更新
- [ ] 提交信息清晰描述改动内容

**可选建议** (鼓励，但非强制):
- [ ] 代码通过 ament_cpplint 检查
- [ ] 代码通过 ament_cppcheck 静态分析
- [ ] 遵循[代码规范](CODE_STYLE.md)

---

## 性能分析

### CPU 性能分析

```bash
# 使用 perf
perf record ros2 run arm_controller controller_manager
perf report

# 使用 valgrind
valgrind --tool=callgrind ros2 run arm_controller controller_manager
kcachegrind callgrind.out.*
```

### 内存分析

```bash
# 内存泄漏检测
valgrind --leak-check=full ros2 run arm_controller controller_manager

# 内存使用监控
watch -n 1 "ps aux | grep controller_manager"
```

---

## 发布流程

### 版本号管理

遵循[语义化版本](https://semver.org/):
- MAJOR.MINOR.PATCH (如 1.2.3)
- MAJOR: 不兼容的 API 修改
- MINOR: 向后兼容的功能性新增
- PATCH: 向后兼容的问题修正

### 发布步骤

1. 更新版本号(`package.xml`)
2. 更新 CHANGELOG
3. 创建 tag
4. 推送到 GitHub
5. 创建 Release

```bash
git tag -a v1.0.0 -m "Release version 1.0.0"
git push origin v1.0.0
```

---

## 相关资源

- [ROS2 开发者指南](https://docs.ros.org/en/humble/Contributing.html)
- [代码规范](CODE_STYLE.md)
- [系统架构](ARCHITECTURE.md)
- [GitHub 仓库](https://github.com/Ding-Kaiyue/universal-arm-controller)

---

## 获取帮助

- **Issues**: [GitHub Issues](https://github.com/Ding-Kaiyue/universal-arm-controller/issues)
- **Email**: kaiyue.ding@raysense.com
