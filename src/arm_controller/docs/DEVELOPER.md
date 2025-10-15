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
├── include/arm_controller/     # 头文件
│   ├── controllers/           # 控制器
│   ├── hardware/              # 硬件接口
│   └── utils/                 # 工具类
├── src/                       # 源文件
├── config/                    # 配置文件
├── test/                      # 测试
└── docs/                      # 文档
```

### 核心模块

| 模块 | 说明 | 关键文件 |
|-----|------|---------|
| ControllerManager | 控制器管理 | `controller_manager.hpp/cpp` |
| Controllers | 各种控制器 | `controllers/*.hpp/cpp` |
| HardwareManager | 硬件接口 | `hardware/hardware_manager.hpp/cpp` |
| TrajectoryController | 轨迹执行 | `trajectory_controller.hpp/cpp` |

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

根据控制器类型,选择合适的基类:
- **轨迹控制器**: 继承 `TrajectoryControllerImpl<MessageType>` (需要钩子状态)
- **工具控制器**: 继承 `UtilityControllerBase` (不需要钩子状态)

注意: 你也可以自己改变是否需要钩子状态

##### 示例 1: 轨迹控制器 (如 MoveJ)

```cpp
// src/controller/my_trajectory/my_trajectory_controller.hpp
#ifndef __MY_TRAJECTORY_CONTROLLER_HPP__
#define __MY_TRAJECTORY_CONTROLLER_HPP__

#include <controller_base/trajectory_controller_base.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "hardware/hardware_manager.hpp"

class MyTrajectoryController final : public TrajectoryControllerImpl<sensor_msgs::msg::JointState> {
public:
    explicit MyTrajectoryController(const rclcpp::Node::SharedPtr& node);
    ~MyTrajectoryController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

private:
    void trajectory_callback(const sensor_msgs::msg::JointState::SharedPtr msg) override;
    void plan_and_execute(const std::string& mapping,
                         const sensor_msgs::msg::JointState::SharedPtr msg) override;

    std::shared_ptr<HardwareManager> hardware_manager_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    std::string active_mapping_;
};

#endif  // __MY_TRAJECTORY_CONTROLLER_HPP__
```

```cpp
// src/controller/my_trajectory/my_trajectory_controller.cpp
#include "my_trajectory_controller.hpp"
#include "controller_interface.hpp"

MyTrajectoryController::MyTrajectoryController(const rclcpp::Node::SharedPtr& node)
    : TrajectoryControllerImpl<sensor_msgs::msg::JointState>("MyTrajectory", node)
{
    hardware_manager_ = HardwareManager::getInstance();

    std::string input_topic;
    node_->get_parameter("controllers.MyTrajectory.input_topic", input_topic);

    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        input_topic, rclcpp::QoS(10).reliable(),
        std::bind(&MyTrajectoryController::trajectory_callback, this, std::placeholders::_1)
    );
}

void MyTrajectoryController::start(const std::string& mapping) {
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] MyTrajectory: not found in hardware configuration."
        );
    }

    active_mapping_ = mapping;
    is_active_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] MyTrajectoryController activated", mapping.c_str());
}

bool MyTrajectoryController::stop(const std::string& mapping) {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] MyTrajectoryController deactivated", mapping.c_str());
    return true;
}

void MyTrajectoryController::trajectory_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!is_active_) return;
    plan_and_execute(active_mapping_, msg);
}

void MyTrajectoryController::plan_and_execute(
    const std::string& mapping,
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 实现你的规划和执行逻辑
    RCLCPP_INFO(node_->get_logger(), "[%s] Executing trajectory", mapping.c_str());
}
```

##### 示例 2: 工具控制器 (如 Disable)

```cpp
// src/controller/my_utility/my_utility_controller.hpp
#ifndef __MY_UTILITY_CONTROLLER_HPP__
#define __MY_UTILITY_CONTROLLER_HPP__

#include "controller_base/utility_controller_base.hpp"
#include "hardware/hardware_manager.hpp"

class MyUtilityController final : public UtilityControllerBase {
public:
    explicit MyUtilityController(const rclcpp::Node::SharedPtr& node);
    ~MyUtilityController() override = default;

    void start(const std::string& mapping = "") override;
    bool stop(const std::string& mapping = "") override;

private:
    bool execute_utility_function(const std::string& mapping);
    std::shared_ptr<HardwareManager> hardware_manager_;
};

#endif  // __MY_UTILITY_CONTROLLER_HPP__
```

```cpp
// src/controller/my_utility/my_utility_controller.cpp
#include "my_utility_controller.hpp"
#include "controller_interface.hpp"

MyUtilityController::MyUtilityController(const rclcpp::Node::SharedPtr& node)
    : UtilityControllerBase("MyUtility", node)
{
    hardware_manager_ = HardwareManager::getInstance();
}

void MyUtilityController::start(const std::string& mapping) {
    const auto& all_mappings = hardware_manager_->get_all_mappings();
    if (std::find(all_mappings.begin(), all_mappings.end(), mapping) == all_mappings.end()) {
        throw std::runtime_error(
            "❎ [" + mapping + "] MyUtility: not found in hardware configuration."
        );
    }

    if (!execute_utility_function(mapping)) {
        throw std::runtime_error(
            "❎ [" + mapping + "] MyUtility: Failed to execute"
        );
    }

    is_active_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] MyUtilityController activated", mapping.c_str());
}

bool MyUtilityController::stop(const std::string& mapping) {
    is_active_ = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] MyUtilityController deactivated", mapping.c_str());
    return true;
}

bool MyUtilityController::execute_utility_function(const std::string& mapping) {
    // 实现你的工具功能逻辑
    RCLCPP_INFO(node_->get_logger(), "[%s] Executing utility function", mapping.c_str());
    return true;
}
```

#### 注册控制器

在 `src/controller/controller_registry.cpp` 中注册新控制器:

```cpp
#include "my_trajectory/my_trajectory_controller.hpp"
#include "my_utility/my_utility_controller.hpp"

std::unordered_map<std::string, ControllerInterface::Creator> get_available_controllers() {
    return {
        // ... 现有控制器 ...

        // 添加新控制器
        {"MyTrajectoryController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<MyTrajectoryController>(node); }},
        {"MyUtilityController", [](rclcpp::Node::SharedPtr node) {
            return std::make_shared<MyUtilityController>(node); }}
    };
}
```

#### 更新配置文件

在 `config/controller_config.yaml` 中添加配置:

```yaml
controllers:
  MyTrajectory:
    input_topic: "/controller_api/my_trajectory_action"
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

---

## 测试

### 单元测试

```cpp
// test/test_controller_manager.cpp
#include <gtest/gtest.h>
#include "arm_controller/controller_manager.hpp"

TEST(ControllerManagerTest, RegisterController) {
  auto manager = std::make_shared<ControllerManager>();
  auto controller = std::make_shared<MockController>();

  manager->registerController("TestMode", controller);

  EXPECT_TRUE(manager->hasController("TestMode"));
}
```

### 运行测试

```bash
# 编译测试
colcon build --cmake-args -DBUILD_TESTING=ON

# 运行所有测试
colcon test

# 运行特定测试
colcon test --packages-select arm_controller --ctest-args -R test_controller_manager
```

### 集成测试

```bash
# 启动系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py

# 在另一个终端测试
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'MoveJ', mapping: 'single_arm'}"
```

---

## 代码审查

### 自动检查

```bash
# 代码格式
ament_cpplint src/

# 静态分析
ament_cppcheck src/

# 格式化
ament_uncrustify --reformat src/
```

### 提交前检查清单

- [ ] 代码通过编译,无警告
- [ ] 遵循[代码规范](CODE_STYLE.md)
- [ ] 添加必要的注释
- [ ] 通过所有测试
- [ ] 更新相关文档
- [ ] 提交信息清晰

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
