# 代码规范

本文档定义 Arm Controller 项目的代码风格和编码规范。

## 📋 目录

- [通用规范](#通用规范)
- [命名规范](#命名规范)
- [代码格式](#代码格式)
- [注释规范](#注释规范)
- [最佳实践](#最佳实践)
- [错误处理](#错误处理)

---

## 通用规范

### 编程语言

- **C++**: C++17 标准
- **Python**: Python 3.8+
- **CMake**: 3.8+

### 编译器支持

- GCC 7.5+
- Clang 10+

### 代码原则

1. **可读性优先**: 代码应该易于理解
2. **一致性**: 遵循统一的风格
3. **简洁性**: 避免不必要的复杂性
4. **ROS2兼容**: 遵循 ROS2 编码标准

---

## 命名规范

### 类名

使用 **PascalCase**(大驼峰命名法):

```cpp
// ✅ 正确
class ControllerManager;
class MoveJController;
class HardwareManager;

// ❌ 错误
class controller_manager;  // 应该是 PascalCase
class moveJ_controller;    // 应该是 MoveJController
class hardwaremanager;     // 应该有分隔
```

### 函数名

使用 **camelCase**(小驼峰命名法):

```cpp
// ✅ 正确
void switchMode();
bool planJointMotion();
std::vector<double> getJointPositions();

// ❌ 错误
void SwitchMode();         // 应该小写开头
bool PlanJointMotion();
std::vector<double> get_joint_positions();  // 应该用驼峰
```

### 变量名

使用 **snake_case**(下划线命名法):

```cpp
// ✅ 正确
int joint_count;
std::string current_mode;
std::vector<double> joint_positions;

// ❌ 错误
int jointCount;            // 应该用下划线
std::string CurrentMode;   // 不应该大写
std::vector<double> JointPositions;  // 应该全小写
```

### 成员变量

使用 **下划线后缀**:

```cpp
class ControllerManager {
private:
    // ✅ 正确
    std::string current_mode_;
    int joint_count_;
    std::shared_ptr<HardwareManager> hardware_manager_;

    // ❌ 错误
    std::string current_mode;    // 缺少后缀
    int m_joint_count;           // 不使用 m_ 前缀
    std::shared_ptr<HardwareManager> _hardware_manager;  // 不使用前缀下划线
};
```

### 常量

使用 **k前缀 + PascalCase**:

```cpp
// ✅ 正确
const int kMaxRetries = 3;
const double kDefaultVelocityScaling = 0.2;
const std::string kDefaultPlanningGroup = "arm";

// ❌ 错误
const int MAX_RETRIES = 3;              // 应该用 k 前缀
const double default_velocity_scaling = 0.2;  // 应该用 PascalCase
```

### 枚举

枚举类型使用 **PascalCase**,枚举值使用 **UPPER_CASE**:

```cpp
// ✅ 正确
enum class PlanningStrategy {
    JOINT_SPACE,
    CARTESIAN_SPACE,
    SMART_CHOICE
};

enum class ControllerState {
    IDLE,
    RUNNING,
    ERROR
};

// ❌ 错误
enum class planning_strategy {  // 应该 PascalCase
    jointSpace,                  // 应该 UPPER_CASE
    cartesianSpace
};
```

### 命名空间

使用 **snake_case**,避免深层嵌套:

```cpp
// ✅ 正确
namespace arm_controller {
namespace controllers {
// ...
}
}

// ❌ 错误
namespace ArmController {  // 应该 snake_case
namespace Controllers {
namespace Trajectory {     // 太深了
// ...
}
}
}
```

### 文件名

使用 **snake_case**:

```
✅ 正确:
controller_manager.hpp
controller_manager.cpp
movej_controller.hpp
hardware_manager.hpp

❌ 错误:
ControllerManager.hpp      // 应该 snake_case
moveJController.cpp
HardwareManager.h          // 应该用 .hpp
```

---

## 代码格式

### 缩进

使用 **2个空格** 缩进,**不使用Tab**:

```cpp
// ✅ 正确
class ControllerManager {
public:
  void update() {
    if (current_controller_) {
      current_controller_->update();
    }
  }
};

// ❌ 错误 (使用了4个空格)
class ControllerManager {
public:
    void update() {
        if (current_controller_) {
            current_controller_->update();
        }
    }
};
```

### 行长度

- 每行不超过 **100字符**
- 超过时合理换行

```cpp
// ✅ 正确
auto result = planning_service_->planJointMotion(
    goal_state,
    velocity_scaling,
    acceleration_scaling);

// ❌ 错误 (超过100字符)
auto result = planning_service_->planJointMotion(goal_state, velocity_scaling, acceleration_scaling, timeout);
```

### 大括号

遵循 **K&R** 风格:

```cpp
// ✅ 正确
if (condition) {
  doSomething();
} else {
  doSomethingElse();
}

class MyClass {
public:
  void method() {
    // ...
  }
};

// ❌ 错误 (Allman风格)
if (condition)
{
  doSomething();
}
```

### 空格

```cpp
// ✅ 正确
int result = a + b * c;
func(arg1, arg2, arg3);
for (int i = 0; i < count; ++i) {
  // ...
}

// ❌ 错误
int result=a+b*c;
func( arg1,arg2,arg3 );
for(int i=0;i<count;++i){
  // ...
}
```

### 头文件包含顺序

```cpp
// 1. 本文件对应的头文件
#include "arm_controller/controller_manager.hpp"

// 2. C系统头文件
#include <cmath>
#include <cstring>

// 3. C++标准库
#include <memory>
#include <vector>
#include <string>

// 4. 第三方库
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

// 5. ROS2头文件
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// 6. 项目其他头文件
#include "arm_controller/controllers/movej_controller.hpp"
#include "arm_controller/hardware/hardware_manager.hpp"
```

### 头文件保护

使用 `#pragma once`:

```cpp
// ✅ 正确
#pragma once

#include <memory>

namespace arm_controller {
class ControllerManager {
  // ...
};
}

// ❌ 错误 (使用传统保护)
#ifndef ARM_CONTROLLER_CONTROLLER_MANAGER_HPP_
#define ARM_CONTROLLER_CONTROLLER_MANAGER_HPP_
// ...
#endif  // ARM_CONTROLLER_CONTROLLER_MANAGER_HPP_
```

---

## 注释规范

### 文件头注释

```cpp
/**
 * @file controller_manager.hpp
 * @brief 控制器管理器,负责控制器注册和模式切换
 * @author Ding Kaiyue
 * @date 2025-01-15
 */
```

### 类注释

使用 **Doxygen** 风格:

```cpp
/**
 * @brief 控制器管理器类
 *
 * 负责:
 * - 控制器的注册和管理
 * - 工作模式的切换
 * - 系统状态的监控
 *
 * @note 这是一个ROS2节点
 */
class ControllerManager : public rclcpp::Node {
  // ...
};
```

### 函数注释

```cpp
/**
 * @brief 切换控制模式
 *
 * @param mode 目标控制模式名称
 * @param mapping 机械臂映射名称 (single_arm/left_arm/right_arm)
 * @return true 切换成功
 * @return false 切换失败
 *
 * @note 切换过程会经过HoldState安全检查
 * @warning 只在机器人完全停止时调用
 */
bool switchMode(const std::string& mode, const std::string& mapping);
```

### 行内注释

```cpp
// ✅ 正确: 解释"为什么",而不是"做什么"
// 使用互斥锁保护,因为可能被多个线程访问
std::lock_guard<std::mutex> lock(mutex_);

// 等待100ms让电机完全停止
std::this_thread::sleep_for(std::chrono::milliseconds(100));

// ❌ 错误: 只是重复代码
// 加锁
std::lock_guard<std::mutex> lock(mutex_);

// 睡眠100毫秒
std::this_thread::sleep_for(std::chrono::milliseconds(100));
```

### TODO注释

```cpp
// TODO(kaiyue): 实现笛卡尔速度控制
// TODO(kaiyue): 添加双臂协同控制支持

// ❌ 错误
// TODO: 这个需要修复  // 没有负责人
```

---

## 最佳实践

### 使用智能指针

```cpp
// ✅ 正确
std::shared_ptr<HardwareManager> hardware_manager_;
std::unique_ptr<Trajectory> trajectory_;

// ❌ 错误
HardwareManager* hardware_manager_;  // 裸指针
Trajectory* trajectory_;
```

### 使用auto

```cpp
// ✅ 正确: 类型明显或者很长时使用auto
auto positions = hardware_manager_->getJointPositions(mapping_);
auto controller = std::make_shared<MoveJController>(hw_mgr, node);

// ❌ 错误: 类型不明确
auto result = calculate();  // 返回类型不清楚

// ✅ 更好: 显式类型
double result = calculate();
```

### 使用const

```cpp
// ✅ 正确: 尽可能使用const
void processJointStates(const std::vector<double>& positions) const;

const std::string& getCurrentMode() const;

// ❌ 错误: 应该但没有使用const
void processJointStates(std::vector<double>& positions);
std::string getCurrentMode();
```

### 使用nullptr

```cpp
// ✅ 正确
if (controller_ == nullptr) {
  // ...
}

// ❌ 错误
if (controller_ == NULL) {  // 使用C风格NULL
  // ...
}
```

### 使用范围for循环

```cpp
// ✅ 正确
for (const auto& position : positions) {
  processPosition(position);
}

// ❌ 错误: 不必要的索引
for (size_t i = 0; i < positions.size(); ++i) {
  processPosition(positions[i]);
}
```

### RAII原则

```cpp
// ✅ 正确: 使用RAII
{
  std::lock_guard<std::mutex> lock(mutex_);
  // 临界区代码
}  // 自动解锁

// ❌ 错误: 手动管理
mutex_.lock();
// 临界区代码
mutex_.unlock();  // 可能因异常而未执行
```

---

## 错误处理

### 使用异常

```cpp
// ✅ 正确: 对无法恢复的错误抛出异常
if (!hardware_manager_->initialize()) {
  throw std::runtime_error("Failed to initialize hardware manager");
}

// ✅ 正确: 对可恢复的错误返回错误码
bool planMotion() {
  if (!validateGoal()) {
    RCLCPP_ERROR(logger_, "Invalid goal");
    return false;
  }
  // ...
  return true;
}
```

### 日志记录

```cpp
// ✅ 正确: 使用适当的日志级别
RCLCPP_DEBUG(logger_, "Planning iteration %d", i);
RCLCPP_INFO(logger_, "Mode switched to %s", mode.c_str());
RCLCPP_WARN(logger_, "Approaching joint limits");
RCLCPP_ERROR(logger_, "Planning failed: %s", error.what());
RCLCPP_FATAL(logger_, "Hardware communication lost");

// ❌ 错误: 滥用ERROR级别
RCLCPP_ERROR(logger_, "Starting planning");  // 应该用INFO
```

### 断言

```cpp
// ✅ 正确: 检查程序逻辑错误
assert(joint_count_ == positions.size());
assert(controller_ != nullptr);

// ❌ 错误: 检查用户输入或外部条件
assert(goal.positions.size() == 6);  // 应该用if检查
```

---

## 代码审查清单

提交代码前检查:

- [ ] 遵循命名规范
- [ ] 代码格式正确(使用clang-format)
- [ ] 添加了必要的注释
- [ ] 公有接口有Doxygen注释
- [ ] 使用智能指针而非裸指针
- [ ] 适当使用const
- [ ] 错误处理完善
- [ ] 日志级别正确
- [ ] 通过编译器警告检查
- [ ] 通过ament_lint检查

---

## 工具配置

### clang-format

`.clang-format`:

```yaml
BasedOnStyle: Google
IndentWidth: 2
ColumnLimit: 100
AllowShortFunctionsOnASingleLine: Empty
AllowShortIfStatementsOnASingleLine: Never
```

使用:

```bash
clang-format -i src/**/*.cpp src/**/*.hpp
```

### ament_lint

```bash
# 运行所有检查
ament_lint_auto arm_controller

# 单独运行
ament_cpplint src/
ament_cppcheck src/
ament_uncrustify src/
```

---

## 相关文档

- [ROS2 C++ Style Guide](https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html)
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [开发者指南](DEVELOPER.md) - 开发流程
