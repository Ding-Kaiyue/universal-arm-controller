# 系统架构

本文档详细介绍 Arm Controller 的系统架构设计。

## 📋 目录

- [架构概览](#架构概览)
- [双节点架构](#双节点架构)
- [核心组件](#核心组件)
- [控制器架构](#控制器架构)
- [依赖组件集成](#依赖组件集成)
- [数据流](#数据流)
- [设计模式](#设计模式)

---

## 架构概览

### 系统架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                      Universal Arm Controller                       │
│                                                                     │
│  ┌──────────────────────────┐    ┌──────────────────────────┐       │
│  │   ControllerManager      │    │   TrajectoryController   │       │
│  │   (状态管理节点)          │    │   (轨迹执行节点)          │       │
│  │                          │    │                          │       │
│  │  • WorkMode Service      │    │  • Action Server         │       │
│  │  • Controller Registry   │    │  • MoveIt Integration    │       │
│  │  • Mode Switching        │    │  • Trajectory Execution  │       │
│  │  • Safety Monitoring     │    │  • Interpolation         │       │
│  └──────────┬───────────────┘    └───────────┬──────────────┘       │
│             │                                │                      │
│             └────────────┬────────────────────┘                      │
│                          │                                          │
│              ┌───────────▼──────────┐                                │
│              │  HardwareManager     │                                │
│              │    (Singleton)       │                                │
│              │                      │                                │
│              │  • Motor Control     │                                │
│              │  • State Feedback    │                                │
│              │  • CAN-FD Protocol   │                                │
│              └───────────┬──────────┘                                │
└──────────────────────────┼─────────────────────────────────────────┘
                           │
                    ┌──────▼───────┐
                    │  CAN-FD Bus  │
                    └──────┬───────┘
                           │
                ┌──────────┴──────────┐
                │                     │
           ┌────▼─────┐         ┌────▼─────┐
           │ Motor 1  │   ...   │ Motor N  │
           └──────────┘         └──────────┘
```

### 设计理念

Arm Controller 基于以下设计理念:

1. **职责分离**: 将状态管理和轨迹执行分离到不同节点
2. **模块化**: 控制器采用插件式架构,易于扩展
3. **线程安全**: 关键组件采用单例模式和互斥锁保护
4. **实时性**: 硬件通信和控制循环独立线程运行
5. **安全性**: 多层安全检查,钩子状态机制

---

## 双节点架构

### 节点职责

#### 1. ControllerManager (控制管理节点)

**主要职责**:
- 控制器注册和管理
- 工作模式切换
- 安全状态监控
- 系统状态发布

**关键接口**:
```cpp
// 服务
/controller_api/controller_mode (controller_interfaces/srv/WorkMode)

// 话题
/controller_api/running_status (std_msgs/msg/String)
/controller_api/movej_action (sensor_msgs/msg/JointState)
/controller_api/movel_action (geometry_msgs/msg/Pose)
/controller_api/movec_action (geometry_msgs/msg/PoseArray)
/controller_api/joint_velocity_action (sensor_msgs/msg/JointState)
```

**核心组件**:
```cpp
class ControllerManager {
private:
    std::map<std::string, std::shared_ptr<IController>> controllers_;
    std::shared_ptr<HardwareManager> hardware_manager_;
    std::string current_mode_;
    std::string current_mapping_;

public:
    void registerController(const std::string& name,
                           std::shared_ptr<IController> controller);
    bool switchMode(const std::string& mode,
                   const std::string& mapping);
    void update();  // 定时调用当前控制器的update()
};
```

#### 2. TrajectoryController (轨迹控制节点)

**主要职责**:
- 接收 MoveIt 轨迹执行请求
- 轨迹插值和优化
- 实时轨迹跟踪
- 执行状态反馈

**关键接口**:
```cpp
// Action Server
/arm_controller/follow_joint_trajectory
  (control_msgs/action/FollowJointTrajectory)

// 话题
/joint_states (sensor_msgs/msg/JointState)
```

**核心组件**:
```cpp
class TrajectoryController {
private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    std::shared_ptr<TrajectoryInterpolator> interpolator_;
    std::shared_ptr<HardwareManager> hardware_manager_;

public:
    void executeTrajectory(const Trajectory& trajectory);
    void handleGoal(const GoalHandle& goal_handle);
    void handleCancel(const GoalHandle& goal_handle);
};
```

### 节点通信

```
用户/MoveIt
    │
    ├──► ControllerManager (WorkMode Service)
    │         │
    │         ├──► HardwareManager (Motor Control)
    │         └──► 控制器切换
    │
    └──► TrajectoryController (Action Server)
              │
              ├──► HardwareManager (Trajectory Execution)
              └──► 状态反馈
```

### 并行执行

两个节点在同一进程中并行运行:

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建执行器
    rclcpp::executors::MultiThreadedExecutor executor;

    // 创建节点
    auto controller_manager = std::make_shared<ControllerManager>();
    auto trajectory_controller = std::make_shared<TrajectoryController>();

    // 添加到执行器
    executor.add_node(controller_manager);
    executor.add_node(trajectory_controller);

    // 并行执行
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
```

---

## 核心组件

### HardwareManager (单例模式)

**职责**:
- 管理所有电机的通信
- 封装硬件驱动库接口
- 提供线程安全的访问接口

**实现**:
```cpp
class HardwareManager {
private:
    static std::shared_ptr<HardwareManager> instance_;
    static std::mutex instance_mutex_;

    std::shared_ptr<hardware_driver::RobotHardware> robot_hardware_;
    std::map<std::string, HardwareConfig> hardware_configs_;

    HardwareManager();  // 私有构造

public:
    static std::shared_ptr<HardwareManager> getInstance();

    void controlMotor(const std::string& mapping,
                     const std::vector<double>& positions);
    void controlMotorVelocity(const std::string& mapping,
                             const std::vector<double>& velocities);
    std::vector<double> getJointPositions(const std::string& mapping);
    std::vector<double> getJointVelocities(const std::string& mapping);
};
```

**单例保证**:
```cpp
std::shared_ptr<HardwareManager> HardwareManager::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = std::shared_ptr<HardwareManager>(new HardwareManager());
    }
    return instance_;
}
```

### ControllerFactory

**职责**:
- 创建和注册所有控制器
- 提供控制器依赖注入

**实现**:
```cpp
class ControllerFactory {
public:
    static void registerAllControllers(
        ControllerManager& manager,
        const std::shared_ptr<HardwareManager>& hardware_manager,
        const std::shared_ptr<rclcpp::Node>& node) {

        // 轨迹控制器
        auto movej = std::make_shared<MoveJController>(
            hardware_manager, node);
        manager.registerController("MoveJ", movej);

        auto movel = std::make_shared<MoveLController>(
            hardware_manager, node);
        manager.registerController("MoveL", movel);

        // ... 注册其他控制器
    }
};
```

---

## 控制器架构

### 控制器接口

所有控制器实现统一接口:

```cpp
class IController {
public:
    virtual ~IController() = default;

    // 控制器生命周期
    virtual void onEnter(const std::string& mapping) = 0;
    virtual void onExit() = 0;
    virtual void update() = 0;

    // 钩子状态
    virtual bool requiresHookState() const { return true; }
    virtual HookStrategy getHookStrategy() const {
        return HookStrategy::POSITION_HOLD;
    }
};
```

### 控制器分类

#### 1. 轨迹控制器基类

```cpp
class TrajectoryControllerBase : public IController {
protected:
    std::shared_ptr<HardwareManager> hardware_manager_;
    std::shared_ptr<trajectory_planning::MotionPlanningService> planning_service_;
    std::shared_ptr<trajectory_interpolator::TrajectoryInterpolator> interpolator_;

    virtual void planTrajectory() = 0;
    virtual void executeTrajectory() = 0;
};
```

#### 2. 速度控制器基类

```cpp
class VelocityControllerBase : public IController {
protected:
    std::shared_ptr<HardwareManager> hardware_manager_;
    std::vector<double> velocity_limits_;
    std::vector<double> position_limits_lower_;
    std::vector<double> position_limits_upper_;

    virtual bool checkSafety(const std::vector<double>& velocities);
    virtual void applyVelocity(const std::vector<double>& velocities);
};
```

### 状态机设计

```
          ┌─────────────┐
          │   Disable   │
          └──────┬──────┘
                 │
                 ▼
          ┌─────────────┐
          │  HoldState  │◄────────┐
          └──────┬──────┘         │
                 │                │
                 ▼                │
          ┌─────────────┐         │
          │   MoveJ     │─────────┤
          └─────────────┘         │
                                  │
          ┌─────────────┐         │
          │   MoveL     │─────────┤
          └─────────────┘         │
                                  │
          ┌─────────────┐         │
          │   MoveC     │─────────┤
          └─────────────┘         │
                                  │
          ┌─────────────┐         │
          │JointVelocity│─────────┘
          └─────────────┘
```

---

## 依赖组件集成

### 1. trajectory_planning 集成

```cpp
// 创建规划服务
auto moveit_adapter = std::make_shared<MoveItAdapter>(node);
auto planning_service = std::make_shared<MotionPlanningService>(
    movej_strategy, movel_strategy, movec_strategy,
    moveit_adapter, logger);

// 在控制器中使用
auto result = planning_service->planJointMotion(goal_state);
if (result.success) {
    executeTrajectory(result.trajectory);
}
```

**集成点**:
- MoveJ 控制器: 使用 `planJointMotion()`
- MoveL 控制器: 使用 `planLinearMotion()`
- MoveC 控制器: 使用 `planArcMotion()`

### 2. trajectory_interpolator 集成

```cpp
// 创建插值器
auto interpolator = std::make_shared<TrajectoryInterpolator>();

// 配置插值参数
SplineConfig config;
config.dt = 0.01;
config.spline_type = SplineConfig::CUBIC_SPLINE;
config.boundary_condition = BoundaryCondition::SECOND_DERIVATIVE;
interpolator->setInterpolationConfig(config);

// 插值轨迹
if (interpolator->loadTrajectory(trajectory)) {
    auto interpolated = interpolator->interpolate(0.01);
    executeInterpolatedTrajectory(interpolated);
}
```

**集成点**:
- TrajectoryController: 接收 MoveIt 轨迹后插值
- 所有轨迹控制器: 规划完成后插值优化

### 3. hardware_driver 集成

```cpp
// 初始化硬件栈
std::vector<std::string> interfaces = {"can0"};
std::map<std::string, std::vector<uint32_t>> motor_config = {
    {"can0", {1, 2, 3, 4, 5, 6}}
};

auto bus = std::make_shared<hardware_driver::bus::CanFdBus>(interfaces);
auto motor_driver = std::make_shared<hardware_driver::motor_driver::MotorDriverImpl>(bus);
auto robot_hardware = std::make_shared<hardware_driver::RobotHardware>(
    motor_driver, motor_config);

// 控制电机
robot_hardware->enable_motor("can0", 1, 4);
robot_hardware->control_motor_in_position_mode("can0", 1, 90.0f);
```

**集成点**:
- HardwareManager: 封装所有硬件通信
- 所有控制器: 通过 HardwareManager 访问硬件

---

## 数据流

### 位置控制数据流

```
用户指令 → MoveJ Controller
           ↓
    MoveIt 规划 (trajectory_planning)
           ↓
    轨迹插值 (trajectory_interpolator)
           ↓
    HardwareManager
           ↓
    Hardware Driver (hardware_driver)
           ↓
    CAN-FD → 电机
```

### 速度控制数据流

```
用户指令 → JointVelocity Controller
           ↓
    安全检查 (限位、急停)
           ↓
    HardwareManager
           ↓
    Hardware Driver
           ↓
    CAN-FD → 电机
```

### 状态反馈数据流

```
电机 → CAN-FD
    ↓
Hardware Driver (Observer Pattern)
    ↓
HardwareManager (缓存状态)
    ↓
ControllerManager / TrajectoryController
    ↓
/joint_states 话题
    ↓
用户 / MoveIt
```

---

## 设计模式

### 1. 单例模式 (Singleton)

**应用**: HardwareManager

**目的**: 确保全局唯一的硬件访问点

### 2. 策略模式 (Strategy)

**应用**: 控制器架构

**目的**: 运行时切换不同的控制策略

```cpp
class ControllerManager {
private:
    std::shared_ptr<IController> current_controller_;

public:
    void switchController(std::shared_ptr<IController> new_controller) {
        if (current_controller_) {
            current_controller_->onExit();
        }
        current_controller_ = new_controller;
        current_controller_->onEnter(mapping_);
    }
};
```

### 3. 观察者模式 (Observer)

**应用**: 硬件状态更新

**目的**: 实时接收电机状态变化

```cpp
class MotorStatusObserver {
public:
    virtual void on_motor_status_update(
        const std::string& interface,
        uint32_t motor_id,
        const Motor_Status& status) = 0;
};
```

### 4. 工厂模式 (Factory)

**应用**: ControllerFactory

**目的**: 统一创建和配置控制器

### 5. 状态模式 (State)

**应用**: HoldState 安全钩子

**目的**: 封装模式切换时的状态转换

---

## 线程模型

```
Main Thread
  │
  ├──► ControllerManager Node (ROS2 Executor)
  │     │
  │     └──► Timer Callback (10Hz - 控制器 update)
  │
  ├──► TrajectoryController Node (ROS2 Executor)
  │     │
  │     └──► Action Server Callbacks
  │
  └──► HardwareManager
        │
        ├──► CAN 接收线程 (hardware_driver)
        │
        └──► CAN 发送线程 (hardware_driver)
```

**线程安全保证**:
- HardwareManager: 单例 + 互斥锁
- 控制器切换: 互斥锁保护
- 硬件通信: hardware_driver 内部保证

---

## 配置管理

```
config/
  ├── hardware_config.yaml      # 硬件配置
  ├── config.yaml               # 通用配置
  └── controllers/
      ├── movej_config.yaml
      ├── movel_config.yaml
      └── ...
```

**加载流程**:
```cpp
// 1. 加载 YAML
YAML::Node config = YAML::LoadFile("config/hardware_config.yaml");

// 2. 解析配置
HardwareConfig hw_config;
hw_config.robot_type = config["robot_type"].as<std::string>();
hw_config.interface = config["interface"].as<std::string>();
// ...

// 3. 初始化组件
hardware_manager->initialize(hw_config);
```

---

## 性能考虑

### 1. 实时性保证

- **控制频率**: 10Hz (ControllerManager)
- **硬件通信**: < 1ms (hardware_driver)
- **规划超时**: 5s (可配置)

### 2. 资源占用

- **内存**: < 200MB
- **CPU**: < 10% (单核)
- **网络**: 仅 ROS2 本地通信

### 3. 优化策略

- 轨迹预计算和缓存
- 硬件状态批量读取
- 关键路径无动态内存分配

---

## 扩展性

### 添加新控制器

1. 继承 `IController` 接口
2. 实现 `onEnter()`, `onExit()`, `update()`
3. 在 `ControllerFactory` 中注册
4. 添加对应的话题订阅

### 添加新硬件

1. 实现 hardware_driver 接口
2. 在 HardwareManager 中集成
3. 更新 hardware_config.yaml

### 添加新规划策略

1. 在 trajectory_planning 中实现
2. 在控制器中调用新策略
3. 添加配置参数

---

## 下一步

- 查看 [开发者指南](DEVELOPER.md) 了解开发流程
- 查看 [控制器详解](CONTROLLERS.md) 了解具体实现
- 查看 [安全机制](SAFETY.md) 了解安全设计
