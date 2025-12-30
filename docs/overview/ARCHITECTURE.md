# 系统架构

Universal Arm Controller 的总体架构设计与设计理念。

## 📋 目录

- [架构总览](#架构总览)
- [分层架构](#分层架构)
- [组件交互](#组件交互)
- [设计理念](#设计理念)

---

## 架构总览

Universal Arm Controller 采用**三层分层架构** + **模块化组件设计**：

![Universal Arm Controller Architecture](diagrams/architecture_overview.png)

---

## 分层架构

### 第一层：应用层

**负责**：用户交互、模式管理、系统状态

**核心组件**：

- **ControllerManager** - 控制器管理和模式切换
- **TrajectoryController** - 轨迹执行控制
- **ROS2 Interfaces** - 服务、话题、动作

**特点**：

- 提供统一的 ROS2 接口
- 隐藏下层复杂性
- 支持多种控制模式

---

### 第二层：控制层

**负责**：轨迹规划、路径生成、运动学计算

**核心库**：

- **Trajectory Planning** - 基于 MoveIt2 的规划
- **Trajectory Interpolator** - 实时轨迹插值

**功能**：

- 路径规划与碰撞检测
- 逆运动学求解
- 轨迹平滑与动力学约束满足

---

### 第三层：硬件层

**负责**：底层硬件通信、电机控制

**核心库**：

- **Hardware Driver** - CAN-FD 通信与电机驱动

**特点**：

- 高性能：微秒级延迟
- 线程安全：CPU 亲和性绑定
- 灵活：事件驱动 + 观察者模式

---

## 组件交互

### 交互流程图

<div align="center">

![Data Flow Diagram](diagrams/data_flow.png)

</div>

### 关键数据结构

1. **关节状态** - sensor_msgs/JointState
2. **任务指令** - geometry_msgs/Pose 或 sensor_msgs/JointState
3. **轨迹** - trajectory_msgs/JointTrajectory
4. **电机指令** - CAN-FD 格式的控制字

---

## 设计理念

### 1. 分离关注点

- **应用层**与**硬件层**隔离
- 便于独立测试和维护
- 支持多种硬件替换

### 2. 模块化

- 各组件可独立使用
- 清晰的接口定义
- 最小化依赖耦合

### 3. 实时性

- 低延迟设计
- 事件驱动架构
- 精确的时序控制

### 4. 可靠性

- 多层安全检查
- 限位保护机制
- 异常处理

### 5. 易用性

- 统一的 ROS2 接口
- 完整的文档
- 丰富的示例

---

## 工作流程示例

### MoveJ 控制流程

```text
1. 用户发送 MoveJ 目标关节角度
   ros2 topic pub --once /controller_api/movej_action/single_arm sensor_msgs/msg/JointState "{position: [pos1, pos2, pos3, pos4, pos5, pos6]}"

2. Arm Controller 接收并验证
   - 检查目标是否在关节限制内
   - 检查碰撞风险

3. 调用轨迹规划
   - MoveIt2 进行路径规划
   - 生成中间路径点

4. 轨迹插值
   - 在路径点间生成光滑轨迹
   - 满足速度/加速度约束

5. 硬件执行
   - 从轨迹中提取控制指令
   - 通过 CAN-FD 发送到电机

6. 状态反馈
   - 电机返回当前位置/速度
   - 发布 ROS2 Topics
   - 可选的事件触发
```

### 状态转换流程

<div align="center">

![Controller State Transition](diagrams/controller_state_transition.png)

</div>

---

## 性能考虑

### 延迟路径

1. **应用层延迟** - ROS2 通信（1-2 ms）
2. **规划延迟** - 轨迹规划（50-500 ms）
3. **插值延迟** - 轨迹生成（< 1 ms）
4. **硬件延迟** - CAN 通信 + 电机响应（200 μs）

**总延迟** - 规划主导（通常 < 1 s）

### 优化策略

- 规划的结果缓存
- 异步规划执行
- 优先级队列管理

---

## 可扩展性

### 支持的扩展

1. **新硬件** - 替换 Hardware Driver
2. **新规划算法** - 扩展 Trajectory Planning
3. **新控制模式** - 添加新的 Controller
4. **新的传感器** - 扩展 Feedback 系统
5. **新的插值方式** - 扩展 Trajectory Interpolator
6. **新的轨迹平滑方式** - 替换 csaps

### 设计原则

- 接口驱动设计
- 插件式架构
- 配置驱动行为

---

## 下一步

- 📖 查看 [Arm Controller 架构](../src/arm_controller/docs/ARCHITECTURE.md) 了解详细设计
- ⚙️ 参考 [配置指南](../src/arm_controller/docs/CONFIGURATION.md)
- 👨‍💻 查看 [开发指南](../src/arm_controller/docs/DEVELOPER.md)

---

**更多信息请访问 [文档中心](README.md)。**
