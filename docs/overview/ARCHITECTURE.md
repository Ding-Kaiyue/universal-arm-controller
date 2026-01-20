# 系统架构与组件

Universal Arm Controller 系统组件、架构设计与设计理念。

## 📋 目录

- [系统概览](#系统概览)
- [核心组件](#核心组件)
- [依赖库](#依赖库)
- [分层架构](#分层架构)
- [组件交互](#组件交互)
- [设计理念](#设计理念)
- [数据流](#数据流)
- [性能指标](#性能指标)

---

## 系统概览

Universal Arm Controller 是一个完整的机械臂控制系统解决方案，采用模块化架构，由多个独立组件组成。整个系统分为三层：

1. **应用层** - ROS2 节点和控制器
2. **控制层** - 轨迹规划和插值
3. **硬件层** - CAN-FD 通信和电机驱动

---

## 核心组件

### 本仓库维护的组件

#### 1. Arm Controller（运动控制核心）

**位置**: `src/arm_controller/`

运动控制系统的核心组件，负责：

- ✅ 多模式控制（MoveJ、MoveL、MoveC、JointVelocity）
- ✅ 状态管理与模式切换
- ✅ 安全监控与限位保护
- ✅ ROS2 接口与服务
- ✅ MoveIt2 集成

**特性**:

- 双节点架构：ControllerManager + TrajectoryController
- 原生双臂支持
- 事件驱动的状态监控
- 微秒级控制延迟

**文档**: [Arm Controller 文档中心](../../src/arm_controller/docs/README.md)

#### 2. Controller Interfaces（ROS2 消息定义）

**位置**: `src/controller_interfaces/`

定义系统中使用的所有 ROS2 消息和服务：

- 工作模式切换服务
- 关节状态消息
- 控制命令消息
- 系统状态消息

#### 3. Robotic Arm Bringup（系统启动）

**位置**: `src/robotic_arm_bringup/`

系统启动和配置：

- ROS2 启动文件
- YAML 配置文件
- 参数管理

---

## 依赖库

### Hardware Driver（CAN-FD 硬件驱动）

**GitHub**: [Ding-Kaiyue/hardware-driver](https://github.com/Ding-Kaiyue/hardware-driver)

提供硬件级别的电机控制能力：

- CAN-FD 高速通信（支持 CAN 2.0 和 CAN-FD）
- 实时电机控制（位置、速度、力矩、MIT 模式）
- 事件驱动的状态监控
- 观察者模式与事件总线
- 线程安全设计
- 微秒级控制延迟

**关键特性**:

- 支持多个电机并发控制
- CPU 亲和性绑定
- 背压控制机制

---

### Trajectory Interpolator（轨迹插值库）

**GitHub**: [Ding-Kaiyue/trajectory-interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator)

提供实时的轨迹插值能力：

- 样条曲线插值（B-spline、Bezier）
- 动力学约束满足（速度、加速度、加加速度）
- 实时轨迹生成
- 运动平滑处理

**应用场景**:

- 从规划的路径生成光滑的执行轨迹
- 满足机械臂的动力学限制
- 实时生成控制指令

---

### Trajectory Planning（轨迹规划库）

**GitHub**: [Ding-Kaiyue/trajectory-planning](https://github.com/Ding-Kaiyue/trajectory-planning)

基于 MoveIt2 的轨迹规划能力：

- 多种规划算法集成（RRT、RRTConnect 等）
- 碰撞检测与避障
- 逆运动学求解（通过 TracIK）
- 路径优化

**应用场景**:

- MoveJ 和 MoveL 控制的路径规划
- 碰撞检测和避障
- IK 求解

---

## 分层架构

### 整体架构图

```
┌─────────────────────────── Universal Arm Controller ──────────────────────┐
│                                                                           │
│  ┌──────────────────────────┐         ┌──────────────────────────┐        │
│  │  Arm Controller Nodes    │         │  User Applications       │        │
│  │ (ControllerManager +     │◄────────│  - ROS2 Nodes           │         │
│  │  TrajectoryController)   │         │  - Python Scripts       │         │
│  └──────────────┬───────────┘         └──────────────────────────┘        │
│                 │                                                         │
│                 │ (Motion Commands & Feedback)                            │
│                 ▼                                                         │
│  ┌──────────────────────────┐         ┌──────────────────────────┐        │
│  │ Trajectory Planning      │         │ Trajectory Interpolator  │        │
│  │ (MoveIt2 + TracIK)       │         │ (Spline + Dynamics)      │        │
│  └──────────────┬───────────┘         └──────────────┬───────────┘        │
│                 │                                    │                    │
│                 └────────────┬───────────────────────┘                    │
│                              │                                            │
│                    ┌─────────▼────────┐                                   │
│                    │ Hardware Manager │                                   │
│                    │ (Unified Driver) │                                   │
│                    └─────────┬────────┘                                   │
│                              │                                            │
│                    ┌─────────▼────────┐                                   │
│                    │  CAN-FD Bus      │                                   │
│                    │  (Communication) │                                   │
│                    └─────────┬────────┘                                   │
└────────────────────────────────┼──────────────────────────────────────────┘
                                 │
                        ┌────────▼────────┐
                        │    Hardware     │
                        │  (Motors/Arm)   │
                        └─────────────────┘
```

### 分层设计详解

#### 第一层：应用层

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

#### 第二层：控制层

**负责**：轨迹规划、路径生成、运动学计算

**核心库**：

- **Trajectory Planning** - 基于 MoveIt2 的规划
- **Trajectory Interpolator** - 实时轨迹插值

**功能**：

- 路径规划与碰撞检测
- 逆运动学求解
- 轨迹平滑与动力学约束满足

---

#### 第三层：硬件层

**负责**：底层硬件通信、电机控制

**核心库**：

- **Hardware Driver** - CAN-FD 通信与电机驱动

**特点**：

- 高性能：微秒级延迟
- 线程安全：CPU 亲和性绑定
- 灵活：事件驱动 + 观察者模式

---

## 组件交互

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

## 数据流

### 1. MoveJ 命令流

```
用户输入
  ↓
MoveJ 控制器
  ↓
轨迹规划 (MoveIt2)  ← 碰撞检测
  ↓
轨迹插值生成
  ↓
硬件驱动
  ↓
电机执行
  ↓
状态反馈
  ↓
用户反馈
```

### 2. 状态反馈流

```
电机状态
  ↓
CAN-FD 接收
  ↓
硬件驱动处理
  ↓
事件总线/观察者
  ↓
用户应用
```

### 3. MoveJ 控制流程详解

```
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

---

## 通信方式

### ROS2 接口

**服务**:

- 模式切换: `/controller_api/controller_mode`
- 系统状态查询

**话题**:

- 关节状态: `/joint_states`
- 控制命令: `/controller_api/*_action`
- 系统状态: `/controller_api/running_status`

### CAN-FD 协议

- 波特率: 5000 kbit/s (CAN-FD)
- 帧格式: 扩展 CAN 帧
- 实时性: 微秒级延迟

---

## 性能指标

### 实时性能

| 指标 | 数值 |
|------|------|
| **控制延迟** | < 200 μs |
| **更新频率** | 500 Hz |
| **CAN-FD 波特率** | 5000 kbit/s |
| **状态反馈延迟** | < 5 ms |

### 硬件支持

| 项目 | 数值 |
|------|------|
| **支持电机数** | 数百个 |
| **关节限位配置** | 动态配置 |
| **内存占用** | < 50 MB |
| **CPU 使用率** | < 5% (Jetson Orin) |

### 代码规模

| 项目 | 数值 |
|------|------|
| **Arm Controller 代码** | 10,798 LOC |
| **源文件总数** | 462+ 文件 |
| **控制模式数** | 13+ 种 |
| **配置文件** | 5+ YAML 文件 |
| **文档文件** | 15+ Markdown 文件 |

### 关键特性

| 特性 | 描述 |
|------|------|
| **13+ 控制模式** | MoveJ、MoveL、MoveC、JointVelocity、CartesianVelocity 等 |
| **全 6D 方向控制** | 完整的末端执行器位姿控制 |
| **双臂协同** | 原生支持 single_arm, left_arm, right_arm 映射 |
| **动态速度缩放** | MoveJ/MoveL/MoveC 运动中无需重规划即可调速 |
| **重力补偿** | 使用 Pinocchio 库的动力学补偿 |
| **轨迹平滑** | CSAPS 自适应平滑，特别适合录制轨迹 |
| **多层安全** | HoldState 钩子、限位保护、硬件监控 |

### 延迟路径分析

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

- 📖 查看 [Arm Controller 架构](../../src/arm_controller/docs/ARCHITECTURE.md) 了解详细设计
- ⚙️ 参考 [配置指南](../../src/arm_controller/docs/CONFIGURATION.md)
- 👨‍💻 查看 [开发指南](../../src/arm_controller/docs/DEVELOPER.md)

---

**更多信息请访问 [文档中心](../README.md)。**
