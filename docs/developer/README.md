# Developer Guide

本目录为系统开发者和扩展者提供完整的架构文档和技术参考。

## 快速导航

### 我想...

| 需求 | 推荐阅读 |
|------|--------|
| **快速了解系统** | [ARCHITECTURE.md](ARCHITECTURE.md) - 简介和架构约束部分 |
| **全面理解架构** | [ARCHITECTURE.md](ARCHITECTURE.md) - 完整阅读 |
| **开发新控制模式** | [ARCHITECTURE.md#场景-1新增一种控制模式](ARCHITECTURE.md#场景-1新增一种控制模式) |
| **接入新硬件/总线** | [ARCHITECTURE.md#场景-2替换硬件驱动](ARCHITECTURE.md#场景-2替换硬件驱动) |
| **添加轨迹平滑算法** | [ARCHITECTURE.md#场景-3引入新的轨迹平滑算法](ARCHITECTURE.md#场景-3引入新的轨迹平滑算法) |
| **了解笛卡尔末端速度计算原理** | [algorithms/CARTESIAN_VELOCITY.md](algorithms/CARTESIAN_VELOCITY.md) |
| **了解重力补偿算法设计** | [algorithms/GRAVITY_COMPENSATION.md](algorithms/GRAVITY_COMPENSATION.md) |
| **了解系统中逆运动学实现** | [algorithms/INVERSE_KINEMATICS](algorithms/INVERSE_KINEMATICS.md) |
| **了解轨迹插值组件的原理** | [algorithms/TRAJECTORY_INTERPOLATION.md](algorithms/TRAJECTORY_INTERPOLATION.md) |


---

## 文档清单

### 核心架构

- **[ARCHITECTURE.md](ARCHITECTURE.md)** (711 行)
  - 系统整体设计理念与核心原则
  - 4 个结构单元与核心运行时组件
  - 3 条关键架构边界定义
  - 3 个典型扩展场景示例
  - 系统设计约束与集成责任

### 算法与技术参考

- **[算法模块架构说明：重力补偿子模块](algorithms/GRAVITY_COMPENSATION.md)**
  - 重力补偿算法原理
  - 动力学补偿方案

- **[algorithms/CARTESIAN_VELOCITY.md](algorithms/CARTESIAN_VELOCITY.md)**
  - 笛卡尔空间末端执行器速度控制

- **[algorithms/INVERSE_KINEMATICS.md](algorithms/INVERSE_KINEMATICS.md)**
  - 逆运动学算法实现

- **[algorithms/TRAJECTORY_INTERPOLATION.md](algorithms/TRAJECTORY_INTERPOLATION.md)**
  - 轨迹插值算法

---

## 完成标志

当你能够做到以下事项时，说明你已掌握本系统的核心：

- ✅ 解释系统的 3 条架构边界及其保护的目标
- ✅ 描述 4 个结构单元各自的职责与依赖关系
- ✅ 开发新的控制模式或硬件驱动
- ✅ 理解系统的设计约束与集成责任

---

**更多信息请访问 [文档中心](../README.md)。**
