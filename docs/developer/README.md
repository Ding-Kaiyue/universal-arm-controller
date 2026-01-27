# Developer Guide

本目录为系统开发者和扩展者提供完整的架构文档和技术参考。

## 快速导航

| 概念 | 相关文档 |
|------|---------|
| 架构分层 | [ARCHITECTURE.md](ARCHITECTURE.md) |
| 示教轨迹平滑 | [algorithms/CSAPS.md](algorithms/CSAPS.md) |
| 轨迹时间参数化 | [algorithms/TOTG.md](algorithms/TOTG.md) |
| 轨迹密集采样 | [algorithms/TRAJECTORY_INTERPOLATION.md](algorithms/TRAJECTORY_INTERPOLATION.md) |
| 末端姿态求解 | [algorithms/INVERSE_KINEMATICS.md](algorithms/INVERSE_KINEMATICS.md) |
| 重力自适应控制 | [algorithms/GRAVITY_COMPENSATION.md](algorithms/GRAVITY_COMPENSATION.md) |
| 速度映射与解耦 | [algorithms/CARTESIAN_VELOCITY.md](algorithms/CARTESIAN_VELOCITY.md) |

---

## 文档结构

### 核心架构文档

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - 系统整体设计
  - 系统分层结构与核心组件
  - 4 个结构单元的职责与依赖
  - 3 条架构边界定义
  - 3 个典型扩展场景示例
  - 系统设计约束与集成责任

### 算法与技术参考

#### 轨迹处理层

- **[algorithms/CSAPS.md](algorithms/CSAPS.md)** - CSAPS 轨迹平滑
  - 轨迹平滑算法设计
  - 示教模式集成方式
  - 平滑参数调整机制

- **[algorithms/TOTG.md](algorithms/TOTG.md)** - 时间最优轨迹生成
  - 轨迹时间参数化
  - 速度/加速度约束处理
  - 多关节协调规划

- **[algorithms/TRAJECTORY_INTERPOLATION.md](algorithms/TRAJECTORY_INTERPOLATION.md)** - 轨迹插值
  - 密集采样与插值方法
  - 样条函数设计
  - 执行层数据适配

#### 运动规划与控制层

- **[algorithms/INVERSE_KINEMATICS.md](algorithms/INVERSE_KINEMATICS.md)** - 逆运动学求解
  - IK 算法实现选择
  - 拓扑连续性保证
  - TRAC-IK 配置说明

- **[algorithms/GRAVITY_COMPENSATION.md](algorithms/GRAVITY_COMPENSATION.md)** - 重力补偿
  - 动力学计算方法
  - 实时力矩补偿
  - Pinocchio 集成

#### 感知与执行层

- **[algorithms/CARTESIAN_VELOCITY.md](algorithms/CARTESIAN_VELOCITY.md)** - 笛卡尔末端速度控制
  - 末端执行器速度计算
  - 雅可比矩阵与微分运动学
  - 实时速度控制

---

## 学习路径

### 初级：理解架构（30分钟）
1. 阅读 [ARCHITECTURE.md](ARCHITECTURE.md) - 前 3 部分
2. 理解 4 个结构单元及 3 条架构边界
3. 了解系统的分层与职责划分

### 中级：掌握核心算法（2小时）
1. [CSAPS.md](algorithms/CSAPS.md) - 示教轨迹如何平滑处理
2. [TOTG.md](algorithms/TOTG.md) - 轨迹如何实现时间最优
3. [TRAJECTORY_INTERPOLATION.md](algorithms/TRAJECTORY_INTERPOLATION.md) - 轨迹如何从关键点生成密集采样

### 高级：实现新功能（根据需要）

#### 添加新控制模式
- 参考 [ARCHITECTURE.md](ARCHITECTURE.md) 场景 1
- 理解控制器与硬件管理器的边界
- 实现新的控制策略

#### 替换硬件或总线
- 参考 [ARCHITECTURE.md](ARCHITECTURE.md) 场景 2
- 理解硬件驱动的标准接口
- 实现硬件特定的驱动层

#### 引入新轨迹平滑/规划算法
- 参考 [ARCHITECTURE.md](ARCHITECTURE.md) 场景 3
- 研究 [CSAPS.md](algorithms/CSAPS.md) / [TOTG.md](algorithms/TOTG.md) 的设计模式
- 参考相应的算法文档实现新算法

---

## 掌握要点检查清单

当你能够做到以下事项时，说明你已掌握本系统的核心：

- ✅ 解释系统的 3 条架构边界及其保护的目标
- ✅ 描述 4 个结构单元各自的职责与依赖关系
- ✅ 说明 CSAPS、TOTG、IK 三个关键算法的作用
- ✅ 开发或扩展一个控制模式或硬件驱动
- ✅ 理解系统的设计约束与集成责任

---

**更多信息请访问 [文档中心](../README.md)。**
