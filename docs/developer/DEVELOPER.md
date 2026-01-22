# Universal Arm Controller - 开发者指南

本指南面向二次开发 / 系统扩展开发者。  
帮助你理解系统架构、扩展控制模式、添加硬件或优化模块。

---

## 1. 架构概览

系统分为三层：

1. **应用层**：ControllerManager + TrajectoryController  
2. **控制层**：Trajectory Planning + Trajectory Interpolator + CSAPS-CPP
3. **硬件层**：Hardware Driver (CAN-FD / EtherCAT)

---

## 2. 组件职责

| 组件 | 位置 | 职责 |
|------|------|------|
| ControllerManager | `src/arm_controller/` | 控制模式管理、状态监控、ROS2 接口 |
| TrajectoryController | `src/arm_controller/` | 轨迹执行、插值调用、动作服务 |
| Hardware Driver | `src/hardware_driver/` | CAN-FD / EtherCAT 通信、电机控制 |
| Trajectory Planning | `src/trajectory-planning/` | MoveIt2 路径规划、碰撞检测 |
| Trajectory Interpolator | `src/trajectory-interpolator/` | 实时轨迹生成、动力学约束 |
| CSAPS | `src/csaps/` | 轨迹点平滑 |

---

## 3. 数据流

### MoveJ 控制流程

用户命令 → MoveJ 控制器 → 轨迹规划 → 轨迹插值 → 硬件执行 → 状态反馈 → 用户

### 状态反馈流程

电机状态 → CAN-FD 接收 → Hardware Driver → 事件总线 → ROS2 Topic → 用户

---

## 4. 插件扩展点

- **控制模式**：通过工厂模式 + 插件注册宏扩展  
- **硬件驱动**：继承 BusInterface / MotorDriverInterface  
- **轨迹规划算法**：替换 Trajectory Planning 模块  
- **轨迹插值 / 平滑**：替换 Trajectory Interpolator 或 CSAPS

---

## 5. 开发流程

1. 克隆代码并建立工作空间  
2. 使用 `colcon build` 构建 ROS2 包  
3. 添加新的控制模式或硬件驱动实现  
4. 注册到 ControllerFactory 或 BusFactory  
5. 编写单元测试  
6. 提交合并请求（参考贡献指南）

---

## 6. 技术附录

### 性能指标

- 控制延迟 < 200 μs  
- 更新频率 500 Hz  
- 状态反馈延迟 < 5 ms  

### 代码规模

- Arm Controller 约 10,798 LOC  
- 控制模式 13+  
- 配置文件 5+ YAML 文件
