# Universal Arm Controller - 技术概览

本文档面向技术评估 / 决策者，快速了解系统能力、性能与扩展性。

---

## 项目定位

- 通用机械臂控制系统  
- 支持单臂 / 双臂、多种控制模式  
- 高实时性、工业级可扩展架构

---

## 总体架构

用户应用
↓
ControllerManager + TrajectoryController
↓
Trajectory Planning + Trajectory Interpolator
↓
Hardware Driver (CAN-FD / EtherCAT)
↓
机械臂执行器


---

## 关键技术点

1. **模块化设计**：应用层 / 控制层 / 硬件层分离  
2. **高实时性**：微秒级控制延迟  
3. **多控制模式支持**：MoveJ、MoveL、MoveC、JointVelocity、CartesianVelocity  
4. **可扩展硬件**：替换驱动总线或添加新电机  
5. **轨迹平滑与动力学约束**：CSAPS + Pinocchio

---

## 性能指标

| 指标 | 数值 |
|------|------|
| 控制延迟 | < 200 μs |
| 更新频率 | 500 Hz |
| 状态反馈延迟 | < 5 ms |
| 支持电机数 | 数百个 |
| 内存占用 | < 50 MB |
| CPU 使用率 | < 5% (Jetson Orin) |

---

## 典型应用场景

- 高精度点到点操作  
- 复杂轨迹示教与重放  
- 双臂协同作业  
- 离线程序开发与快速原型

---

## 可扩展性与未来路线

- 添加新控制模式  
- 支持新硬件驱动或总线  
- 扩展规划算法或插值方法  
- 集成新传感器与反馈机制
