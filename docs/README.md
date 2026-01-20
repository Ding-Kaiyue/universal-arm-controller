# Universal Arm Controller - 文档中心

欢迎来到 Universal Arm Controller 的文档中心！这里包含了项目的所有技术文档、快速开始指南、架构设计和故障排除信息。

## 完整文档列表

### 快速上手

- **[README](../README.md)** - 项目简介与快速安装
- **[本地安装指南](getting_started/INSTALLATION.md)** - 详细安装步骤与前置准备

### 用户文档

- **[控制命令使用指南](getting_started/CONTROLLERS.md)** - 所有控制模式与使用方法
- **[故障排除](getting_started/TROUBLESHOOTING.md)** - 常见问题与解决方案

### 开发者文档
- **[系统架构与组件](overview/ARCHITECTURE.md)** - 系统组件、架构设计与设计理念
- **[开发指南](../.github/CONTRIBUTING.md)** - 贡献指南与开发流程

### 组件文档

#### 本仓库维护的组件

- **[Arm Controller 文档中心](../src/arm_controller/docs/README.md)** - 运动控制核心详细文档
  - 包含: 控制器详解、配置指南、安全机制、代码规范等
  - **13+ 控制模式**: MoveJ、MoveL、MoveC、JointVelocity、CartesianVelocity、PointRecord、PointReplay、TrajectoryRecord、TrajectoryReplay 等
  - **关键特性**: 全 6D 方向控制、双臂协同、动态速度缩放
  - **高级功能**: 重力补偿、轨迹录制回放、点位录制回放

#### VCS 导入的依赖组件

- **[Hardware Driver](https://github.com/Ding-Kaiyue/hardware-driver#readme)** - CAN-FD 硬件驱动库
  - 功能: CAN-FD 高速通信(5000 kbit/s)、实时电机控制、事件驱动监控
  - 特性: <200μs 控制延迟、500Hz 更新频率、线程安全

- **[Trajectory Interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator#readme)** - 轨迹插值库
  - 功能: 样条曲线插值、动力学约束满足、实时轨迹生成
  - 特性: 光滑轨迹生成、约束自适应

- **[Trajectory Planning](https://github.com/Ding-Kaiyue/trajectory-planning#readme)** - 轨迹规划库
  - 功能: MoveIt2 集成、TracIK 逆运动学求解、碰撞检测与避障
  - 特性: 多种规划策略、快速规划、支持双臂协同规划

- **[CSAPS 库](https://github.com/Ding-Kaiyue/csaps)** - C++ 样条曲线库
  - 功能: 用于轨迹平滑、录制轨迹后处理
  - 特性: 高效曲线拟合、光滑输出

---

## 外部资源

### 官方文档与参考

- **[ROS2 Humble](https://docs.ros.org/en/humble/)** - ROS2 官方文档
- **[MoveIt 2](https://moveit.picknik.ai/humble/)** - MoveIt 官方教程

### 数学与优化

- **[eigenpy](https://github.com/stack-of-tasks/eigenpy)** - Eigen 的 Python 绑定
- **[OSQP](https://github.com/osqp/osqp)** - 二次规划求解器
- **[OsqpEigen](https://github.com/gbionics/osqp-eigen.git)** - OSQP 的 C++ 包装
- **[NLopt](https://github.com/stevengj/nlopt)** - 非线性优化库

### 机器人学

- **[Pinocchio](https://github.com/stack-of-tasks/pinocchio)** - 刚体动力学库
- **[TracIK](https://github.com/aprotyas/trac_ik)** - 逆运动学求解器

### Python 包

- **[qdldl](https://pypi.org/project/qdldl/)** - 二次规划求解器的 Python 包，为 **[Pinocchio](https://github.com/stack-of-tasks/pinocchio)** 的依赖库

### 社区

- **[Bug 报告](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=bug_report.md)** - 报告使用中的 Bug
- **[安装问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=installation_issue.md)** - 报告安装过程中的问题
- **[功能请求](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=feature_request.md)** - 请求新功能或改进
- **[使用问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=usage_question.md)** - 询问库的使用方法
- **[安全漏洞报告](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=security_report.md)** - 报告安全漏洞

---

**最后更新**: 2026年1月20日 | **维护者**: [Ding-Kaiyue](https://github.com/Ding-Kaiyue)
