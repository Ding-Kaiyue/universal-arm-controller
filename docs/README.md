# Universal Arm Controller - 文档中心

本页面是 Universal Arm Controller 的唯一官方文档入口。

如果你是第一次使用本项目，请严格按照“推荐阅读顺序”阅读。

## 推荐阅读顺序

> 以下 4 篇文档覆盖了 90% 用户从 “零” 到 “稳定使用”的全部路径。

**1. [README](../README.md)**
   项目概览、支持能力与快速开始。

**2. [安装指南](getting_started/INSTALLATION.md)**
   环境准备、依赖安装、源码构建。

**3. [控制命令使用指南](getting_started/CONTROLLERS.md)**
   所有控制模式的使用方法与示例命令。

**4. [故障排除](getting_started/TROUBLESHOOTING.md)**
   编译失败、启动异常、机械臂不动等高频问题。

> 如果你已经完成以上 4 篇的阅读与实践，说明你已具备**正常使用本系统的全部能力。**
---
## 面向不同读者的文档指引

为避免信息过载，文档按照读者类型分层组织。

### 普通用户（使用机械臂）

你只需要阅读：

- [README](../README.md)
- [INSTALLATION](getting_started/INSTALLATION.md)
- [CONTROLLERS](getting_started/CONTROLLERS.md)
- [TROUBLESHOOTING](getting_started/TROUBLESHOOTING.md)

其余文档**可以全部跳过。**

### 系统集成者（集成到自己系统）

在完成“新用户必读”后，建议额外阅读：

- **[系统架构与组件](overview/ARCHITECTURE.md)**
  了解系统分层、组件关系与数据流。
- **[Arm Controller 文档中心](../src/arm_controller/docs/README.md)**
  了解控制核心的配置方式与高级功能。

### 开发者（二次开发/扩展控制模式）

在具备系统使用经验后，建议额外阅读：

- **[系统架构与组件](overview/ARCHITECTURE.md)**
- **[Arm Controller 文档中心](../src/arm_controller/docs/README.md)**
- **[贡献指南](../.github/CONTRIBUTING.md)**

> [!NOTE]
> 以下文档面向系统集成者与开发者。
> 如果你只是使用本系统控制机械臂，可以跳过本部分。

---

## 核心组件文档

### 本仓库维护的组件

- **[Arm Controller 文档中心](../src/arm_controller/docs/README.md)** 
  运动控制核心详细文档：
    - 控制器架构
    - 控制模式详解
    - 配置指南
    - 安全机制
- **Controller Interfaces**
  ROS2 消息与服务定义（位于 `src/controller_interfaces/`）
- **Robotic Arm Bringup**
  系统启动与参数配置（位于 `src/robotic_arm_bringup/`）

### 外部依赖组件（vcs 导入）

- **[Hardware Driver](https://github.com/Ding-Kaiyue/hardware-driver#readme)** 
  CAN-FD 硬件驱动与电机控制库。

- **[Trajectory Interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator#readme)**
  轨迹插值与动力学约束。

- **[Trajectory Planning](https://github.com/Ding-Kaiyue/trajectory-planning#readme)**
  基于 MoveIt2 的轨迹规划库。

- **[CSAPS 库](https://github.com/Ding-Kaiyue/csaps-cpp-redo#readme)** 
  轨迹平滑与样条曲线库。

---

## 外部资源

### 官方文档

- **[ROS2 Humble](https://docs.ros.org/en/humble/)**
- **[MoveIt 2](https://moveit.picknik.ai/humble/)**

### 机器人与优化库
- **[Pinocchio](https://github.com/stack-of-tasks/pinocchio)**
- **[TracIK](https://github.com/aprotyas/trac_ik)** 
- **[OSQP](https://github.com/osqp/osqp)**
- **[NLopt](https://github.com/stevengj/nlopt)**

---

## 问题反馈与社区

- **[Bug 报告](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=bug_report.md)**
- **[安装问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=installation_issue.md)**
- **[使用问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=usage_question.md)** 
- **[功能请求](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=feature_request.md)** 
- **[安全漏洞报告](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=security_report.md)**

---

**维护者主页**: [Ding-Kaiyue](https://github.com/Ding-Kaiyue)
**最后更新**: 2026年1月21日 
