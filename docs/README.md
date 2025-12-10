# Universal Arm Controller - 文档中心

欢迎来到 Universal Arm Controller 的文档中心！这里包含了项目的所有技术文档、快速开始指南、架构设计和故障排除信息。

## 📚 文档目录

### 🚀 快速上手
- **[README](../README.md)** - 项目简介与快速安装
- **[快速开始](QUICKSTART.md)** - 5分钟快速上手教程
- **[安装指南](INSTALLATION.md)** - 详细安装步骤与前置准备

### 📖 用户文档
- **[系统概览](COMPONENTS.md)** - 系统组件与架构说明
- **[故障排除](TROUBLESHOOTING.md)** - 常见问题与解决方案

### 👨‍💻 开发者文档
- **[系统架构](ARCHITECTURE.md)** - 总体架构设计与设计理念
- **[开发指南](../.github/CONTRIBUTING.md)** - 贡献指南与开发流程

### 🔗 组件文档

#### 本仓库维护的组件
- **[Arm Controller 文档中心](../src/arm_controller/docs/README.md)** - 运动控制核心详细文档
  - 包含: 控制器详解、配置指南、安全机制、代码规范等

#### VCS 导入的依赖组件
- **[Hardware Driver](https://github.com/Ding-Kaiyue/hardware-driver#readme)** - CAN-FD 硬件驱动库
  - 功能: CAN-FD 高速通信、实时电机控制、事件驱动监控

- **[Trajectory Interpolator](https://github.com/Ding-Kaiyue/trajectory-interpolator#readme)** - 轨迹插值库
  - 功能: 样条曲线插值、动力学约束满足、实时轨迹生成

- **[Trajectory Planning](https://github.com/Ding-Kaiyue/trajectory-planning#readme)** - 轨迹规划库
  - 功能: MoveIt2 集成、多种规划策略、碰撞检测与避障

---

## 🎯 快速导航

### 我是新用户，想快速开始
1. 阅读 [README](../README.md) 了解项目
2. 跟随 [快速开始](QUICKSTART.md) 5分钟上手
3. 了解 [系统概览](COMPONENTS.md)

### 我需要安装和配置
1. 查看 [安装指南](INSTALLATION.md) 完整的安装步骤
2. 参考 [Arm Controller 配置指南](../src/arm_controller/docs/CONFIGURATION.md) 配置硬件绑定关系

### 我是开发者
1. 阅读 [系统架构](ARCHITECTURE.md) 了解设计
2. 查看 [Arm Controller 文档中心](../src/arm_controller/docs/README.md) 了解控制模块
3. 参考 [开发指南](./.github/CONTRIBUTING.md) 了解贡献流程

### 我遇到了问题
1. 查看 [故障排除](TROUBLESHOOTING.md) 寻找解决方案
2. 在 GitHub Issues 中搜索类似问题
3. 联系维护者: kaiyue.ding@raysense.com

---

## 🔗 外部资源

### 技术参考
- **[ROS2 Humble](https://docs.ros.org/en/humble/)** - ROS2 官方文档
- **[MoveIt 2](https://moveit.picknik.ai/humble/)** - MoveIt 官方教程
- **[Eigen](https://eigen.tuxfamily.org/)** - 线性代数库文档

### 社区
- **[GitHub Issues](https://github.com/Ding-Kaiyue/universal-arm-controller/issues)** - 问题跟踪与讨论
- **[GitHub Discussions](https://github.com/Ding-Kaiyue/universal-arm-controller/discussions)** - 社区讨论

---

## 💡 使用提示

- 📌 **使用目录**: 每个文档开头都有目录，便于快速导航
- 🔗 **交叉链接**: 点击文档中的链接快速跳转到相关内容
- 🔍 **搜索功能**: 使用浏览器的查找功能（Ctrl+F）在文档中搜索关键词
- 📱 **移动友好**: 所有文档都支持在手机/平板上查看

---

**最后更新**: 2025年11月19日 | **维护者**: [Ding-Kaiyue](https://github.com/Ding-Kaiyue)
