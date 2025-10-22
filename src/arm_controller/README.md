# Arm Controller

[![ROS Version](https://img.shields.io/badge/ROS-ROS2%20Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()

Universal Arm Controller 的运动控制核心，提供多模式运动控制、状态管理和安全监控功能。

> **📖 本组件属于 [Universal Arm Controller](https://github.com/Ding-Kaiyue/universal-arm-controller)。** 系统安装和全局配置请参考 [主项目文档](../../README.md)。

## 🎯 核心功能

**控制模式**
- MoveJ - 关节空间运动
- MoveL - 笛卡尔直线运动
- MoveC - 圆弧轨迹运动
- JointVelocity - 关节速度控制

**关键特性**
- 双臂原生支持（单臂/双臂配置）
- 双节点架构（ControllerManager + TrajectoryController）
- 多层安全机制（HoldState、限位保护）
- MoveIt2 深度集成（碰撞检测、路径规划）
- ROS2 Action Server 完整支持

## 📚 文档

所有详细文档请访问 **[Arm Controller 文档中心](docs/README.md)**：

- **[快速开始](docs/QUICKSTART.md)** - 5分钟上手教程
- **[控制器详解](docs/CONTROLLERS.md)** - 所有控制模式和接口说明
- **[系统架构](docs/ARCHITECTURE.md)** - 双节点架构和设计
- **[配置指南](docs/CONFIGURATION.md)** - 硬件和参数配置
- **[开发者指南](docs/DEVELOPER.md)** - 开发流程和最佳实践
- **[安全机制](docs/SAFETY.md)** - 安全设计和限位保护
- **[故障排除](docs/TROUBLESHOOTING.md)** - 常见问题解决
- **[代码规范](docs/CODE_STYLE.md)** - C++ 编码标准

## 📄 许可证

MIT License - 详见 [LICENSE](../../LICENSE)

## 📞 联系方式

- **GitHub Issues**: [提交问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues)
- **Email**: kaiyue.ding@raysense.com

---

⭐ **如果这个项目对你有帮助，请给我们一个星标！**
