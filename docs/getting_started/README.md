# Getting Started

本目录面向普通用户，涵盖从零开始使用 Universal Arm Controller 的全部内容。

## 推荐阅读顺序

请**严格按照以下顺序**阅读，这是从"零"到"稳定使用"的完整路径：

### [INSTALLATION.md](INSTALLATION.md)
**源码构建详细指南**

- 系统要求检查
- 环境准备与依赖安装
- 源码编译与验证

> [!NOTE]
> 如果使用 Docker 部署，请参考根目录 [README.md](../../README.md#快速开始) 中的快速开始部分，无需阅读本文档。

**预计耗时**：30-60 分钟
**完成标志**：系统成功启动，无编译错误

### [CONTROLLERS.md](CONTROLLERS.md)
**学习所有控制模式与使用方法**

- 轨迹控制（MoveJ / MoveL / MoveC）
- 速度控制（JointVelocity / CartesianVelocity）
- 实用控制（Move2Start / Move2Initial / HoldState / ROS2ActionControl）
- 示教模式（PointRecord / TrajectoryRecord / PointReplay /TrajectoryReplay）

**预计耗时**：1-2 小时
**完成标志**：能够发送各类控制命令，机械臂正常响应

### [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
**遇到问题时查阅**

- 编译失败排查
- 启动异常排查
- 机械臂不动排查
- 通信异常排查

**使用场景**：安装或运行过程中遇到问题时，按照本文档逐步排查

---

## 核心文档速查

| 文档 | 用途 | 何时阅读 |
|------|------|--------|
| **[INSTALLATION.md](INSTALLATION.md)** | 安装与环境配置 | 第一次使用 |
| **[CONTROLLERS.md](CONTROLLERS.md)** | 控制命令参考 | 学习如何控制机械臂 |
| **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** | 问题排查 | 遇到异常时 |

---

## 完成标志

当你能够：
- ✅ 成功启动系统（无错误）
- ✅ 发送各类控制命令（机械臂正常响应）
- ✅ 遇到问题能查看故障排查流程解决问题

说明你已具备**正常使用本系统的全部能力**。

---

**更多信息请访问 [文档中心](../README.md)。**
