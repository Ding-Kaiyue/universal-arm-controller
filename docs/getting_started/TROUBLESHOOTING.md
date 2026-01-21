# 故障排除（Troubleshooting）

本文档用于帮助用户**按阶段、按优先级**定位并解决 Universal Arm Controller 在安装、启动、运行及长期使用过程中可能遇到的问题。

请严格按照本文档的结构，从上到下逐步排查。  

**切勿在未完成基础检查的情况下直接跳到后续章节**，否则很可能浪费大量调试时间。

---
[TOC]

---

## 快速定位

- 无法编译 / 依赖错误 → [第 1 章](#1-无法完成安装或编译构建阶段问题)
- 能编译但无法 launch → [第 2 章](#2-系统无法启动launch--node-阶段)
- 能 launch 但机械臂不动 → [第 3 章](#3-系统已启动但机械臂不运动常见问题)（最常见）
- 能动但轨迹异常 → [第 4 章](#4-机械臂能运动但行为异常)
- 运行一段时间后出问题 → [第 5 章](#5-长时间运行相关问题稳定性)

---

## 0. 快速自检（必须优先完成）

> [!IMPORTANT]
> 如果未完成本章节，请不要直接跳到后续章节。实践中，绝大多数问题可在此阶段被发现并解决。

* 已正确加载 ROS 2 环境：

  ```bash
  source /opt/ros/humble/setup.bash
  source ~/robotic_arm_ws/install/setup.bash
  ```
* 控制相关话题存在：

  ```bash
  ros2 topic list | grep controller_api
  ```
* CAN 接口存在且状态为 UP：

  ```bash
  ip link show can0
  ```
* 机械臂已上电，急停已释放
* 当前控制模式不为 `HoldState`
  可通过以下方式检查运行状态：

  ```bash
  ros2 topic echo /controller_api/running_status
  ```

---

## 1. 无法完成安装或编译（构建阶段问题）

> 适用于 `colcon build` 失败、依赖缺失等情况。

### 1.1 找不到第三方依赖（如 Boost）

**症状**: `CMake Error: Could not find Boost`

**解决方案**:

```bash
# 重新运行依赖安装
cd ~/robotic_arm_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 确保 ROS 环境已加载
source /opt/ros/humble/setup.bash
```

---

### 1.2 编译时报头文件缺失

**症状**: `fatal error: arm_controller/...h: No such file or directory`

**解决方案**:

```bash
# 清理并重新编译
cd ~/robotic_arm_ws
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

### 1.3 `vcs import` 或源码获取失败

**症状**: `vcs import failed` 或 `git clone error`

**解决方案**:

```bash
# 检查网络连接
ping github.com

# 重新尝试导入
cd ~/robotic_arm_ws/src/universal-arm-controller/src
vcs import < ../deps.repos --recursive

# 如果仍然失败，手动克隆
cd src/
git clone https://github.com/Ding-Kaiyue/hardware-driver.git
git clone https://github.com/Ding-Kaiyue/trajectory-interpolator.git
git clone https://github.com/Ding-Kaiyue/trajectory-planning.git
git clone https://github.com/Ding-Kaiyue/csaps-cpp-redo.git
```

---

## 2. 系统无法启动（Launch / Node 阶段）

> 编译成功，但在 `ros2 launch` 阶段失败或节点异常退出。

### 2.1 找不到 launch 文件

**症状**: `Launch file not found: robotic_arm_real.launch.py`

**解决方案**:

```bash
# 验证环境
source ~/robotic_arm_ws/install/setup.bash

# 检查启动文件位置
ls ~/robotic_arm_ws/install/robotic_arm_bringup/share/robotic_arm_bringup/ | grep robotic_arm_real.launch.py

# 使用完整路径启动
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

---

### 2.2 节点启动后立即退出或无日志

**症状**: 节点启动后立即退出，无错误信息

**解决方案**:

```bash
# 启用详细日志
export ROS_LOG_DIR=~/.ros/log
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py --log-level debug

# 查看日志
cat ~/.ros/log/*/robot_state_publisher*/*.log
```

---

## 3. 系统已启动，但机械臂不运动（常见问题）

> 节点与话题存在，但执行命令后机械臂无响应。

### 3.1 机械臂完全无动作

**症状**: 发送控制命令但机械臂无动作

**检查清单**:

```bash
# 1. 检查电机是否上电
# 物理检查：LED 是否亮

# 2. 检查 CAN 接口状态
ip link show can0
# 应该看到 "UP,RUNNING"

# 3. 检查控制命令是否发送成功
# 监听话题
ros2 topic echo /controller_api/movej_action/single_arm

# 4. 检查当前模式
ros2 topic echo /controller_api/running_status

# 5. 查看系统日志
ros2 topic echo /diagnostics
```

---

### 3.2 模式切换失败

**症状**: 无法切换到期望的控制模式

**解决方案**:

```bash
# 查看当前模式
ros2 topic echo /controller_api/running_status

# 尝试先切换到 HoldState 模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'HoldState', mapping: 'single_arm'}"

# 等待安全检查完成（通常 1-2 秒）
sleep 2

# 再切换到目标模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'MoveJ', mapping: 'single_arm'}"
```

---

### 3.3 CAN 接口相关问题

#### 3.3.1 CAN 接口未成功启动

**症状**: `Cannot assign requested address` 或 `Device or resource busy`

**解决方案**:

首先检查 CAN 硬件：

```bash
# 列出所有网络接口
ifconfig -a

# 或使用 ip 命令
ip link show
```

如果看到 `can0`，尝试配置：

```bash
# 配置 CAN 接口（CAN-FD）
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 \
  dbitrate 5000000 dsample-point 0.75 fd on loopback off restart-ms 100

# 验证配置
ip link show can0
```

如果 CAN 接口不存在，检查驱动程序：

```bash
# 加载 CAN 驱动
sudo modprobe can
sudo modprobe can_raw

# 检查是否加载成功
lsmod | grep can
```

#### 3.3.2 权限不足导致无法访问 CAN

**症状**: `Operation not permitted` 或 `Permission denied`

**解决方案**:

选项 1 - 使用 sudo：

```bash
sudo ip link set can0 up type can bitrate 1000000 ...
```

选项 2 - 添加到组：

```bash
# 将用户添加到 dialout 组
sudo usermod -a -G dialout $USER

# 重新登录或运行
newgrp dialout

# 重新配置 CAN
ip link set can0 up type can bitrate 1000000 ...
```

#### 3.3.3 接口正常但通信异常

**症状**: CAN 接口启动成功，但数据无法收发

**解决方案**:

```bash
# 测试 CAN 通信
cansend can0 123#0011223344556677

# 在另一个终端监听
candump can0

# 检查 CAN 总线状态
ip -s link show can0
```

---

## 4. 机械臂能运动，但行为异常

> 可执行运动，但轨迹或控制效果不符合预期。

### 4.1 MoveIt 规划失败

**症状**: `Planning failed` 或 `No solution found`

**原因及解决**:

```bash
# 1. 检查目标是否在工作空间内
# 参考 CONFIGURATION.md 了解关节限制

# 2. 启用 MoveIt 可视化
# 在 RViz 中检查碰撞情况, 并确保执行路径不会经过奇异位姿

# 3. 尝试更简单的目标
# 先尝试接近当前位置的目标

# 4. 检查 MoveIt 配置
# 确保 SRDF 和 URDF 配置正确
```

---

### 4.2 运动不平滑 / 延迟较高

**症状**: 响应不及时，运动不平滑

**优化方案**:

```bash
# 1. 检查 CPU 使用率
top -p $(pgrep -f arm_controller)

# 2. 检查 ROS 网络延迟
ros2 topic hz /joint_states

# 3. 启用 CPU 亲和性
# 在配置文件中设置 CPU core
# 详见 CONFIGURATION.md

# 4. 关闭不必要的日志
export ROS_LOG_LEVEL=warn

# 5. 使用 Release 编译
# 确保编译时使用了 -DCMAKE_BUILD_TYPE=Release
```

---

## 5. 长时间运行相关问题（稳定性）

> 运行一段时间后才出现的问题。

### 5.1 内存或资源占用持续上升

**症状**: 程序运行一段时间后内存占用增加

**检查方案**:

```bash
# 监控内存使用
watch -n 1 'ps aux | grep arm_controller'

# 使用 valgrind 检查内存泄漏
valgrind --leak-check=full ros2 run arm_controller arm_controller_node

# 查看内存映射
cat /proc/$(pgrep -f arm_controller)/maps
```

---

## 6. 调试工具与进阶排查手段

> 面向需要进一步定位问题的高级用户。

### 启用详细日志

```bash
# 设置日志级别
export ROS_LOG_LEVEL=debug

# 查看特定模块的日志
ros2 run arm_controller arm_controller_node --ros-args --log-level arm_controller:=debug
```

### 使用 RViz 可视化

```bash
# 启动 RViz
rviz2

# 添加 TF 显示机械臂位置
# 添加 MarkerArray 显示规划路径
```

### 使用 rqt 工具

```bash
# 启动 rqt 图形工具
rqt

# 使用 rqt_graph 查看节点连接
# 使用 rqt_topic 监控话题
# 使用 rqt_service_caller 调用服务
```

---

## 7. 问题仍未解决时的处理方式

### 7.1 提供必要信息

在寻求帮助前，请准备以下信息：

* 使用的控制模式与 mapping
* 相关日志文件（`~/.ros/log/...`）
* 硬件型号与 CAN 配置
* 最近一次可正常运行的 commit

查看日志文件：

```bash
# 查看最新的日志
ls -t ~/.ros/log/*.log | head -1 | xargs cat

# 或查看所有最近的日志
tail -n 100 ~/.ros/log/*.log
```

---

### 7.2 提交 GitHub Issue（推荐）

- **[Bug 报告](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=bug_report.md)** - 报告使用中的 Bug
- **[安装问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=installation_issue.md)** - 报告安装过程中的问题
- **[使用问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=usage_question.md)** - 询问使用方法
- **[功能请求](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=feature_request.md)** - 请求新功能或改进
- **[安全漏洞报告](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=security_report.md)** - 报告安全漏洞

---

### 7.3 紧急或硬件相关问题

* 联系方式：[kaiyue.ding@raysense.com](mailto:kaiyue.ding@raysense.com)
* 提供详细的问题描述和调试日志

---

## 文档维护原则

本文档优先收录高频、可复现、对系统稳定性影响较大的问题。偶发性或一次性问题通常通过 Issue 进行跟踪，而不直接写入本文档。

---

> 本文档将随着项目演进持续更新。欢迎通过 Issue 或 Pull Request 补充新的问题与解决方案。

