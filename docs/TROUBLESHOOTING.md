# 故障排除

Universal Arm Controller 常见问题与解决方案。

## 📋 目录

- [安装问题](#安装问题)
- [CAN 接口问题](#can-接口问题)
- [系统启动问题](#系统启动问题)
- [运行问题](#运行问题)
- [性能问题](#性能问题)

---

## 安装问题

### 编译失败：找不到依赖

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

### 编译失败：找不到头文件

**症状**: `fatal error: arm_controller/...h: No such file or directory`

**解决方案**:
```bash
# 清理并重新编译
cd ~/robotic_arm_ws
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### VCS import 失败

**症状**: `vcs import failed` 或 `git clone error`

**解决方案**:
```bash
# 检查网络连接
ping github.com

# 重新尝试导入
cd ~/robotic_arm_ws/src/universal-arm-controller/src
vcs import < ../deps.repos --recursive

# 如果仍然失败，手动克隆
git clone https://github.com/Ding-Kaiyue/hardware-driver.git
git clone https://github.com/Ding-Kaiyue/trajectory-interpolator.git
git clone https://github.com/Ding-Kaiyue/trajectory-planning.git
```

---

## CAN 接口问题

### CAN 接口无法启动

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

### 权限被拒绝

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

### CAN 接口配置正确但仍无法通信

**症状**: CAN 接口启动成功，但数据无法收发

**解决方案**:
```bash
# 测试 CAN 通信
cansend can0 123#0011223344556677

# 在另一个终端监听
candump can0

# 检查 CAN 总线状态
ip -s link show can0

# 可能需要启用混杂模式
sudo ip link set can0 promisc on
```

---

## 系统启动问题

### 启动失败：找不到启动文件

**症状**: `Launch file not found: robotic_arm_real.launch.py`

**解决方案**:
```bash
# 验证环境
source ~/robotic_arm_ws/install/setup.bash

# 检查启动文件位置
ls ~/robotic_arm_ws/install/robotic_arm_bringup/share/robotic_arm_bringup/launch/

# 使用完整路径启动
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

### 启动时 ROS2 节点崩溃

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

## 运行问题

### 机械臂不响应命令

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
ros2 topic echo /controller_api/movej_action

# 4. 检查当前模式
ros2 topic echo /controller_api/running_status

# 5. 查看系统日志
ros2 topic echo /diagnostics
```

### 模式切换失败

**症状**: 无法切换到期望的控制模式

**解决方案**:
```bash
# 查看当前模式
ros2 topic echo /controller_api/running_status

# 尝试先切换到 Disabled 模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'Disable', mapping: 'single_arm'}"

# 等待安全检查完成（通常 1-2 秒）
sleep 2

# 再切换到目标模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'MoveJ', mapping: 'single_arm'}"
```

### 轨迹规划失败

**症状**: `Planning failed` 或 `No solution found`

**原因及解决**:
```bash
# 1. 检查目标是否在工作空间内
# 参考 CONFIGURATION.md 了解关节限制

# 2. 启用 MoveIt 可视化
# 在 RViz 中检查碰撞情况

# 3. 尝试更简单的目标
# 先尝试接近当前位置的目标

# 4. 检查 MoveIt 配置
# 确保 SRDF 和 URDF 配置正确

# 5. 查看规划时间设置
ros2 param get /move_group planning_time
# 如果太短，尝试增加
ros2 param set /move_group planning_time 5.0
```

---

## 性能问题

### 控制延迟过高

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

### 内存占用过高

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

## 调试技巧

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

## 获取帮助

如果问题未在上述列表中解决：

1. **查看日志文件**
   ```bash
   cat ~/.ros/log/*/*/stdout_stderr.log
   ```

2. **提交 GitHub Issue**
   - 包含完整的错误信息和堆栈跟踪
   - 说明你的系统配置和硬件
   - 提供复现问题的步骤

3. **联系维护者**
   - Email: kaiyue.ding@raysense.com
   - 提供详细的问题描述和调试日志

---

**更多文档请访问 [文档中心](README.md)。**