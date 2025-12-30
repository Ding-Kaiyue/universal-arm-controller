# 故障排除

本文档列出常见问题及其解决方案。

## 📋 目录

- [启动问题](#启动问题)
- [CAN通信问题](#can通信问题)
- [控制问题](#控制问题)
- [规划问题](#规划问题)
- [性能问题](#性能问题)

---

## 启动问题

### 节点启动失败

**症状**: 启动时报错,节点无法运行

**可能原因**:

1. 工作空间未 source
2. 依赖包未安装
3. 配置文件错误

**解决方法**:

```bash
# 1. 确保 source 工作空间
source ~/robotic_arm_ws/install/setup.bash

# 2. 检查依赖
rosdep install --from-paths src --ignore-src -r -y

# 3. 重新编译
cd ~/robotic_arm_ws
colcon build --symlink-install

# 4. 检查配置文件
cat src/universal-arm-controller/src/arm_controller/config/hardware_config.yaml
```

### MoveIt 启动失败

**症状**: MoveIt 相关节点无法启动

**解决方法**:

```bash
# 检查 URDF 加载
ros2 param get /move_group robot_description

# 检查规划组配置
ros2 param get /move_group planning_groups

# 查看详细日志
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py --log-level debug
```

---

## CAN通信问题

### CAN接口未找到

**症状**: 报错 `can0: Cannot find device`

**解决方法**:

```bash
# 1. 检查 CAN 设备
ip link show

# 2. 加载 CAN 驱动(根据实际硬件)
sudo modprobe can
sudo modprobe can_raw
sudo modprobe socketcan

# 3. 配置 CAN 接口
sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on loopback off restart-ms 100
```

### 电机无响应

**症状**: 命令发送后电机不动

**可能原因**:

1. CAN 接口未正确配置
2. 电机 ID 配置错误
3. 电机未上电或未使能

**解决方法**:

```bash
# 1. 检查 CAN 接口状态
ip -details -statistics link show can0

# 2. 监控 CAN 报文
candump can0

# 3. 检查电机配置
ros2 param get /controller_manager hardware_interfaces

# 4. 查看硬件管理器日志
ros2 topic echo /rosout | grep -i "hardware"
```

### CAN总线错误

**症状**: CAN 总线进入 ERROR-PASSIVE 或 BUS-OFF 状态

**解决方法**:

```bash
# 1. 重置 CAN 接口
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 \
  sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on \
  restart-ms 100

# 2. 检查硬件连接
# - 检查终端电阻(120Ω)
# - 检查 CAN_H 和 CAN_L 连接
# - 检查线缆长度和质量
```

---

## 控制问题

### 机械臂不响应命令

**症状**: 发送控制命令后机械臂无反应

**诊断步骤**:

```bash
# 1. 检查当前控制模式
ros2 topic echo /controller_api/running_status

# 2. 检查是否在 HoldState
ros2 topic echo /rosout | grep HoldState

# 3. 检查关节状态
ros2 topic echo /joint_states

# 4. 检查是否有错误日志
ros2 topic echo /rosout | grep -i error
```

**解决方法**:

```bash
# 如果卡在 HoldState,等待安全条件满足
# 查看具体阻塞原因:
ros2 topic echo /rosout | grep "HoldState.*not satisfied"

# 如果关节超限,使用 JointVelocity 反向运动
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'JointVelocity', mapping: 'single_arm'}"
```

### 模式切换失败

**症状**: 模式切换服务调用失败或超时

**可能原因**:

1. 系统未满足安全条件
2. 目标控制器未注册
3. 机器人运动中

**解决方法**:

```bash
# 1. 等待机器人完全停止
# 查看速度是否为零
ros2 topic echo /joint_states --field velocity

# 2. 检查关节是否在限位内
ros2 topic echo /joint_states --field position

# 3. 查看 HoldState 日志
ros2 topic echo /rosout | grep "Safety condition"

# 4. 如果长时间无法切换,重启系统
```

### 运动抖动或不平滑

**症状**: 机械臂运动时出现明显抖动

**可能原因**:

1. 轨迹插值参数不当
2. 控制频率过低
3. 硬件通信延迟

**解决方法**:

```yaml
# 调整 config/config.yaml 中的插值参数
interpolator:
  dt: 0.01              # 减小插值间隔
  spline_type: "CUBIC_SPLINE"  # 使用三次样条
  boundary_condition: "SECOND_DERIVATIVE"
```

```bash
# 检查控制频率
ros2 topic hz /joint_states

# 检查 CAN 延迟
sudo tc qdisc show dev can0
```

---

## 规划问题

### MoveIt 规划失败

**症状**: 规划总是失败,无法生成轨迹

**可能原因**:

1. 目标位置超出工作空间
2. 目标导致碰撞
3. IK 求解失败
4. 规划超时

**解决方法**:

```bash
# 1. 启动 RViz 可视化
ros2 launch arm620_config demo.launch.py

# 2. 在 RViz 中检查:
# - 目标位置是否可达
# - 是否有碰撞(红色表示碰撞)
# - IK 是否有解

# 3. 调整规划参数
ros2 param set /move_group planning_time 10.0  # 增加规划时间
ros2 param set /move_group goal_tolerance 0.01  # 放宽目标容差

# 4. 尝试不同的规划算法
ros2 param set /move_group planner_id "RRTConnectkConfigDefault"
```

### 笛卡尔路径规划失败

**症状**: MoveL 总是回退到关节空间规划

**原因**: 笛卡尔路径规划对约束更严格

**解决方法**: 调整起始位姿,避开奇点

### 接近奇点时规划慢

**症状**: 机械臂接近奇点区域时规划很慢或失败

**原因**: 奇点附近 IK 求解困难

**解决方法**:

1. **使用关节约束规划**(如果有):

```bash
# 限制特定关节,避开奇点
ros2 topic pub --once /joint_constrained_goals \
  trajectory_planning_interfaces/msg/JointConstrainedRequest \
  "{...joint_constraints...}"
```

1. **调整起始位置**:

```bash
# 先移动到远离奇点的位置
ros2 service call /controller_api/controller_mode \
  "{mode: 'MoveJ', mapping: 'single_arm'}"
```

1. **使用 TracIK**:
TracIK 在奇点附近性能更好,确保已安装 TracIK 插件。本工程已经使用了TracIK,无需额外配置。

---

## 性能问题

### 规划速度慢

**症状**: 每次规划需要很长时间

**优化方法**:

```yaml
# 调整规划参数
move_group:
  planning_attempts: 1         # 减少尝试次数
  planning_time: 3.0          # 合理设置超时
```

> **注意**: MoveJ 控制器使用自适应动力学计算,无 velocity_scaling 配置参数

### CPU占用高

**症状**: 系统 CPU 占用过高

**诊断**:

```bash
# 查看进程 CPU 占用
top -H -p $(pgrep -f controller_manager)

# 查看话题频率
ros2 topic hz /joint_states
ros2 topic hz /controller_api/*
```

**优化**:

```yaml
# 降低发布频率
hardware_driver:
  status_publish_rate: 20  # 从 100Hz 降到 20Hz

# 减少日志输出
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py \
  log_level:=warn
```

### 内存占用高

**症状**: 内存持续增长

**诊断**:

```bash
# 检查内存泄漏
valgrind --leak-check=full ros2 run ...

# 监控内存使用
watch -n 1 "ps aux | grep controller"
```

---

## 日志收集

遇到无法解决的问题时,收集以下信息:

```bash
# 1. 系统信息
uname -a
ros2 doctor

# 2. 节点信息
ros2 node list
ros2 node info /controller_manager

# 3. 话题信息
ros2 topic list
ros2 topic info /controller_api/controller_mode

# 4. 参数信息
ros2 param dump /controller_manager

# 5. 日志
ros2 topic echo /rosout > rosout.log
```

---

## 获取帮助

如果问题仍未解决:

1. 搜索 [GitHub Issues](https://github.com/Ding-Kaiyue/universal-arm-controller/issues)
2. 创建新的 Issue,附带上述日志信息
3. 发送邮件至 <kaiyue.ding@raysense.com>

---

## 相关文档

- [配置指南](CONFIGURATION.md) - 详细配置说明
- [控制器详解](CONTROLLERS.md) - 控制器使用方法
- [安全机制](SAFETY.md) - 安全保护机制
