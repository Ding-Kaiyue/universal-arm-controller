# 控制器详解

本文档详细介绍 Arm Controller 支持的所有控制器及其使用方法。

## 📋 目录

- [控制器总览](#控制器总览)
  - [控制器分类](#控制器分类)
  - [模式切换](#模式切换)
- [轨迹执行与控制](#轨迹执行与控制)
  - [轨迹控制器](#轨迹控制器)
    - [MoveJ - 关节空间运动](#movej---关节空间运动)
    - [MoveL - 直线运动](#movel---笛卡尔直线运动)
    - [MoveC - 圆弧运动](#movec---圆弧运动)
  - [实时轨迹控制](#实时轨迹控制)
- [速度控制器](#速度控制器)
  - [JointVelocity - 关节速度控制](#jointvelocity---关节速度控制)
  - [CartesianVelocity - 笛卡尔速度控制](#cartesianvelocity---笛卡尔速度控制)
- [实用控制器](#实用控制器)
  - [HoldState - 安全保持状态](#holdstate---安全保持状态)
  - [Move2Start - 移动到启动位置](#move2start---移动到启动位置)
  - [Move2Initial - 移动到初始位置](#move2initial---移动到初始位置)
  - [ROS2ActionControl - MoveIt 集成](#ros2actioncontrol---moveit-集成)
  - [Disable - 禁用控制](#disable---禁用控制)
- [双臂控制](#双臂控制)
- [状态监控](#状态监控)
- [下一步](#下一步)

## 控制器总览

### 控制器分类

系统支持三类控制器:

| 类别 | 控制器 | 状态 | 说明 |
|------|--------|------|------|
| **轨迹控制器** | MoveJ | ✅ 稳定 | 关节空间点对点运动 |
| | MoveL | ✅ 稳定 | 笛卡尔空间直线运动 |
| | MoveC | ✅ 稳定 | 圆弧/圆周轨迹运动 |
| **速度控制器** | JointVelocity | ✅ 稳定 | 关节空间速度控制 |
| | CartesianVelocity | 🚧 开发中 | 笛卡尔速度控制 |
| **实用控制器** | HoldState | ✅ 稳定 | 安全保持状态 |
| | Move2Start | ✅ 稳定 | 移动到启动位置 |
| | Move2Initial | ✅ 稳定 | 移动到初始位置 |
| | ROS2ActionControl | ✅ 稳定 | MoveIt 轨迹执行 |
| | Disable | ✅ 稳定 | 机械臂失能 |

### 模式切换

所有控制器通过统一的服务接口切换:

```bash
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: '<MODE_NAME>', mapping: '<MAPPING_NAME>'}"
```

**参数说明**:
- `mode`: 目标控制模式名称
- `mapping`: 机械臂映射名称 (`single_arm`, `left_arm`, `right_arm`)

---

## 轨迹执行与控制

### 轨迹控制器

#### MoveJ - 关节空间运动

#### 功能描述

在关节空间进行点对点运动,基于 MoveIt2 规划避免碰撞。

#### 特点

- ✅ 自动碰撞检测和避障
- ✅ 平滑的关节空间轨迹
- ✅ 自适应速度和加速度限制
- ✅ 基于 MoveIt2 的路径规划

#### 使用方法

```bash
# 1. 切换到 MoveJ 模式
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'MoveJ', mapping: 'single_arm'}"

# 2. 发送目标关节位置
ros2 topic pub --once /controller_api/movej_action/single_arm sensor_msgs/msg/JointState \
  "{position: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

#### 消息格式

**订阅话题**: `/controller_api/movej_action/{mapping}`
**消息类型**: `sensor_msgs/msg/JointState`

```yaml
position: [j1, j2, j3, j4, j5, j6]  # 目标关节角度(弧度)
```

#### 工作流程

```
用户指令 → MoveJ控制器 → MoveIt规划 → 轨迹插值 → 硬件执行
```

#### 动力学参数

MoveJ 控制器采用**自适应动力学计算**,无需手动配置速度和加速度参数:

1. **轨迹分析**: 使用 `analyzeTrajectoryDynamics()` 分析规划轨迹的运动特性
2. **参数计算**: 通过 `calculateSafeInterpolationParams()` 自动计算安全的插值参数
3. **自适应插值**: 根据计算的动力学参数生成平滑的插值轨迹

这种方法确保每条轨迹都以最优且安全的速度和加速度执行。

#### 常见问题

**Q: 规划失败怎么办?**

A: 检查目标位置是否在工作空间内,是否有碰撞。

**Q: 运动速度如何调整?**

A: 🚧 **开发中** - 当前版本 MoveJ 自动根据轨迹特性计算最优速度和加速度参数,暂不支持用户自定义调整。velocity_scaling 接口正在开发中,未来版本将支持速度缩放配置。

---

#### MoveL - 笛卡尔直线运动

#### 功能描述

末端执行器在笛卡尔空间沿直线运动,支持智能碰撞规避。

#### 特点

- ✅ 笛卡尔空间直线轨迹(可选择规划策略)
- ✅ 自动碰撞检测和智能回退重规划
- ✅ 保证末端执行器姿态平滑变化
- ✅ 支持关节空间/笛卡尔空间/智能选择三种策略

#### 使用方法

```bash
# 1. 切换到 MoveL 模式
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'MoveL', mapping: 'single_arm'}"

# 2. 发送目标位姿
ros2 topic pub --once /controller_api/movel_action/single_arm geometry_msgs/msg/Pose \
  "{position: {x: 0.19, y: 0.0, z: 0.63}, orientation: {x: -0.4546, y: 0.4546, z: -0.5417, w: 0.5417}}"
```

#### 消息格式

**订阅话题**: `/controller_api/movel_action/{mapping}`
**消息类型**: `geometry_msgs/msg/Pose`

```yaml
position:
  x: 0.19     # X坐标(米)
  y: 0.0      # Y坐标(米)
  z: 0.63     # Z坐标(米)
orientation:
  x: -0.4546  # 四元数X
  y: 0.4546   # 四元数Y
  z: -0.5417  # 四元数Z
  w: 0.5417   # 四元数W
```

#### 规划策略

MoveL 控制器使用 **trajectory_planning** 库的智能规划策略 (`INTELLIGENT`):

1. **笛卡尔空间规划优先**: 先尝试笛卡尔路径规划,保证直线轨迹
2. **自动回退**: 笛卡尔规划失败时自动回退到关节空间规划
3. **智能选择**: 平衡直线性和可达性,确保成功率

> **注意**: MoveL 当前使用固定的 INTELLIGENT 策略,可配置为其他策略,如 CARTESIAN 或 JOINT(表示在笛卡尔空间规划还是关节空间规划)

#### 动力学参数

与 MoveJ 类似,MoveL 采用**自适应动力学计算**:

1. **轨迹分析**: 使用 `analyzeTrajectoryDynamics()` 分析规划轨迹的运动特性
2. **参数计算**: 通过 `calculateSafeInterpolationParams()` 自动计算安全的插值参数
3. **自适应插值**: 根据计算的动力学参数生成平滑的插值轨迹

#### 常见问题

**Q: 为什么轨迹不是直线?**

A: 可能是目标位置导致笛卡尔规划失败,系统自动回退到关节空间规划。检查目标位姿是否接近奇点或有障碍物。

**Q: 规划经常失败?**

A: 目标位置可能超出工作空间或导致碰撞。使用 RViz 可视化检查目标位置的可达性。

---

#### MoveC - 圆弧运动

#### 功能描述

末端执行器沿圆弧或圆周路径运动,支持多种轨迹类型。

#### 特点

- ✅ 支持 Arc(圆弧)轨迹
- 🚧 支持 Bézier(贝塞尔曲线)轨迹(开发中)
- 🚧 支持 Circle(圆周)轨迹(开发中)
- ✅ 自动从当前位置规划到第一个途径点
- ✅ 平滑的圆弧插值

#### 使用方法

```bash
# 1. 切换到 MoveC 模式
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'MoveC', mapping: 'single_arm'}"

# 2. 发送圆弧轨迹(通过途径点)
ros2 topic pub --once /controller_api/movec_action/single_arm geometry_msgs/msg/PoseArray \
  "{poses: [
     {position: {x: 0.30, y: 0.0, z: 0.55}, orientation: {x: -0.5, y: 0.5, z: -0.5, w: 0.5}},
     {position: {x: 0.25, y: 0.0, z: 0.60}, orientation: {x: -0.4777, y: 0.4777, z: -0.5213, w: 0.5213}}
   ]}"
```

#### 消息格式

**订阅话题**: `/controller_api/movec_action/{mapping}`
**消息类型**: `geometry_msgs/msg/PoseArray`

```yaml
poses:
  - position: {x: 0.30, y: 0.0, z: 0.55}      # 途径点1
    orientation: {x: -0.5, y: 0.5, z: -0.5, w: 0.5}
  - position: {x: 0.25, y: 0.0, z: 0.60}      # 途径点2(目标点)
    orientation: {x: -0.4777, y: 0.4777, z: -0.5213, w: 0.5213}
```

#### 工作流程

```
当前位置 → [MoveJ到途径点1] → [圆弧到途径点2]
```

#### 轨迹类型

| 类型 | 描述 | 状态 |
|------|------|------|
| Arc | 通过当前点和两个途径点的圆弧 | ✅ 可用 |
| Bézier | 贝塞尔曲线,可控形状 | 🚧 开发中 |
| Circle | 完整圆周运动 | 🚧 开发中 |

#### 常见问题

**Q: 圆弧不圆滑?**

A: 检查途径点是否合理,三点是否近似共圆。若三点共线,则轨迹会自动退化为直线。

---

## 速度控制器

### JointVelocity - 关节速度控制

#### 功能描述

实时控制各关节的速度,适用于遥操作和力控等场景。

#### 特点

- ✅ 实时速度控制
- ✅ 自动安全限位检查
- ✅ 支持急停状态下的安全反向运动
- ✅ 零速度自动停止

#### 使用方法

```bash
# 1. 切换到速度控制模式
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'JointVelocity', mapping: 'single_arm'}"

# 2. 发送关节速度命令
ros2 topic pub --rate 10 /controller_api/joint_velocity_action/single_arm sensor_msgs/msg/JointState \
  "{velocity: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 3. 停止运动
ros2 topic pub --once /controller_api/joint_velocity_action/single_arm sensor_msgs/msg/JointState \
  "{velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

#### 消息格式

**订阅话题**: `/controller_api/joint_velocity_action/{mapping}`
**消息类型**: `sensor_msgs/msg/JointState`

```yaml
velocity: [v1, v2, v3, v4, v5, v6]  # 关节速度(弧度/秒)
```

#### 安全机制

1. **速度限位**: 自动限制在配置的最大速度内
2. **位置限位**: 接近限位时自动减速或停止
3. **急停恢复**: 超限后允许反向运动脱离限位区
4. **零速停止**: 零速度命令立即停止

#### 使用场景

- 遥操作控制
- 力/阻抗控制
- 示教编程
- 速度跟随

#### 常见问题

**Q: 速度响应延迟?**

A: 速度控制需要持续发送命令,建议使用 `--rate 10` 或更高频率。

**Q: 关节不动?**

A: 检查是否触发急停,查看关节是否在限位。

---

### CartesianVelocity - 笛卡尔速度控制

> 🚧 **开发中** - 该控制器正在开发中,敬请期待。

#### 规划功能

- 末端执行器笛卡尔空间速度控制
- 实时逆运动学求解
- 奇点自动处理

---

## 实用控制器

### HoldState - 安全保持状态

#### 功能描述

安全钩子状态,用于模式切换时的安全过渡。**系统自动管理,无需手动切换**。

#### 工作机制

1. **位置保持策略**: 从位置控制模式切换时,锁定当前关节位置并持续发送保持命令
2. **速度保持策略**: 从速度控制模式切换时,发送零速度命令使机械臂平稳停止
3. **安全检查**: 持续监控机器人停止状态、关节限位和系统健康
4. **自动转换**: 当所有安全条件满足时,自动切换到目标模式

#### 监控的安全条件

1. ✅ 机器人已完全停止
2. ✅ 所有关节在安全限位范围内
3. ✅ 系统健康状态正常

#### 状态转换

```
当前模式 → HoldState(安全检查) → 目标模式
          ↓ (条件不满足)
          等待并持续保持
```

#### 查看状态

```bash
# 查看 HoldState 日志
ros2 topic echo /rosout | grep HoldState
```

---

### Move2Start - 移动到启动位置

#### 功能描述

自动移动到 `hardware_config.yaml` 中定义的 `start_position`。

#### 特点

- ✅ 一键返回预定义启动位置
- ✅ 基于 MoveIt 规划,自动避障
- ✅ 执行完成后保持在 Move2Start 模式

#### 使用方法

```bash
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'Move2Start', mapping: 'single_arm'}"
```

#### 配置

在 `config/hardware_config.yaml` 中配置:

```yaml
hardware_interfaces:
  single_arm:
    start_position: [0.0, -0.5236, -0.7854, 0.0, 0.5236, 0.0]  # 启动位置
```

---

### Move2Initial - 移动到初始位置

#### 功能描述

自动移动到 `hardware_config.yaml` 中定义的 `initial_position`。

#### 特点

- ✅ 一键返回初始配置
- ✅ 基于 MoveIt 规划,自动避障
- ✅ 执行完成后保持在 Move2Initial 模式

#### 使用方法

```bash
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'Move2Initial', mapping: 'single_arm'}"
```

#### 配置

```yaml
hardware_interfaces:
  single_arm:
    initial_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 初始位置
```

---

### ROS2ActionControl - MoveIt 集成

#### 功能描述

通过 ROS2 Action Server 接收来自 MoveIt 的轨迹执行请求。

#### 工作流程

```
MoveIt规划 → FollowJointTrajectory Action → ROS2ActionControl → 轨迹执行
```

1. MoveIt 发送 `FollowJointTrajectory` 动作目标
2. 系统自动切换到 `ROS2ActionControl` 模式
3. 执行轨迹
4. 完成后自动返回 `HoldState`

#### 特点

- ✅ 与 MoveIt 无缝集成
- ✅ 自动状态管理
- ✅ 实时反馈轨迹执行进度
- ✅ 支持轨迹取消和抢占

#### Action 接口

**Action Server**: `/arm_controller/follow_joint_trajectory`

**Action Type**: `control_msgs/action/FollowJointTrajectory`

---

### Disable - 禁用控制

#### 功能描述

禁用电机控制,释放机械臂。

#### 特点

- ✅ 立即禁用所有电机
- ✅ 跳过安全钩子状态
- ✅ 机械臂可手动拖动

#### 使用方法

```bash
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'Disable', mapping: 'single_arm'}"
```

#### 使用场景

- 紧急停止
- 手动示教
- 系统维护
- 拖动示教

---

## 双臂控制

### 独立控制

双臂系统中,每个臂可以独立控制:

```bash
# 左臂 MoveJ
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'MoveJ', mapping: 'left_arm'}"
ros2 topic pub --once /controller_api/movej_action/left_arm sensor_msgs/msg/JointState \
  "{position: [...]}"

# 右臂 MoveL
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: 'MoveL', mapping: 'right_arm'}"
ros2 topic pub --once /controller_api/movel_action/right_arm geometry_msgs/msg/Pose \
  "{position: {...}, orientation: {...}}"
```

### 协同控制

> 🚧 **规划中** - 双臂协同控制功能正在规划中。

---

## 状态监控

### 查看当前模式

```bash
ros2 topic echo /controller_api/running_status
```

### 查看关节状态

```bash
ros2 topic echo /joint_states
```

### 查看硬件状态

```bash
ros2 topic echo /hardware/motor_status
```

---

## 下一步

- 查看 [安全机制](SAFETY.md) 了解安全保护细节
- 查看 [配置指南](CONFIGURATION.md) 了解参数调优
- 查看 [故障排除](TROUBLESHOOTING.md) 解决常见问题
