# 控制模式详解

本文档详细介绍 Arm Controller 支持的所有控制模式及其使用方法。

[TOC]

## 控制模式总览

### 模式分类

系统支持三类模式:

| 类别 | 模式 | 状态 | 说明 |
|:------:|:--------:|:------:|:------:|
| **轨迹模式** | MoveJ | ✅ 稳定 | 关节空间点对点运动 |
| | MoveL | ✅ 稳定 | 笛卡尔空间直线运动 |
| | MoveC | ✅ 稳定 | 圆弧/圆周轨迹运动 |
| **速度模式** | JointVelocity | ✅ 稳定 | 关节空间速度控制 |
| | CartesianVelocity | ✅ 稳定 | 笛卡尔速度控制 |
| **实用模式** | HoldState | ✅ 稳定 | 安全保持状态 |
| | Move2Start | ✅ 稳定 | 移动到启动位置 |
| | Move2Initial | ✅ 稳定 | 移动到初始位置 |
| | ROS2ActionControl | ✅ 稳定 | MoveIt 轨迹执行 |

### 模式切换

所有模式通过统一的服务接口切换，使用以下命令格式。

```bash
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode \
  "{mode: '<MODE>', mapping: '<MAPPING>'}"
```

**参数说明**:
- `<MODE>`: 目标控制模式（详见表 2-1）
- `<MAPPING>`: 机械臂映射名称（详见表 2-2）

**表 2-1：控制模式对照表**

| `<MODE>` | 类别 | 说明 |
|:---:|:---:|:---:|
| **MoveJ** | 轨迹控制模式 | 关节空间点对点运动 |
| **MoveL** | 轨迹控制模式 | 末端执行器在笛卡尔空间沿直线运动 |
| **MoveC** | 轨迹控制模式 | 末端执行器沿圆弧或圆周路径运动 |
| **JointVelocity** | 速度控制模式 | 实时控制各关节的速度 |
| **CartesianVelocity** | 速度控制模式 | 末端执行器笛卡尔空间速度控制 |
| **Move2Start** | 实用控制模式 | 一键移动到启动位置（机械臂运动的起始位姿，非奇异位姿） |
| **Move2Initial** | 实用控制模式 | 一键移动到初始位置（机械臂的折叠位姿，可能奇异） |
| **HoldState** | 实用控制模式 | 安全保持状态，用于模式切换时的安全过渡|
| **ROS2ActionControl** | 实用控制模式 | 通过 ROS2 Action 接收 MoveIt 的轨迹执行请求 |
| **PointRecord** | 示教模式 | 记录当前机械臂的关节位置到文件中 |
| **PointReplay** | 示教模式 | 移动到之前用 PointRecord 记录的位置 |
| **TrajectoryRecord** | 示教模式 | 持续记录机械臂的运动轨迹到文件中 |
| **TrajectoryReplay** | 示教模式 | 重现之前用 TrajectoryRecord 记录的机械臂运动轨迹 |

> [!CAUTION]
> HoldState 模式由系统自动管理，用于模式切换时的安全过渡。用户不应主动切换。

**表 2-2：机械臂映射对照表**

| `<MAPPING>` | 说明 |
|:---:|:---:|
| **single_arm** | 单臂控制（适用于单臂系统） |
| **left_arm** | 左臂控制（适用于双臂系统） |
| **right_arm** | 右臂控制（适用于双臂系统） |

> [!WARNING]
> 当前双臂模式下，各 mapping 仍独立调度，不支持跨臂协调规划。如需双臂协调控制，请移步 `feature/ipc-dual-arm` 分支

#### 模式切换约束说明

- 任意模式切换都会先进入 HoldState
- 正在执行轨迹时，请求切换模式会被延迟，并且正在执行的轨迹会取消执行
- 一个 mapping 同时只能有一个活跃模式
- 非法模式切换请求会返回错误状态

---

## 轨迹控制模式

### 概述

轨迹模式用于规划和执行预定义的运动轨迹，支持三种运动方式：

| 模式 | 运动空间 | 功能描述  |  特点 |
|:---:|:---:|:---:|:---|
| **MoveJ** | 关节空间 | 点对点运动，自动碰撞检测 | • 自动碰撞检测和避障<br/>• 平滑的关节空间轨迹<br/>• 自适应速度和加速度限制<br/>• 基于 MoveIt2 的路径规划 |
| **MoveL** | 笛卡尔空间 | 直线运动，时间最优轨迹 | • 笛卡尔空间轨迹<br/>• 末端速度严格为零（硬约束）<br/>• 所有关节速度/加速度限制被尊重<br/>• 执行时间在约束下最优 |
| **MoveC** | 笛卡尔空间 | 圆弧运动，支持多种轨迹 | • 笛卡尔空间轨迹<br/>• 末端速度严格为零（硬约束）<br/>• 所有关节速度/加速度限制被尊重<br/>• 执行时间在约束下最优 |

### 详细介绍表

| 项目 | MoveJ | MoveL | MoveC |
|:---:|:---|:---|:---|
| **订阅话题** | `/controller_api/movej_action/{mapping}` | `/controller_api/movel_action/{mapping}` | `/controller_api/movec_action/{mapping}` |
| **消息类型** | `sensor_msgs/msg/JointState` | `geometry_msgs/msg/Pose` | `geometry_msgs/msg/PoseArray` |
| **消息格式** | `position: [j1, j2, j3, j4, j5, j6]`<br/>目标关节角度（单位：rad） | `position: {x, y, z}`<br/>`orientation: {x, y, z, w}`<br/>末端执行器目标位姿（单位：m 和 rad）| `poses: [{position, orientation}, ...]`<br/>多个途径点位姿数组 |
| **示例命令** | `ros2 topic pub --once /controller_api/movej_action/single_arm sensor_msgs/msg/JointState "{position: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"` | `ros2 topic pub --once /controller_api/movel_action/single_arm geometry_msgs/msg/Pose "{position: {x: 0.19, y: 0.0, z: 0.63}, orientation: {x: -0.4546, y: 0.4546, z: -0.5417, w: 0.5417}}"` | `ros2 topic pub --once /controller_api/movec_action/single_arm geometry_msgs/msg/PoseArray "{poses: [{position: {x: 0.30, y: 0.0, z: 0.55}, orientation: {x: -0.5, y: 0.5, z: -0.5, w: 0.5}}, {position: {x: 0.25, y: 0.0, z: 0.60}, orientation: {x: -0.4777, y: 0.4777, z: -0.5213, w: 0.5213}}]}"` |
| **工作流程** | 用户指令 → MoveIt规划 → 轨迹采样 → 轨迹插值 → 硬件执行 | 用户指令 → 生成直线轨迹 → 笛卡尔采样 → TracIK逆解 → TOTG生成关键点 → 轨迹插值 → 硬件执行 | 用户指令 → 生成圆弧轨迹 → 笛卡尔采样 → TracIK逆解 → TOTG生成关键点 → 轨迹插值 → 硬件执行 |
| **规划算法** | MoveIt2 路径规划 | 沿笛卡尔直线 IK 采样<br/>TimeOptimalTrajectoryGeneration（TOTG） | 支持多种轨迹类型（见 [MoveC - 轨迹类型详解](#movec---轨迹类型详解)）<br/>TimeOptimalTrajectoryGeneration（TOTG） |

#### MoveC - 轨迹类型详解

MoveC 支持多种轨迹类型，可根据应用场景选择合适的轨迹定义方式：

**轨迹类型**:

| 轨迹类型 | 描述 | 参数 | 状态 |
|:---:|:---|:---|:---:|
| Arc | 通过起点、中间点、目标点的圆弧 | start_pose, via_point, goal_pose | ✅ 可用 |
| Circle | 整圆轨迹，通过圆心和半径定义 | center, radius_point | 🚧 开发中 |
| CircleThrough3Points | 通过三个点定义的圆轨迹 | point1, point2, point3 | 🚧 开发中 |
| Bezier | 贝塞尔曲线，通过控制点定义 | start, ctrl1, ctrl2, goal | 🚧 开发中 |

> [!TIP]
> 这一类控制模式都不适合高频实时控制（规划需要时间）

#### 轨迹速度配置

轨迹模式的速度和加速度通过缩放因子控制（0.0~1.0）。支持使用 ROS2 参数服务动态修改：

```bash
# 修改 MoveJ 速度缩放为 50%
ros2 param set /arm_controller movej.velocity_scaling_factor 0.5
# 修改 MoveJ 加速度缩放为 50%
ros2 param set /arm_controller movej.acceleration_scaling_factor 0.5

# 修改 MoveL 加速度缩放为 60%
ros2 param set /arm_controller movel.acceleration_scaling_factor 0.6
# 修改 MoveL 加速度缩放为 60%
ros2 param set /arm_controller movel.acceleration_scaling_factor 0.6

# 修改 MoveC 速度缩放为 20%
ros2 param set /arm_controller movec.velocity_scaling_factor 0.2
# 修改 MoveC 加速度缩放为 20%
ros2 param set /arm_controller movec.acceleration_scaling_factor 0.2
```

**说明**：
- 缩放因子范围：0.0（停止）~ 1.0（最大速度）
- ROS2 参数修改立即生效，无需重启

#### 轨迹暂停/恢复/取消

轨迹执行过程中支持暂停、恢复和取消操作，通过发布 `TrajectoryControl` 消息实现：

```bash
# 暂停轨迹执行
ros2 topic pub --once /trajectory_control controller_interfaces/msg/TrajectoryControl \
  "{mapping: 'single_arm', action: 'Pause'}"

# 恢复轨迹执行
ros2 topic pub --once /trajectory_control controller_interfaces/msg/TrajectoryControl \
  "{mapping: 'single_arm', action: 'Resume'}"

# 取消轨迹执行
ros2 topic pub --once /trajectory_control controller_interfaces/msg/TrajectoryControl \
  "{mapping: 'single_arm', action: 'Cancel'}"
```

**消息格式**：
- `mapping` - 指定要操作的机械臂映射（"single_arm"、"left_arm"、"right_arm"）
- `action` - 操作类型（"Pause"、"Resume"、"Cancel"）

**说明**：
- 暂停后可恢复继续执行
- 取消后轨迹执行停止，自动返回 HoldState
- 一个 mapping 同时只能执行一个轨迹

---

## 速度控制模式

### 概述

速度模式用于实时控制机械臂的运动速度，支持两种速度控制方式：

| 模式 | 控制空间 | 功能描述 | 特点 |
|:---:|:---:|:---:|:---|
| **JointVelocity** | 关节空间 | 实时关节速度控制 | • 实时速度控制<br/>• 自动安全限位检查<br/>• 支持急停状态下的安全反向运动<br/>• 零速度自动停止 |
| **CartesianVelocity** | 笛卡尔空间 | 末端执行器速度控制 | • 笛卡尔空间速度控制<br/>• 实时逆运动学求解<br/>• 三层安全检测机制<br/>• 奇点自动缩速处理 |

### 详细介绍表

| 项目 | JointVelocity | CartesianVelocity |
|:---:|:---|:---|
| **订阅话题** | `/controller_api/joint_velocity_action/{mapping}` | `/controller_api/cartesian_velocity_action/{mapping}` |
| **消息类型** | `sensor_msgs/msg/JointState` | `geometry_msgs/msg/TwistStamped` |
| **消息格式** | `velocity: [v1, v2, v3, v4, v5, v6]`<br/>关节速度（单位：rad/s） | `header: {frame_id, stamp}` 运动的参考坐标系 <br/>`twist: {linear: {x, y, z}, angular: {x, y, z}}`<br/>末端执行器速度（单位： m/s 和 rad/s） |
| **示例命令** | `ros2 topic pub --rate 10 /controller_api/joint_velocity_action/single_arm sensor_msgs/msg/JointState "{velocity: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"` | `ros2 topic pub --rate 10 /controller_api/cartesian_velocity_action/single_arm geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"` |
| **控制频率** | 100Hz（10ms 控制循环）</br> 100ms超时时间（100ms 没有收到新指令就停止运动） | 100Hz（10ms 控制循环）</br> 100ms超时时间（100ms 没有收到新指令就停止运动） |
| **求解方式** | 直接发送关节速度 | 阻尼最小二乘法（Damped Least Squares）将笛卡尔速度转换为关节速度 |
| **安全机制** | • 速度限位<br/>• 位置限位<br/>• 急停恢复<br/>• 零速停止 | • 前置几何可行性检测<br/>• 阻尼最小二乘求解验证<br/>• 后置方向验证<br/>• 限制恢复机制 |

> [!TIP]
> 这一类控制模式都不适合长时间无人监督运行，并且需要保证外部控制器能做到稳定输入
---

## 实用控制模式

### 概述

实用控制模式用于系统管理、安全过渡和特殊场景控制，支持多种实用功能：

| 模式 | 功能类型 | 功能描述 | 特点 |
|:---:|:---:|:---:|:---|
| **HoldState** | 安全过渡 | 模式切换时的安全保持状态<br/>具体工作机制见[HoldState - 安全保持状态](#HoldState---安全保持状态) | • 自动管理，无需手动切换<br/>• MIT模式锁定位置<br/>• 三层安全条件检查<br/>• 自动转换到目标模式 |
| **Move2Start** | 预定义位置 | 自动移动到 `hardware_config.yaml` 中定义的 `start_position`（启动位置） | • 基于 MoveIt 规划<br/>• 预定义启动位姿<br/>• 执行完成后保持模式 |
| **Move2Initial** | 预定义位置 | 自动移动到 `hardware_config.yaml` 中定义的 `initial_position`（折叠位置） | • 基于 MoveIt 规划<br/>• 预定义折叠位姿<br/>• 执行完成后保持模式 |

### 详细介绍表

| 项目 | HoldState | Move2Start | Move2Initial |
|:---:|:---|:---|:---|
| **系统管理** | 自动管理<br/>无需手动切换 | 进入模式后自动执行 | 进入模式后自动执行 |
| **配置位置** | 系统自动 | `hardware_config.yaml`<br/>`start_position` | `hardware_config.yaml`<br/>`initial_position` |
| **使用场景** | 模式切换过渡 | 快速返回工作起点 | 快速返回折叠位姿 |

#### HoldState - 安全保持状态

**工作机制**:
1. **停止检查**: 检查所有关节速度是否接近零（阈值：0.01 rad/s）
2. **MIT模式保持**: 使用MIT模式（kp=0.05, kd=0.005）锁定当前关节位置，持续发送保持命令
3. **安全检查**: 每100ms执行一次安全检查，监控机器人停止状态、关节限位和系统健康
4. **自动转换**: 当所有安全条件满足时，自动切换到目标模式

**监控的安全条件**:
- 机器人已完全停止
- 所有关节在安全限位范围内
- 系统健康状态正常

---

### ROS2 Action 控制

#### ROS2ActionControl - MoveIt 集成

| 项目 | ROS2ActionControl |
|:---:|:---|
| **功能描述** | 通过 ROS2 Action Server 接收来自 MoveIt 的轨迹执行请求，实现与 MoveIt 的无缝集成 |
| **Action Server** | `/{controller_name}/follow_joint_trajectory` |
| **Action 类型** | `control_msgs::action::FollowJointTrajectory` |
| **工作流程** | MoveIt规划 → FollowJointTrajectory Action → ROS2ActionControl → 轨迹执行 → 自动切换回 HoldState |
| **特点** | • 与 MoveIt 无缝集成<br/>• 自动状态管理<br/>• 实时反馈轨迹执行进度<br/>• 支持轨迹取消和抢占 |
| **事件类型** | • `action_goal_accepted` - 目标被接受<br/>• `action_goal_rejected` - 目标被拒绝<br/>• `action_succeeded` - 轨迹执行成功<br/>• `action_failed` - 轨迹执行失败<br/>• `action_cancelled` - 轨迹被取消<br/>• `action_aborted` - 轨迹执行中止 |

> [!CAUTION]
> 执行完毕后会自动返回 HoldState,如果想使用其他控制模式，需要重新切换到对应模式

---

## 示教模式

### 概述

示教模式用于记录和重现机械臂的运动，支持点位记录/重放和轨迹记录/重放两种方式：

| 模式 | 功能类型 | 功能描述 | 特点 |
|:---:|:---:|:---:|:---|
| **PointRecord** | 点位记录 | 记录当前关节位置到文件 | • 瞬时记录<br/>• 快速点位采集<br/>• 文件存储 |
| **PointReplay** | 点位重放 | 移动到已记录的关节位置 | • 基于MoveIt规划<br/>• 安全轨迹执行<br/>• 批量位置重放 |
| **TrajectoryRecord** | 轨迹记录 | 连续记录机械臂运动轨迹 | • 实时轨迹采样<br/>• 完整轨迹保存<br/>• 高精度记录 |
| **TrajectoryReplay** | 轨迹重放 | 重现已记录的运动轨迹 | • 精确轨迹复现<br/>• 时间同步<br/>• 连续平滑执行 |

### 详细介绍表

| 项目 | PointRecord | PointReplay | TrajectoryRecord | TrajectoryReplay |
|:---:|:---|:---|:---|:---|
| **订阅话题** | `/controller_api/point_record_action/{mapping}` | `/controller_api/point_replay_action/{mapping}` | `/controller_api/trajectory_record_action/{mapping}` | `/controller_api/trajectory_replay_action/{mapping}` |
| **消息类型** | `std_msgs/msg/String` | `std_msgs/msg/String` | `std_msgs/msg/String` | `std_msgs/msg/String` |
| **示例命令** | `ros2 topic pub --once /controller_api/point_record_action/single_arm std_msgs/msg/String "{data: 'point1'}"` | `ros2 topic pub --once /controller_api/point_replay_action/single_arm std_msgs/msg/String "{data: 'point1'}"` | `ros2 topic pub --once /controller_api/trajectory_record_action/single_arm std_msgs/msg/String "{data: 'my_task'}"` | `ros2 topic pub --once /controller_api/trajectory_replay_action/single_arm std_msgs/msg/String "{data: 'my_task'}"` |
| **文件路径** | `<ws>/trajectories/point1.csv` | 从CSV文件中读取（输入的字符串是需要读取的点所被保存的文件名） | `<ws>/trajectories/my_task.csv`<br/>`<ws>/trajectories/my_task_smooth.csv` | 从CSV文件读取<br/>（输入的字符串是需要读取的轨迹所被保存的文件名，无`smooth`后缀的是原始轨迹，有`smooth`后缀的是平滑后轨迹） |
| **控制操作** | - | - | `/controller_api/trajectory_record_control/{mapping}`<br/>• `pause` - 暂停记录<br/>• `resume` - 恢复记录<br/>• `cancel` - 取消记录<br/>• `complete` - 完成记录 | - |

> [!TIP]
> 示教模式特别适合于：
> - 编程复杂任务序列
> - 学习和演示操作流程
> - 离线程序开发
> - 快速原型制作

---

## 状态监控

**查看当前模式**

```bash
ros2 topic echo /controller_api/running_status
```

**查看关节状态**

```bash
ros2 topic echo /joint_states
```

**查看末端位姿**
```bash
ros2 run tf2_tools tf2_echo base_link Link6
```

---

## 下一步

至此，你已经掌握了 Universal Arm Controller 中所有主要控制模式的使用方法，包括：

- 轨迹控制（MoveJ / MoveL / MoveC）
- 速度控制（JointVelocity / CartesianVelocity）
- 实用控制（Move2Start / Move2Initial / HoldState）
- 示教模式（PointRecord / TrajectoryRecord 及回放）

接下来你可以：

- 👉 参考 **[系统启动指南](INSTALLATION.md)** 重新启动系统并进行完整测试  
- 👉 在复杂任务中结合 **示教模式 + 轨迹回放** 进行离线编程  
- 👉 如果在使用过程中遇到任何异常，请优先查阅 **[故障排除](TROUBLESHOOTING.md)**  

---

> [!NOTE]
> 本文档侧重于“如何使用控制接口”。  
> 如果你是开发者，想深入理解控制器实现、插件机制或架构设计，请参阅 **[系统架构与组件说明](overview/ARCHITECTURE.md)** 。
