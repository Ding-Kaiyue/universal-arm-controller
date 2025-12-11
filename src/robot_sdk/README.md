# Robot SDK

机器人SDK节点，提供外部程序（如QT界面）调用机械臂控制功能的统一接口。

## 功能概述

SDK通过ROS2服务接口 `/sdk_cmd` 提供以下功能：
- 示教录制（开始/停止）
- 轨迹复现
- 模式切换
- 获取当前位姿

## 启动

```bash
ros2 run robot_sdk robot_sdk_node
```

## 服务接口

### 服务名称
`/sdk_cmd` (controller_interfaces/srv/SdkCmd)

### 请求参数

| 参数 | 类型 | 说明 |
|------|------|------|
| command | uint8 | 命令类型（见下方枚举） |
| mapping | string | 映射名称，默认 "single_arm" |
| trajectory_name | string | 轨迹名称（示教/复现时使用） |
| joint_positions | float64[] | 关节目标位置（MoveJ时使用） |
| cartesian_pose | float64[] | 笛卡尔目标位姿（MoveL时使用） |

### 命令枚举

| 命令 | 值 | 说明 |
|------|-----|------|
| SDK_DISABLE | 1 | 失能电机 |
| SDK_JOINT_CONTROL | 2 | 单关节速度控制 |
| SDK_CARTESIAN | 3 | 笛卡尔速度控制 |
| SDK_MOVEJ | 4 | 关节空间运动 |
| SDK_MOVEL | 5 | 笛卡尔空间运动 |
| SDK_TEACH_START | 7 | 开始示教录制 |
| SDK_TEACH_REPEAT | 8 | 开始轨迹复现 |
| SDK_TEACH_STOP | 16 | 停止示教录制 |
| SDK_GET_POSE | 17 | 获取当前位姿 |
| SDK_HOLD_STATE | 20 | 保持当前状态 |

### 响应参数

| 参数 | 类型 | 说明 |
|------|------|------|
| success | bool | 操作是否成功 |
| message | string | 返回消息 |
| current_joint_positions | float64[] | 当前关节位置（弧度） |
| current_cartesian_pose | float64[] | 当前笛卡尔位姿 [x,y,z,rx,ry,rz] |

## 使用示例

### 1. 启动SDK节点

```bash
ros2 run robot_sdk robot_sdk_node
```

### 2. 开始示教录制（自动启动gravity_compensator）

```bash
ros2 service call /sdk_cmd controller_interfaces/srv/SdkCmd "{command: 7, trajectory_name: 'my_traj'}"
```

### 3. 停止示教录制（自动停止gravity_compensator）

```bash
ros2 service call /sdk_cmd controller_interfaces/srv/SdkCmd "{command: 16}"
```

### 4. 轨迹复现（自动启动smoother_node）

```bash
ros2 service call /sdk_cmd controller_interfaces/srv/SdkCmd "{command: 8, trajectory_name: 'my_traj'}"
```

### 5. 获取当前位姿

```bash
ros2 service call /sdk_cmd controller_interfaces/srv/SdkCmd "{command: 17}"
```

### 6. 切换到保持状态

```bash
ros2 service call /sdk_cmd controller_interfaces/srv/SdkCmd "{command: 20}"
```

## 自动节点管理

SDK会自动管理示教和复现所需的辅助节点，无需手动启动：

| 模式 | 自动启动 | 自动停止 |
|------|---------|---------|
| 示教录制 (command: 7) | gravity_compensator | smoother_node |
| 轨迹复现 (command: 8) | smoother_node | gravity_compensator |
| 停止示教 (command: 16) | - | gravity_compensator |
| 切换到其他模式 | - | 当前模式对应的节点 |

## 完整示教复现流程

```bash
# 1. 启动SDK节点
ros2 run robot_sdk robot_sdk_node

# 2. 开始示教（自动启动重力补偿，机械臂可拖动）
ros2 service call /sdk_cmd controller_interfaces/srv/SdkCmd "{command: 7, trajectory_name: 'demo'}"

# 3. 手动拖动机械臂进行示教...

# 4. 停止示教（自动停止重力补偿）
ros2 service call /sdk_cmd controller_interfaces/srv/SdkCmd "{command: 16}"

# 5. 轨迹复现（自动启动轨迹平滑节点）
ros2 service call /sdk_cmd controller_interfaces/srv/SdkCmd "{command: 8, trajectory_name: 'demo'}"
```

## 注意事项

1. SDK节点需要在arm_controller节点启动后运行
2. trajectory_name不需要包含.txt后缀
3. 轨迹文件保存在 `/home/w/work/robotic_arm_ws/trajectories/` 目录
