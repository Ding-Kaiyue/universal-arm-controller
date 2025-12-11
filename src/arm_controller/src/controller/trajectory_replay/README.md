# 轨迹复现功能 (Trajectory Replay)

## 功能概述

轨迹复现功能允许用户回放之前录制的机械臂运动轨迹。系统会自动：
1. 使用 MoveIt 将机械臂移动到轨迹起点
2. 通过 smoother_node 对轨迹进行平滑处理
3. 直接调用 hardware_manager 执行平滑后的轨迹（绕过 trajectory_controller 的插值）

## 系统架构

```
TrajectoryReplayController
        |
        | 1. 加载轨迹文件 (.txt)
        | 2. MoveIt 回到起点
        | 3. 发布原始轨迹到 /raw_joint_trajectory
        v
   smoother_node (Python)
        |
        | 4. csaps 样条平滑 (smooth_factor=0.95)
        | 5. 发布平滑轨迹到 /smoothed_joint_trajectory
        v
TrajectoryReplayController::smoothed_trajectory_callback
        |
        | 6. 弧度 -> 度数 单位转换
        | 7. hardware_manager_->executeTrajectory()
        v
   RobotHardware::execute_trajectory
        |
        | 8. 直接发送 MIT 控制命令到电机
        v
      机械臂执行
```

## 使用方法

### 1. 启动必要节点

```bash
# 终端1: 启动机械臂主系统
cd /home/w/work/robotic_arm_ws
source install/setup.bash
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py robot_model_name:=arm380

# 终端2: 启动轨迹平滑节点
cd /home/w/work/robotic_arm_ws
source install/setup.bash
ros2 run traj_smoother_py smoother_node
```

### 2. 录制轨迹

```bash
# 切换到录制模式
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'TrajectoryRecord', mapping: 'single_arm'}"

# 开始录制（发送轨迹名称）
ros2 topic pub --once /controller_api/trajectory_record std_msgs/msg/String "{data: 'my_trajectory'}"

# 手动拖动机械臂...

# 停止录制
ros2 topic pub --once /controller_api/trajectory_record std_msgs/msg/String "{data: 'stop'}"
```

轨迹文件保存在: `/home/w/work/robotic_arm_ws/trajectories/my_trajectory.txt`

### 3. 复现轨迹

```bash
# 切换到复现模式
ros2 service call /controller_api/controller_mode controller_interfaces/srv/WorkMode "{mode: 'TrajectoryReplay', mapping: 'single_arm'}"

# 开始复现（发送轨迹名称，不带.txt后缀）
ros2 topic pub --once /controller_api/trajectory_replay_action std_msgs/msg/String "{data: 'trajectory_2'}"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='trajectory_2')
```

### 4. 停止复现

```bash
ros2 topic pub --once /controller_api/trajectory_replay_action std_msgs/msg/String "{data: 'stop'}"
```

## 轨迹文件格式

轨迹文件为纯文本格式，每行一个轨迹点：
```
timestamp joint1 joint2 joint3 joint4 joint5 joint6
```
- timestamp: 相对时间（秒）
- joint1-6: 关节角度（弧度）

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 录制频率 | 100 Hz | JointRecorder 采样率 |
| 平滑系数 | 0.95 | csaps smooth factor (0-1，越大越平滑) |
| 最少点数 | 5 | 少于5点不进行平滑处理 |

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/controller_api/trajectory_replay` | std_msgs/String | 复现命令输入 |
| `/controller_api/trajectory_replay_status` | std_msgs/String | 状态反馈 |
| `/raw_joint_trajectory` | trajectory_msgs/JointTrajectory | 原始轨迹（发给smoother） |
| `/smoothed_joint_trajectory` | trajectory_msgs/JointTrajectory | 平滑后轨迹 |

## 状态反馈

| 状态 | 说明 |
|------|------|
| `replaying` | 正在复现 |
| `completed` | 复现完成 |
| `stopped` | 已停止 |
| `error:file_not_found` | 轨迹文件不存在 |
| `error:empty_trajectory` | 轨迹为空 |
| `error:recording_active` | 正在录制，无法复现 |
| `error:move_to_start_failed` | 回到起点失败 |
| `error:invalid_interface` | 无效的硬件接口 |
| `error:execution_failed` | 轨迹执行失败 |

## 注意事项

1. **确保只有一个 smoother_node 实例运行**
   ```bash
   # 检查是否有多个实例
   ros2 topic info /raw_joint_trajectory

   # 如果有多个订阅者，杀掉旧进程
   pkill -f "smoother_node"
   ```

2. **单位转换**
   - ROS 轨迹消息使用弧度 (rad)
   - 硬件驱动使用度数 (deg)
   - 转换在 `smoothed_trajectory_callback` 中自动完成

3. **复现前会自动回到起点**
   - 使用 MoveIt MoveGroupInterface 规划路径
   - 如果起点与当前位置相差太大，规划可能失败

4. **轨迹执行是阻塞的**
   - 执行期间会显示进度条
   - 完成后自动切换到 HoldState 模式
