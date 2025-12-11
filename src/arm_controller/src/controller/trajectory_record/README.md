# 示教录制功能使用指南

TrajectoryRecordController 提供完整的示教录制功能，支持拖动示教、轨迹录制和回放。

## 📋 目录

- [功能概述](#功能概述)
- [系统架构](#系统架构)
- [快速开始](#快速开始)
- [详细步骤](#详细步骤)
- [监控调试](#监控调试)
- [文件格式](#文件格式)
- [常见问题](#常见问题)

---

## 功能概述

### 核心功能
- ✅ **拖动示教**：通过重力补偿实现零力拖动
- ✅ **轨迹录制**：100Hz高频率录制关节状态
- ✅ **异步写入**：后台线程保存数据，不阻塞控制
- ✅ **数据完整**：记录位置、速度、力矩全信息

### 技术特性
- 录制频率：100Hz
- 数据格式：CSV文本格式
- 存储位置：可配置，默认 `~/.../arm_controller/share/arm_controller/recordings/`
- 重力补偿：基于 Pinocchio 动力学库

---

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                   示教录制系统架构                        │
└─────────────────────────────────────────────────────────┘

┌──────────────────┐        ┌──────────────────┐
│  主控制系统      │        │  重力补偿节点     │
│  (arm_controller)│        │  (gravity_comp)   │
└────────┬─────────┘        └────────┬─────────┘
         │                           │
         │ /joint_states             │ /gravity_torque
         │                           │
         ▼                           ▼
┌─────────────────────────────────────────────┐
│     TrajectoryRecordController              │
│  ┌─────────────────────────────────────┐   │
│  │  1. 订阅 /joint_states              │   │
│  │  2. 订阅 /gravity_torque            │   │
│  │  3. 发送重力补偿力矩到电机          │   │
│  │  4. 录制数据到文件 (异步写入)       │   │
│  └─────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
         │
         │ 录制文件 (.txt)
         ▼
┌─────────────────────────────────────────────┐
│  录制文件格式 (CSV)                          │
│  timestamp, pos0-5, vel0-5, effort0-5       │
└─────────────────────────────────────────────┘
```

---

## 快速开始

### 前置准备
```bash
cd /home/w/work/robotic_arm_ws
source install/setup.bash
```

### 三步启动

**终端 1 - 启动主系统**
```bash
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py robot_model_name:=arm380
```

**终端 2 - 启动重力补偿**
```bash
ros2 launch robot_dynamics gravity_compensator.launch.py \
  urdf_file:=/home/w/work/robotic_arm_ws/install/robot_description/share/robot_description/urdf/arm380.urdf
```

**终端 3 - 切换模式并录制**
```bash
# 切换到示教模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'TrajectoryRecord'}"

# 开始录制
ros2 topic pub --once /controller_api/trajectory_record_action \
  std_msgs/msg/String "{data: 'my_trajectory'}"

# 拖动机械臂进行示教...

# 停止录制
ros2 topic pub --once /controller_api/trajectory_record_action \
  std_msgs/msg/String "{data: 'stop'}"
```

---

## 详细步骤

### 步骤 1：启动主控制系统

在终端 1 执行：
```bash
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py robot_model_name:=arm380
```

**等待以下提示出现：**
```
[INFO] [arm_controller]: Initialized 10 controllers
[INFO] [arm_controller]: ✅ Switched to mode SystemStart [single_arm]
[move_group-3] You can start planning now!
```

**验证系统状态：**
```bash
# 检查关节状态是否发布
ros2 topic hz /joint_states

# 应该显示约 1000Hz 的频率
```

---

### 步骤 2：启动重力补偿节点

在终端 2 执行：
```bash
cd /home/w/work/robotic_arm_ws
source install/setup.bash

ros2 launch robot_dynamics gravity_compensator.launch.py \
  urdf_file:=/home/w/work/robotic_arm_ws/install/robot_description/share/robot_description/urdf/arm380.urdf
```

**验证重力补偿是否工作：**
```bash
# 检查话题是否有发布者
ros2 topic info /gravity_torque
# 应显示：Publisher count: 1

# 查看重力补偿数据
ros2 topic echo /gravity_torque --once
# 应显示包含 effort 数组的消息
```

**如果没有数据发布，检查日志：**
- 查找错误信息：`参数 'robot_description' 未设置`
- 确认 URDF 文件路径正确

---

### 步骤 3：切换到示教模式

在终端 3 执行：
```bash
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'TrajectoryRecord'}"
```

**成功响应：**
```
response:
  success: True
  message: '✅ Switched to mode TrajectoryRecord successfully.'
```

**此时系统状态：**
- ✅ TrajectoryRecordController 已激活
- ✅ 订阅 `/gravity_torque` 话题
- ✅ 持续发送重力补偿力矩到电机（kp=0, kd=0 的 MIT 模式）
- ✅ 机械臂可以手动拖动

---

### 步骤 4：开始录制

```bash
ros2 topic pub --once /controller_api/trajectory_record_action \
  std_msgs/msg/String "{data: 'demo_trajectory_001'}"
```

**替换文件名规则：**
- 使用描述性名称，如：`pick_place_demo`, `assembly_task_01`
- 避免特殊字符，只使用字母、数字、下划线
- 不需要添加 `.txt` 后缀，系统会自动添加

**录制开始标志：**
- 状态话题会发布：`recording:demo_trajectory_001`
- 日志显示：`Started recording trajectory: 'demo_trajectory_001'`

---

### 步骤 5：手动拖动示教

**现在可以手动拖动机械臂进行示教！**

**拖动技巧：**
1. **轻柔操作**：虽然有重力补偿，但避免剧烈晃动
2. **关注反馈**：感受机械臂的响应，确保重力补偿生效
3. **速度控制**：保持平稳的速度，便于后续回放
4. **边界注意**：避免触及关节限位

**录制过程监控：**
```bash
# 查看录制状态
ros2 topic echo /controller_api/trajectory_record_status

# 查看关节状态
ros2 topic echo /joint_states --once

# 查看重力补偿力矩
ros2 topic echo /gravity_torque --once
```

---

### 步骤 6：停止录制

```bash
ros2 topic pub --once /controller_api/trajectory_record_action \
  std_msgs/msg/String "{data: 'stop'}"
```

**停止确认：**
- 状态话题会发布：`stopped`
- 日志显示：`Recording stopped`
- 文件已保存并关闭

---

### 步骤 7：查看录制文件

```bash
# 查看录制目录
ls -lh /home/w/work/robotic_arm_ws/install/arm_controller/share/arm_controller/recordings/

# 查看文件内容（前10行）
head -10 /home/w/work/robotic_arm_ws/install/arm_controller/share/arm_controller/recordings/demo_trajectory_001.txt

# 统计录制点数
wc -l /home/w/work/robotic_arm_ws/install/arm_controller/share/arm_controller/recordings/demo_trajectory_001.txt
```

---

## 监控调试

### 实时监控命令

**查看录制状态**
```bash
ros2 topic echo /controller_api/trajectory_record_status
```

**查看当前控制模式**
```bash
ros2 topic echo /controller_api/running_status
```

**查看关节状态频率**
```bash
ros2 topic hz /joint_states
```

**查看重力补偿力矩**
```bash
ros2 topic echo /gravity_torque
```

**查看话题列表**
```bash
ros2 topic list | grep -E "joint|gravity|record"
```

**查看节点信息**
```bash
ros2 node list
ros2 node info /arm_controller
```

### 日志查看

**实时查看主控制器日志：**
- 在启动主系统的终端查看输出

**查看历史日志：**
```bash
ls ~/.ros/log/latest/
cat ~/.ros/log/latest/arm_controller-*.log
```

---

## 文件格式

### 存储位置

**默认路径：**
```
/home/w/work/robotic_arm_ws/install/arm_controller/share/arm_controller/recordings/
```

**降级路径（如果默认路径不可用）：**
```
/tmp/arm_recordings/
```

**自定义路径（可选）：**
在配置文件中设置：
```yaml
controllers:
  TrajectoryRecord:
    output_dir: "/your/custom/path"
```

### 文件格式详解

**文件命名：**
```
<你指定的名称>.txt
例如：demo_trajectory_001.txt
```

**文件格式：CSV（逗号分隔）**

**文件结构：**
```csv
timestamp,pos0,pos1,pos2,pos3,pos4,pos5,vel0,vel1,vel2,vel3,vel4,vel5,effort0,effort1,effort2,effort3,effort4,effort5
0.032444,-0.006370,0.011928,0.000968,0.003193,-0.000157,0.005953,0.001468,-0.000189,-0.000314,-0.000975,0.003479,-0.002228,-0.002964,-0.002242,-0.003106,-0.000000,0.001340,-0.000000
0.083490,-0.006370,0.011929,0.000967,0.003191,-0.000158,0.005953,0.001468,0.000309,-0.003934,-0.003051,-0.000429,0.010325,-0.002964,-0.005854,-0.012632,-0.000000,-0.005880,-0.007069
...
```

**字段说明：**

| 字段 | 说明 | 单位 | 数量 |
|------|------|------|------|
| `timestamp` | 从录制开始的相对时间 | 秒 | 1 |
| `pos0-pos5` | 6个关节的位置 | 弧度 (rad) | 6 |
| `vel0-vel5` | 6个关节的速度 | 弧度/秒 (rad/s) | 6 |
| `effort0-effort5` | 6个关节的力矩 | 牛·米 (N·m) | 6 |

**录制频率：** 100Hz（每0.01秒一个数据点）

**文件大小估算：**
- 每行约400字节
- 1分钟录制：100Hz × 60s = 6000行 ≈ 2.4MB
- 10分钟录制：≈ 24MB

---

## 常见问题

### Q1: 切换到示教模式失败

**现象：**
```
response:
  success: False
  message: '❎ Failed to switch to mode TrajectoryRecord.'
```

**原因：**
- TrajectoryRecordController 未正确注册

**解决方法：**
1. 检查日志是否有 `Controller class 'TrajectoryRecordController' not found`
2. 确认 `controller_registry.cpp` 中已取消注释 TrajectoryRecordController
3. 重新编译：`colcon build --packages-select arm_controller`
4. 重启系统

---

### Q2: 机械臂拖不动

**现象：**
- 切换到示教模式成功
- 但机械臂仍然很硬，无法拖动

**原因：**
- 重力补偿节点未启动或未发布数据

**解决方法：**
```bash
# 1. 检查重力补偿话题
ros2 topic info /gravity_torque

# 如果 Publisher count: 0，说明没有发布者
# 2. 重新启动重力补偿节点，确保提供 urdf_file 参数
ros2 launch robot_dynamics gravity_compensator.launch.py \
  urdf_file:=/home/w/work/robotic_arm_ws/install/robot_description/share/robot_description/urdf/arm380.urdf

# 3. 验证数据发布
ros2 topic echo /gravity_torque --once
```

---

### Q3: 录制文件找不到

**现象：**
- 发送录制命令成功
- 但在默认目录找不到文件

**解决方法：**
```bash
# 1. 检查默认目录
ls -la /home/w/work/robotic_arm_ws/install/arm_controller/share/arm_controller/recordings/

# 2. 检查降级目录
ls -la /tmp/arm_recordings/

# 3. 搜索文件
find /home/w/work/robotic_arm_ws -name "你的文件名.txt" 2>/dev/null

# 4. 查看日志确认保存路径
# 在主系统终端查找类似这样的日志：
# [INFO] [arm_controller]: TrajectoryRecordController initialized. Output dir: /path/to/recordings
```

---

### Q4: 录制文件一直在增长

**现象：**
- 录制文件大小持续增长
- 已经超过预期大小

**原因：**
- 忘记发送停止命令

**解决方法：**
```bash
# 立即停止录制
ros2 topic pub --once /controller_api/trajectory_record_action \
  std_msgs/msg/String "{data: 'stop'}"

# 确认停止
ros2 topic echo /controller_api/trajectory_record_status --once
```

---

### Q5: 启动时提示控制器未找到

**现象：**
```
[WARN] [arm_controller]: [controllers] Controller class 'TrajectoryRecordController' not found for key 'TrajectoryRecord'
```

**原因：**
- 代码修改后未重新编译
- 或编译后未重新 source 环境

**解决方法：**
```bash
# 1. 重新编译
cd /home/w/work/robotic_arm_ws
colcon build --packages-select arm_controller --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. 重新 source
source install/setup.bash

# 3. 重启系统
```

---

### Q6: 同时只能录制一个轨迹

**现象：**
```
[WARN] [arm_controller]: Already recording! Stop current recording first.
```

**原因：**
- 系统设计为同时只支持一个录制任务
- 防止数据混乱

**解决方法：**
```bash
# 先停止当前录制
ros2 topic pub --once /controller_api/trajectory_record_action \
  std_msgs/msg/String "{data: 'stop'}"

# 然后开始新的录制
ros2 topic pub --once /controller_api/trajectory_record_action \
  std_msgs/msg/String "{data: 'new_trajectory'}"
```

---

### Q7: 重力补偿节点启动失败

**现象：**
```
[ERROR] [gravity_compensator]: 参数 'robot_description' 未设置！
```

**原因：**
- 启动时未提供 `urdf_file` 参数

**解决方法：**
```bash
# 确保使用正确的命令启动，包含 urdf_file 参数
ros2 launch robot_dynamics gravity_compensator.launch.py \
  urdf_file:=/home/w/work/robotic_arm_ws/install/robot_description/share/robot_description/urdf/arm380.urdf

# 如果路径不存在，检查 URDF 文件位置
find /home/w/work/robotic_arm_ws -name "arm380.urdf" 2>/dev/null
```

---

## 高级配置

### 自定义录制频率

编辑文件：`src/arm_controller/src/controller/trajectory_record/trajectory_record_controller.cpp`

```cpp
// 第 24-25 行，默认 100Hz
recorder_ = std::make_unique<JointRecorder>(100.0);  // 修改为你想要的频率
```

重新编译后生效。

---

### 自定义保存路径

方法 1：通过配置文件（推荐）

编辑：`src/arm_controller/config/config.yaml`

```yaml
controllers:
  TrajectoryRecord:
    output_dir: "/your/custom/path/recordings"
    input_topic:
      name: /controller_api/trajectory_record_action
    output_topic:
      name: /controller_api/trajectory_record_status
```

方法 2：通过启动参数

```bash
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py \
  robot_model_name:=arm380 \
  trajectory_record_output_dir:="/your/custom/path"
```

---

## 技术细节

### JointRecorder 类

**位置：** `src/arm_controller/include/arm_controller/utils/joint_recorder.hpp`

**核心功能：**
1. 异步写入线程，避免阻塞主控制循环
2. 消息队列缓冲，确保数据不丢失
3. 自动创建目录
4. 时间戳管理

**关键参数：**
- `record_rate_hz`: 录制频率（默认100Hz）
- `record_period_`: 录制周期（1.0 / rate）

---

### TrajectoryRecordController 类

**位置：** `src/arm_controller/src/controller/trajectory_record/trajectory_record_controller.hpp`

**订阅话题：**
- `/joint_states` - 关节状态（用于录制）
- `/gravity_torque` - 重力补偿力矩（用于示教）
- `/controller_api/trajectory_record_action` - 录制命令

**发布话题：**
- `/controller_api/trajectory_record_status` - 录制状态反馈

**控制流程：**
```
start() → 激活控制器 → 订阅重力补偿话题
  ↓
gravity_torque_callback() → 接收重力力矩 → 发送到电机（MIT模式，kp=0, kd=0）
  ↓
joint_states_callback() → 接收关节状态 → 加入录制队列
  ↓
JointRecorder → 异步写入文件
  ↓
stop() → 停止录制 → 取消订阅
```

---

## 相关文件

- **控制器头文件**: `trajectory_record_controller.hpp`
- **控制器实现**: `trajectory_record_controller.cpp`
- **接口文件**: `trajectory_record_interface.cpp`
- **录制器头文件**: `joint_recorder.hpp`
- **录制器实现**: `joint_recorder.cpp`
- **配置文件**: `config/config.yaml`
- **注册文件**: `controller_registry.cpp`

---

## 版本历史

- **v1.0.0** - 初始版本，支持基本示教录制功能
- 支持100Hz录制频率
- 支持异步文件写入
- 集成重力补偿

---

## 贡献指南

如需改进此功能，请遵循以下步骤：
1. 阅读代码并理解现有实现
2. 在新分支进行开发
3. 添加单元测试
4. 更新此文档
5. 提交 Pull Request

---

## 许可证

MIT License - 详见主项目 LICENSE 文件

---

**最后更新：** 2025-11-25
**维护者：** Universal Arm Controller Team
