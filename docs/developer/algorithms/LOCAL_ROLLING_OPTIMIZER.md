# 局部滚动优化器设计说明

> 本文档描述 Universal Arm Controller 系统中，reactive_task 控制器内的局部滚动优化器（Local Rolling Optimizer / Local TrajOpt）的功能定位、算法流程、代码组织、系统集成方式与责任边界。

## 1. 功能定位与系统层角色

局部滚动优化器解决的核心问题是：**在全局参考轨迹已经存在的前提下，围绕当前执行位置截取一个短视距窗口，用 TrajOpt 对未来若干步关节路径做局部平滑和障碍约束优化，再把优化后的局部参考交给 NEO 做实时速度级跟踪**。

在 reactive_task 架构中，它位于全局规划器和 NEO 之间：

```text
Global Planner / Global Trajectory
    -> Local Rolling Optimizer
    -> NEO velocity-level QP
    -> joint velocity command
```

它不是最终控制器，也不直接发布关节命令。系统内的角色是：

- **短视距局部优化**：只优化当前 tick 附近的一小段参考，不重新搜索完整全局路径
- **轨迹形状修正**：在全局路径基础上改善局部平滑性、避障间隙和关节空间走向
- **动态障碍响应补充**：从点云地图中选择局部障碍球，交给 TrajOpt 参与优化
- **参考生成器**：输出末端目标 pose / twist 和可选 joint target，供 NEO 使用
- **异步滚动更新**：执行循环周期性发起局部优化，后台结果返回后再接管局部参考

### 1.1 适用范围声明

**本模块面向**：reactive_task 中的短视距局部参考优化问题。

**适用于**：

- 全局路径已经存在，但局部需要更平滑或更贴合当前环境
- 点云中出现局部障碍，需要在接下来几步内调整参考
- NEO 仍负责最终速度命令，局部优化器只提供更好的跟踪目标
- 低频局部重规划和高频速度控制分离的架构

**不适用于**：

- 替代全局规划器做大范围路径搜索
- 替代 NEO 处理每个 tick 的速度级 CBF
- 直接发布关节位置、关节速度或力矩命令
- 保证对所有动态障碍的硬实时响应
- 处理移动底盘或全身动力学优化

> [!NOTE]
> 当前实现对应代码中的 `ReactiveTaskLocalPlanner`。它使用 Tesseract 表达机器人环境、运动指令和 profile，再通过 Tesseract 暴露的 TrajOpt planner 求解局部轨迹优化问题。

---

## 2. 问题定义

每次局部优化请求的核心输入包括：

- 当前或预测起点关节状态：

$$
\mathbf{q}_{start} \in \mathbb{R}^{n}
$$

- 全局轨迹在短视距窗口内采样得到的局部参考：

  $$
  \{S_1, S_2, \ldots, S_H\}
  $$

  其中每个 sample 可能包含：

  - 目标末端位姿 $T_i$
  - IK 关节目标 $\mathbf{q}_i$
  - 目标 twist
  - 时间信息

- 窗口末端关节目标：

$$
\mathbf{q}_{goal}
$$

- 局部障碍球集合：

$$
\mathcal{O} = \{(\mathbf{c}_j, r_j)\}
$$

局部优化器求解一条短窗口关节轨迹：

$$
\mathbf{Q}^{*} =
\{\mathbf{q}^{*}_0, \mathbf{q}^{*}_1, \ldots, \mathbf{q}^{*}_K\}
$$

然后通过 FK 转换成：

- 优化后的末端目标 pose 序列
- 优化后的末端 target twist 序列
- 优化后的 joint target 序列

reactive_task 执行循环会从这条局部轨迹中按当前时间取一个 sample，作为 NEO 的跟踪目标。

---

## 3. 滚动窗口机制

### 3.1 触发频率与窗口长度

局部优化器低频运行，配置位于 `reactive_task_controller.local_planner`：

```yaml
local_planner:
  type: trajopt
  frequency_hz: 20.0
  horizon_steps: 16
  dt_sec: 0.08
```

其中：

- `frequency_hz` 控制局部优化请求的触发频率
- `horizon_steps` 控制从全局参考中截取多少个 lookahead sample
- `dt_sec` 控制局部窗口内部参考采样的时间间隔

NEO 通常以更高频率运行，例如 100 Hz；局部 TrajOpt 只负责周期性刷新未来短窗口参考。

### 3.2 起点预测

局部优化请求不总是直接使用当前 $\mathbf{q}_{now}$ 作为起点。为了补偿 TrajOpt 求解耗时和调度延迟，执行循环会根据上一 tick 的关节速度参考预测短时间后的起点：

$$
\mathbf{q}_{start}
=
\mathbf{q}_{now}
+
\Delta t_{pred}\dot{\mathbf{q}}_{prev}
$$

预测结果会裁剪到关节限位内：

$$
\mathbf{q}_{min}
\leq
\mathbf{q}_{start}
\leq
\mathbf{q}_{max}
$$

如果预测起点 FK 失败，则回退到当前关节状态和当前末端位姿。

### 3.3 窗口采样

局部窗口从当前全局参考时间向前采样：

$$
t_k =
t_{current}
+
\Delta t_{pred}
+
k\Delta t_{local}
$$

采样步数由 `horizon_steps` 控制。窗口末端 sample 的 IK 结果作为局部优化的 `q_goal`。

这意味着局部优化器不是独立寻找终点，而是在全局参考轨迹附近做短视距修正。

---

## 4. Tesseract 与 TrajOpt 的分工

本模块同时使用 Tesseract 和 TrajOpt，但两者职责不同：

| 组件 | 在本模块中的作用 |
|------|------------------|
| Tesseract Environment | 保存机器人 URDF / SRDF、规划组、运动学信息、碰撞管理器和局部障碍球 |
| Tesseract Command Language | 用 `CompositeInstruction`、`MoveInstruction`、`StateWaypoint`、`JointWaypoint` 描述一段要优化的运动程序 |
| Tesseract Profile | 用 `TrajOptDefaultMoveProfile` 和 `TrajOptDefaultCompositeProfile` 配置代价、约束、平滑项和碰撞项 |
| TrajOpt | 根据 Tesseract instruction、environment 和 profile 构造并求解轨迹优化问题 |

可以把 Tesseract 理解为“机器人环境与规划任务描述框架”，TrajOpt 是其中一个具体的优化求解后端。

> [!IMPORTANT]
> 本模块没有直接手写 TrajOpt 的底层优化矩阵，而是通过 Tesseract 的 planner request 组织问题：
>
> ```cpp
> tesseract_planning::PlannerRequest request;
> request.instructions = seed_program;
> request.env = env;
> request.profiles = profiles;
> ```

---

## 5. TrajOpt 优化模型

### 5.1 种子轨迹

`ReactiveTaskLocalPlanner::compute()` 先构造局部 seed waypoints：

1. 当前或预测起点 $\mathbf{q}_{start}$
2. 短视距窗口内带 IK joint target 的全局参考点
3. 窗口末端 $\mathbf{q}_{goal}$

相邻关节点如果几乎相同会被跳过，避免生成退化路径。

如果局部 seed 数量超过代码常量限制，会先压缩为最多 16 个状态：

```cpp
kMaxTrajOptStates = 16
```

当前实现会保持局部问题稀疏，直接使用构造好的 `program` 作为 TrajOpt 输入：

```cpp
tesseract_planning::CompositeInstruction seed_program = program;
```

这里有一个重要设计取舍：没有再调用 `generateInterpolatedProgram(...)` 做按段插值。原因是该接口的 `min_steps` 是按 segment 生效，一个 10 到 16 点的局部窗口可能膨胀成上百个 TrajOpt 状态，导致求解结果明显变旧，反而让 NEO 跟踪到过期参考。

### 5.2 起点与终点

首尾点使用 `StateWaypoint` 表达：

```cpp
tesseract_planning::StateWaypoint waypoint(input.joint_names, q);
```

求解后还会校验首尾点是否被优化器移动过多：

```cpp
kStartStateToleranceRad = 1.0e-4
kEndStateToleranceRad = 1.0e-4
```

如果 TrajOpt 返回的首点或末点偏离超过容忍度，本次局部结果会被拒绝。

### 5.3 中间关节走廊

中间点使用 `JointWaypoint`，围绕全局参考 IK 解设置软走廊：

```cpp
kJointCorridorSoftToleranceRad = 0.35
```

对应含义是：局部优化可以在参考关节路径附近调整，但不应完全脱离全局路径意图。

这个值需要在“贴近全局参考”和“局部绕障自由度”之间折中：

| 参数趋势 | 可能效果 | 主要风险 |
|----------|----------|----------|
| 过小 | 中间点被强烈约束在全局 IK 参考附近，输出轨迹更像原始全局路径 | 遇到局部障碍时可调整空间不足，TrajOpt 可能失败，或生成几乎不避障的结果 |
| 适中 | 允许局部窗口在关节空间内做小范围修正，同时保留全局路径的拓扑意图 | 需要和 collision cost、平滑项、NEO 跟踪能力一起调参 |
| 过大 | 局部优化有更大的绕障空间，可能找到离障碍更远的短窗口轨迹 | 更容易脱离全局参考、跳到不期望的关节分支，或陷入对 NEO 不友好的局部最优 |

当前实现开启 joint cost，关闭 joint constraint：

```cpp
move_profile->joint_cost_config.enabled = true;
move_profile->joint_cost_config.use_tolerance_override = true;
move_profile->joint_constraint_config.enabled = false;
```

因此关节走廊主要是软偏好，而不是硬限制。真正的实时关节限位仍由 NEO 的速度级约束和关节限位 CBF 处理。

### 5.4 平滑项

Composite profile 开启速度和加速度平滑：

```cpp
composite_profile->smooth_velocities = true;
composite_profile->smooth_accelerations = true;
composite_profile->smooth_jerks = false;
```

这使 TrajOpt 倾向于生成速度和加速度更平滑的局部关节轨迹。jerk smoothing 当前关闭，避免局部优化问题过重。

### 5.5 碰撞 cost 与 constraint

局部优化器把点云中的局部障碍近似为一组 sphere obstacle，并放入 Tesseract environment。TrajOpt 中配置：

```cpp
collision_cost_config = TrajOptCollisionConfig(0.005, 15.0);
collision_constraint_config = TrajOptCollisionConfig(0.0, 20.0);
```

是否启用由配置控制：

```yaml
enable_collision_cost: true
enable_collision_constraint: true
```

局部 TrajOpt 避障和 NEO CBF 避障处在不同层级：

| 模块 | 频率 | 作用对象 | 主要职责 |
|------|------|----------|----------|
| Local TrajOpt | 低频 | 未来短窗口轨迹 | 让参考路径提前绕开局部障碍 |
| NEO CBF | 高频 | 当前 tick 关节速度 | 直接约束当前速度命令，保证实时安全边界 |

两者互补，而不是互相替代。

---

## 6. 局部障碍选择

### 6.1 点云体素到障碍球

执行循环从 `camera_driver_pointcloud` 地图读取 occupied cell centers：

```cpp
pointcloud_map->occupiedCellCenters(0u)
```

每个体素中心被近似成一个 sphere obstacle，半径为：

$$
r_{obs}
=
\frac{\sqrt{3}}{2}r_{voxel}
+
r_{padding}
$$

其中 `r_padding` 来自配置：

```yaml
obstacle_padding_m: 0.005
```

### 6.2 距离局部参考的筛选

不是所有点云体素都会进入 TrajOpt。执行循环会计算每个 occupied cell 到局部参考轨迹或当前机器人模型的距离，只保留靠近局部窗口的障碍：

```yaml
obstacle_selection_radius_m: 0.25
```

然后按距离从近到远排序，最多保留：

```yaml
max_obstacle_spheres: 24
```

这样做的目的：

- 控制 TrajOpt 问题规模
- 避免远处障碍干扰当前短窗口
- 把计算预算集中在当前局部路径附近

### 6.3 obstacle slot 机制

Tesseract environment 构建成本较高。当前实现使用 obstacle slot 机制：

1. 初始化环境时预先添加固定数量的障碍球 link
2. 每次局部优化 clone base environment
3. 用 `ChangeJointOriginCommand` 更新障碍球位置
4. 用 `ChangeLinkCollisionEnabledCommand` 开启或关闭对应障碍

未使用的 obstacle slot 会移动到远处：

```cpp
kInactiveObstacleOffsetM = 50.0
```

这样可以避免每次都重建完整 URDF / SRDF 环境。

### 6.4 环境缓存键

base environment 会按机器人与障碍 slot 配置缓存。缓存键包含：

- `robot_type`
- `planning_group`
- `base_link`
- `tip_link`
- `urdf_path`
- `srdf_path`
- obstacle slot 数量
- obstacle slot 半径量化值
- joint name 列表

当 obstacle slot 数量或半径变化时，会触发新的环境缓存项。

---

## 7. 输出与 NEO 集成

### 7.1 输出内容

局部优化成功后，`ReactiveTaskLocalPlanner::Output` 会包含：

- `target_pose`：下一步局部优化末端目标
- `target_twist`：由相邻优化 pose 差分得到的目标 twist
- `optimized_joint_target`：下一步关节目标
- `optimized_joint_velocity`：根据下一步关节目标差分得到的关节速度参考
- `optimized_joint_trajectory`：完整局部优化关节序列
- `target_poses`：完整局部优化末端 pose 序列
- `target_twists`：完整局部优化末端 twist 序列
- `trajectory_dt_sec`：局部轨迹采样间隔

### 7.2 pose / twist 生成

TrajOpt 输出的是关节轨迹。代码不会直接把这条关节轨迹发布给硬件，而是对每个优化后的关节点做 FK：

```cpp
input.joint_to_pose(q_i, &pose_i)
```

并通过相邻 pose 差分生成 target twist：

$$
\mathbf{v}_i =
\frac{\mathbf{p}_i - \mathbf{p}_{i-1}}{\Delta t}
$$

姿态部分使用旋转误差：

$$
\boldsymbol{\omega}_i =
\frac{
R_{i-1}\mathrm{Log}(R_{i-1}^{T}R_i)
}{\Delta t}
$$

这些 pose / twist 最终进入 `TaskVelocityGenerator` 和 NEO。

### 7.3 为什么不直接发布局部优化 q

局部 TrajOpt 的输出仍然是参考，不是最终命令。原因和 NEO 文档中的设计取舍一致：

- TrajOpt 运行频率低于控制频率
- TrajOpt 使用的是优化请求发起时的环境快照
- 执行时障碍和机器人状态可能已经变化
- NEO 还需要叠加实时 CBF、关节限位、速度边界、姿态偏好和可操作度

因此，局部优化结果会以两种方式影响 NEO：

1. 作为末端 `target_pose / target_twist`
2. 作为 `local_planner_joint_target`，进入 posture / joint preference 逻辑

最终硬件收到的仍然是 NEO 求解得到的 `qdot_cmd`。

---

## 8. 异步滚动集成

### 8.1 pending future

局部 TrajOpt 可能比单个 NEO tick 慢，因此执行循环用 `std::async` 启动后台任务：

```cpp
pending_local_planner_future = std::async(...)
```

主循环不会等待 TrajOpt 完成，而是继续使用当前已有参考。后台结果 ready 后再取出并检查。

### 8.2 generation 机制

为了避免旧结果覆盖新状态，执行上下文维护：

- `local_planner_generation`
- `pending_local_planner_generation`
- `pending_local_planner_tick`

如果后台结果返回时 generation 已经过期，该结果会被丢弃。

### 8.3 结果接管

结果有效时，执行循环缓存：

- `local_planner_target_poses`
- `local_planner_target_twists`
- `local_planner_joint_targets`
- `local_planner_start_time_sec`
- `local_planner_dt_sec`

之后每个 tick 根据当前 tracked reference time 计算局部轨迹 elapsed index，从局部轨迹中取一个 sample 作为当前 NEO 目标。

如果接管时发现剩余 sample 太少，结果会被认为过期并丢弃，避免刚接上局部轨迹就到尾部。

---

## 9. 代码组织

| 文件 | 职责 |
|------|------|
| `reactive_task_local_planner.hpp/.cpp` | 局部 TrajOpt 优化器主体，负责构建 Tesseract 程序、环境、障碍球和输出轨迹 |
| `reactive_task_execution_loop.cpp` | 决定何时触发局部优化、如何选择障碍、如何接管异步结果 |
| `reactive_task_execution_context.hpp` | 保存局部优化缓存、pending future、generation 和上一次输出 |
| `reactive_task_neo_pipeline.cpp` | 使用局部 joint target 生成 posture / joint preference |
| `reactive_task_terminal_policy.cpp` | 在局部 joint target 存在时生成 path-follow posture reference |
| `reactive_task_runtime_config.cpp` | 从 YAML 读取 local_planner 配置 |
| `reactive_task_config.yaml` | 局部优化器运行频率、窗口长度、障碍选择和碰撞项配置 |

核心类：

```cpp
ReactiveTaskLocalPlanner
```

核心入口：

```cpp
bool ReactiveTaskLocalPlanner::compute(
    const Input& input,
    Output* output) const;
```

---

## 10. 配置项

主要配置位于：

```yaml
reactive_task_controller:
  local_planner:
    type: trajopt
    frequency_hz: 20.0
    horizon_steps: 16
    dt_sec: 0.08
    enable_collision_cost: true
    enable_collision_constraint: true
    max_obstacle_spheres: 24
    obstacle_selection_radius_m: 0.25
    obstacle_padding_m: 0.005
```

含义：

| 配置 | 含义 |
|------|------|
| `type` | 局部优化器类型；当前实现为 `trajopt` |
| `frequency_hz` | 局部优化请求触发频率 |
| `horizon_steps` | 每次截取的局部参考窗口长度 |
| `dt_sec` | 局部窗口内参考采样间隔 |
| `enable_collision_cost` | 是否启用 TrajOpt collision cost |
| `enable_collision_constraint` | 是否启用 TrajOpt collision constraint |
| `max_obstacle_spheres` | 最多放入 TrajOpt 的障碍球数量，也是 obstacle slot 数量 |
| `obstacle_selection_radius_m` | 只选择离局部参考足够近的点云体素 |
| `obstacle_padding_m` | 给体素障碍球增加额外半径 |

---

## 11. 设计取舍与边界

### 11.1 为什么是局部滚动而不是每次全局重规划

全局规划器负责大范围拓扑搜索，通常计算更重，也不适合短周期反复运行。局部滚动优化器只看未来一小段，因此可以更频繁地运行，并且更适合在已有路径附近做平滑和局部避障修正。

### 11.2 为什么低频 TrajOpt + 高频 NEO

TrajOpt 可以处理更复杂的多步轨迹优化，但计算成本较高；NEO 只解单 tick 速度级 QP，计算更轻，更适合实时闭环。

因此当前架构把职责分开：

```text
TrajOpt:
    低频优化未来短窗口参考

NEO:
    高频生成当前 tick 可执行速度命令
```

### 11.3 为什么使用点云障碍球而不是完整 ESDF

局部 TrajOpt 的碰撞接口更自然地消费几何体。当前实现把点云 occupied cells 近似成 sphere obstacles，方便接入 Tesseract / Bullet collision。

代价是：

- 障碍数量需要限制
- 几何近似比 ESDF 粗糙
- 点云残影或噪声会影响优化

因此障碍球会经过选择半径、数量上限和 padding 控制。

### 11.4 不承诺的职责

局部滚动优化器不负责：

- 替代全局 RRT / TrajOpt 搜索完整路径
- 替代 NEO 做实时速度级安全约束
- 直接发布硬件命令
- 保证每次局部优化一定成功
- 对所有点云噪声做完整滤波
- 处理移动底盘自由度

---

## 12. 常见失败原因

### 12.1 输入无效

常见原因：

- `q_goal_valid == false`
- `joint_names` 为空
- `q_current / q_goal` 维度不一致
- `planning_group` 为空
- `joint_to_pose` 回调为空

### 12.2 环境构建失败

常见原因：

- `urdf_path` 为空或文件不存在
- SRDF / URDF 初始化 Tesseract environment 失败
- contact manager plugin 配置失败
- planning group 不存在且无法通过 joint names 创建
- obstacle slot 添加失败

### 12.3 TrajOpt 求解失败

常见原因：

- seed program 退化，局部 waypoint 太少
- 障碍约束过强
- 起点或终点被优化器移动超过容忍度
- 返回轨迹维度不匹配
- FK 失败或返回非有限 pose

### 12.4 运行期结果被丢弃

即使 `compute()` 成功，执行循环也可能丢弃结果：

- generation 已经过期
- terminal goal tracking 阶段不再接管局部结果
- 接管时局部轨迹剩余 sample 太少
- 输出 pose 或 joint target 出现非有限值

---

## 13. 后续改进方向

当前 Local TrajOpt 更适合作为过渡实现：它利用现有 Tesseract / TrajOpt 能力，快速获得短窗口平滑与几何避障能力，但本质上仍是一个异步、低频、局部批优化模块。

后续更合理的方向不是继续围绕 TrajOpt 做大量补丁，而是借鉴 **REMANI-Planner** 的在线滚动优化思想，先实现适用于固定基座机械臂的 arm-only 在线局部规划器，再逐步扩展到底盘-机械臂联合规划：

- 将局部规划从“异步批量优化一段参考”改为“随控制循环持续更新的滚动优化问题”
- 在优化变量中显式表达时间、速度、加速度以及必要的动力学 / 运动学约束
- 让避障、平滑、目标跟踪和可执行性在同一个在线局部规划问题中共同建模
- 减少对 obstacle slot、固定障碍球数量和 TrajOpt environment 重建 / clone 机制的依赖
- 支持更自然的动态障碍更新、旧问题热启动和过期计算取消
- 当前阶段优先优化机械臂关节轨迹 $\mathbf{q}_{arm}(t)$，接口设计上预留底盘状态 $\mathbf{x}_{base}(t)$，便于后续扩展为移动机械臂的 whole-body planner
- 底盘加入后，局部规划变量可从 arm-only 状态扩展为 $\mathbf{x}(t) = [x_{base}, y_{base}, \theta_{base}, \mathbf{q}_{arm}]$，使底盘位姿、机械臂构型、避障和末端目标在同一个滚动优化问题中协同求解

因此，本模块当前的定位应理解为：**在 REMANI-style 在线局部规划器落地之前，为 NEO 提供一个可运行、可调试的短窗口局部参考优化层；未来演进时应先形成 arm-only 在线局部规划器，再自然扩展到底盘-机械臂联合规划**。
