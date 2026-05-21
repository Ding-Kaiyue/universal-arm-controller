# NEO 反应式速度优化算法设计说明

> 本文档描述 Universal Arm Controller 系统中，算法层子模块 "NEO / Reactive QP" 的算法原理、代码组织、系统集成方式与使用边界。

## 1. 功能定位与系统层角色

NEO 模块解决的核心问题是：**在实时控制循环中，将末端任务位置和速度、关节限位、全身避障、姿态偏好、可操作性和平滑性要求统一建模为一个速度级二次规划问题，求解当前 tick 应该发送的关节速度命令**。

在系统架构中，NEO 属于 **算法层（Algorithm Layer）** 的实时优化子模块。它不直接访问硬件，不订阅 ROS 话题，也不决定任务阶段；它只接收当前控制状态和约束数据，构造并求解一个标准 QP：

$$
\min_x \frac{1}{2}x^T H x + g^T x
$$

$$
\text{s.t. } l \leq A x \leq u
$$

NEO 在系统内的角色是：

- **任务速度生成**：由当前末端位姿、目标位姿和目标 twist 生成 QP 使用的期望任务速度
- **任务速度跟踪**：尽量使关节速度产生的末端 twist 接近期望 twist
- **约束统一处理**：把关节速度边界、关节限位 CBF、障碍物 CBF 放进同一个 QP
- **实时反应式避障**：每个 tick 根据当前距离场和全身椭球模型重新生成障碍约束
- **姿态与平滑偏好**：通过二次项鼓励姿态参考、速度连续和可操作性提升
- **reactive_task 底层优化内核**：作为 reactive_task 的速度级 QP 控制核心，同时以算法层接口组织，便于未来被其他控制器或 whole-body controller 复用

### 1.1 适用范围声明

**本模块面向**：速度级实时控制问题。

**适用于**：

- 机械臂末端位姿跟踪
- 单臂 / 双臂 mapping 下的关节速度求解
- 关节限位附近的速度约束
- 基于距离场的全身反应式避障
- 局部规划轨迹的速度级跟踪与姿态引导

**不适用于**：

- 离线全局路径规划
- 时间参数化或轨迹插值
- 动力学级力矩控制
- 接触丰富的 whole-body inverse dynamics
- 直接处理移动底盘速度，除非上层把底盘自由度扩展进统一的 Jacobian 和速度边界

> [!NOTE]
> NEO 当前是 **velocity-level QP**，决策变量是关节速度，而不是关节位置、力矩或完整轨迹。

---

## 2. 算法模型与问题定义

### 2.1 输入与输出

**核心输入**：

- 当前关节位置： $\mathbf{q} \in \mathbb{R}^{n}$
- 任务雅可比： $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{m \times n}$
- 当前末端位姿： $T_{current} \in SE(3)$
- 目标末端位姿： $T_{target} \in SE(3)$
- 目标前馈 twist： $\mathbf{v}_{ff} \in \mathbb{R}^{m}$
- 由 `TaskVelocityGenerator` 合成的期望任务速度： $\mathbf{v}_{des} \in \mathbb{R}^{m}$
- 关节速度上下界： $\dot{\mathbf{q}}_{min}, \dot{\mathbf{q}}_{max} \in \mathbb{R}^{n}$
- 关节位置上下界： $\mathbf{q}_{min}, \mathbf{q}_{max} \in \mathbb{R}^{n}$
- 可选姿态速度参考： $\dot{\mathbf{q}}_{ref} \in \mathbb{R}^{n}$
- 可选上一 tick 速度： $\dot{\mathbf{q}}_{prev} \in \mathbb{R}^{n}$
- 可选 manipulability 梯度： $\nabla \log m(\mathbf{q}) \in \mathbb{R}^{n}$
- 可选障碍约束列表： $\{d_i, \mathbf{a}_i\}$

其中障碍约束中的 $\mathbf{a}_i$ 是法向雅可比：

$$
\mathbf{a}_i = \mathbf{n}_i^T \mathbf{J}_{p_i}(\mathbf{q})
$$

**输出**：

- 当前 tick 的关节速度命令：

$$
\dot{\mathbf{q}}^* \in \mathbb{R}^{n}
$$

在代码中，QP 的完整解向量可能还包含 task slack 和 obstacle slack，但控制器最终只取前 $n$ 维作为关节速度命令。

### 2.2 任务速度生成

NEO 的 QP 本身跟踪的是期望任务速度 $\mathbf{v}_{des}$，但在 reactive_task 中，$\mathbf{v}_{des}$ 不是直接由外部写死输入，而是由 `TaskVelocityGenerator` 根据目标 pose 和目标 twist 生成。

输入结构：

```cpp
struct TaskVelocityInput {
    Eigen::Isometry3d T_current;
    bool has_target_pose;
    Eigen::Isometry3d T_target;
    bool has_target_twist;
    Eigen::Matrix<double, 6, 1> target_twist;
};
```

输出结构：

```cpp
struct TaskVelocityOutput {
    Eigen::Matrix<double, 6, 1> v_des;
    Eigen::Vector3d e_pos;
    Eigen::Vector3d e_ori;
    Eigen::Matrix<double, 6, 1> v_ff;
    Eigen::Matrix<double, 6, 1> v_fb;
};
```

生成逻辑分为三步：

1. **前馈项**：如果输入包含目标 twist，则直接作为：

$$
\mathbf{v}_{ff} = \mathbf{v}_{target}
$$

2. **反馈项**：如果输入包含目标位姿，则根据当前位置误差和姿态误差生成反馈速度：

$$
\mathbf{e}_{pos} = \mathbf{p}_{target} - \mathbf{p}_{current}
$$

$$
\mathbf{v}_{fb,linear} = K_p \mathbf{e}_{pos}
$$

姿态误差使用从当前姿态到目标姿态的旋转向量，并映射到基坐标系表达：

$$
\mathbf{e}_{ori} = R_{current}\ \mathrm{Log}(R_{current}^T R_{target})
$$

$$
\mathbf{v}_{fb,angular} = K_o \mathbf{e}_{ori}
$$

此处 $Log(*)$ 表示 $SO(3)$ 李群上的对数映射，把旋转转成三维误差向量。


3. **合成与限幅**：

$$
\mathbf{v}_{des} = \mathbf{v}_{ff} + \mathbf{v}_{fb}
$$

然后分别对线速度和角速度做范数限幅：

$$
\|\mathbf{v}_{des,linear}\| \leq v_{linear,max}
$$

$$
\|\mathbf{v}_{des,angular}\| \leq v_{angular,max}
$$

同时，当位置误差或姿态误差小于 deadband 时，对应反馈项会被清零，避免目标附近抖动。

在 reactive_task 主循环中，对应数据流如下：
<div align="center">

![NEO pipeline](../../diagrams/neo_pipeline.png)

</div>

因此，更准确地说：

- `TaskVelocityGenerator` 负责“目标位姿 / 目标 twist -> 期望任务速度”
- `ReactiveQpBuilder` 负责“期望任务速度 + 约束 -> QP”
- `ReactiveQpSolver` 负责“QP -> 关节速度”


### 2.3 决策变量

当前实现中，QP 决策变量为：

$$
x = \begin{bmatrix}
\dot{\mathbf{q}} \\
\mathbf{s} \\
\mathbf{r}
\end{bmatrix}
$$

其中：

- $\dot{\mathbf{q}} \in \mathbb{R}^{n}$：关节速度
- $\mathbf{s} \in \mathbb{R}^{m}$：任务 slack，用于软化任务跟踪
- $\mathbf{r} \in \mathbb{R}^{k}$：障碍 CBF slack，可选；它不是障碍 CBF 约束本身，而是障碍约束的松弛量

这里没有单独的 joint-limit slack。关节限位 CBF 在当前实现中是硬约束，只会增加约束行，不会增加新的决策变量。障碍物 CBF 之所以额外引入 $\mathbf{r}$，是因为动态障碍、多障碍同时激活或距离场噪声可能在某个 tick 产生瞬态冲突；给障碍约束单独配置高权重 slack，可以避免 QP 直接不可行，同时仍然强烈惩罚违反避障约束。

若没有启用 obstacle slack，或当前没有 active obstacle rows，则 $\mathbf{r}$ 不出现在决策变量中，此时决策变量退化为：

$$
x = \begin{bmatrix}
\dot{\mathbf{q}} \\
\mathbf{s}
\end{bmatrix}
$$

### 2.4 主任务跟踪模型

速度级任务关系为：

$$
\mathbf{v}_{pred} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}
$$

引入 task slack 后，优化目标变成：

$$
\mathbf{J}\dot{\mathbf{q}} + \mathbf{s} \approx \mathbf{v}_{des}
$$

对应代价项：

$$
w_{task}\left\|\mathbf{J}\dot{\mathbf{q}} + \mathbf{s} - \mathbf{v}_{des}\right\|^2
$$

task slack 的作用是：当关节限位或障碍约束使任务无法完全满足时，允许任务残差存在，而不是直接让 QP 不可行。

---

## 3. QP 目标函数与 Hessian 构造

### 3.1 总体目标函数

NEO 的目标函数由多个二次项和一个线性 manipulability 项组成：

$$
\begin{aligned}
J(x) =
&\ w_{task}\left\|\mathbf{J}\dot{\mathbf{q}} + \mathbf{s} - \mathbf{v}_{des}\right\|^2 \\
&+ w_{qdot}\left\|\dot{\mathbf{q}}\right\|^2 \\
&+ w_{slack}\left\|\mathbf{s}\right\|^2 \\
&+ w_{smooth}\left\|\dot{\mathbf{q}} - \dot{\mathbf{q}}_{prev}\right\|^2 \\
&+ w_{posture}\left\|\mathbf{W}_{posture}(\dot{\mathbf{q}} - \dot{\mathbf{q}}_{ref})\right\|^2 \\
&- w_{manip}\left(\nabla \log m(\mathbf{q})\right)^T\dot{\mathbf{q}} \\
&+ J_{shell} + J_{tangent} + J_{obs\_slack}
\end{aligned}
$$

对应代码：

- `HessianBuilder::build()` 构造基础 Hessian 和 gradient
- `ReactiveQpBuilder::build()` 在需要 obstacle slack 时扩展 Hessian

### 3.2 任务跟踪项

展开：

$$
w_{task}\left\|\mathbf{J}\dot{\mathbf{q}} + \mathbf{s} - \mathbf{v}_{des}\right\|^2
$$

对 Hessian 和 gradient 的贡献为：

$$
H_{\dot{q}\dot{q}} += 2w_{task}\mathbf{J}^T\mathbf{J}
$$

$$
H_{\dot{q}s} += 2w_{task}\mathbf{J}^T
$$

$$
H_{ss} += 2w_{task}\mathbf{I}
$$

$$
g_{\dot{q}} += -2w_{task}\mathbf{J}^T\mathbf{v}_{des}
$$

$$
g_s += -2w_{task}\mathbf{v}_{des}
$$

代码位置：

```cpp
out_hessian.topLeftCorner(dof, dof).noalias() +=
    2.0 * w_task * input.jacobian_task.transpose() * input.jacobian_task;

out_hessian.topRightCorner(dof, task_dim).noalias() +=
    2.0 * w_task * input.jacobian_task.transpose();

out_gradient.head(dof).noalias() +=
    -2.0 * w_task * input.jacobian_task.transpose() * input.desired_twist;
```

### 3.3 关节速度正则项

$$
w_{qdot}\left\|\dot{\mathbf{q}}\right\|^2
$$

作用：

- 避免不必要的大关节速度
- 在冗余解中偏向小范数速度
- 提升 QP 数值稳定性

Hessian 贡献：

$$
H_{\dot{q}\dot{q}} += 2w_{qdot}\mathbf{I}
$$

### 3.4 task slack 惩罚项

$$
w_{slack}\left\|\mathbf{s}\right\|^2
$$

作用：

- 允许任务软化
- 但通过较大权重避免轻易牺牲任务跟踪

Hessian 贡献：

$$
H_{ss} += 2w_{slack}\mathbf{I}
$$

### 3.5 速度平滑项

$$
w_{smooth}\left\|\dot{\mathbf{q}} - \dot{\mathbf{q}}_{prev}\right\|^2
$$

作用：

- 降低相邻 tick 速度跳变
- 减少硬件层速度命令抖动
- 让 CBF 约束激活/失活时过渡更平滑

Hessian 和 gradient 贡献：

$$
H_{\dot{q}\dot{q}} += 2w_{smooth}\mathbf{I}
$$

$$
g_{\dot{q}} += -2w_{smooth}\dot{\mathbf{q}}_{prev}
$$

### 3.6 姿态偏好项

$$
w_{posture}\left\|\mathbf{W}_{posture}(\dot{\mathbf{q}} - \dot{\mathbf{q}}_{ref})\right\|^2
$$

其中 $\mathbf{W}_{posture}$ 是对角权重矩阵。

作用：

- 引导关节向偏好姿态或局部规划关节目标运动
- 在任务冗余空间中选择更自然的姿态
- terminal / local trajopt tracking 阶段可提高姿态参考权重

### 3.7 Manipulability 线性项

$$
- w_{manip}\left(\nabla \log m(\mathbf{q})\right)^T\dot{\mathbf{q}}
$$

这是一个线性目标项，等价于鼓励关节速度沿着提高 manipulability 的方向运动。

代码中明确使用的是：

$$
\nabla \log m(\mathbf{q})
$$

而不是 $\nabla m(\mathbf{q})$。使用 log gradient 的好处是尺度更稳定。

### 3.8 shell / tangential 引导项

`ReactiveQpBuildInput` 中还有以下可选项：

```cpp
Eigen::RowVectorXd shell_jacobian;
double shell_desired_rate;
double shell_weight_scale;
Eigen::MatrixXd tangential_jacobian;
Eigen::VectorXd tangential_desired_velocity;
double tangential_weight_scale;
```

它们用于预留的安全壳引导：

$$
J_{shell} =
w_{shell}\alpha_{shell}
\left\|\mathbf{J}_{shell}\dot{\mathbf{q}} - \dot{d}_{des}\right\|^2
$$

$$
J_{tangent} =
w_{shell}\alpha_{tan}
\left\|\mathbf{J}_{tan}\dot{\mathbf{q}} - \mathbf{v}_{tan,des}\right\|^2
$$

含义：

- `shell_jacobian`：障碍法向方向的速度映射，通常是 $\mathbf{n}^T\mathbf{J}_{point}$
- `shell_desired_rate`：期望的远离/接近安全壳速度
- `tangential_jacobian`：障碍切平面方向的速度映射
- `tangential_desired_velocity`：期望的切向绕行速度

当前 reactive_task 主链路主要依赖 obstacle CBF 约束，这些 shell/tangential 项是可选扩展接口。

---

## 4. 约束构造

### 4.1 标准约束形式

NEO 使用 OSQP 的标准线性约束形式：

$$
l \leq A x \leq u
$$

约束矩阵按以下 block 构造：

```text
block 1: 关节速度边界
block 2: task slack 边界
block 3: obstacle slack 边界（可选）
block 4: 关节限位 CBF
block 5: 障碍物 CBF
```

### 4.2 关节速度边界

对每个关节：

$$
\dot{q}_{min,i} \leq \dot{q}_i \leq \dot{q}_{max,i}
$$

矩阵形式：

$$
A_i x = \dot{q}_i
$$

这部分永远存在，是 QP 的基础 box constraint。

### 4.3 task slack 边界

对 task slack：

$$
-s_{max} \leq s_i \leq s_{max}
$$

对应配置：

```cpp
ReactiveQpBuildConfig::slack_abs_bound
```

作用：

- 防止 slack 无界吸收所有任务误差
- 保持任务跟踪和可行性之间的平衡

### 4.4 关节限位 CBF

关节下限安全函数：

$$
h_{low}(q_i) = q_i - (q_{min,i} + d_{safe})
$$

CBF 条件：

$$
\dot{h}_{low} \geq -\gamma_{low}h_{low}
$$

由于：

$$
\dot{h}_{low} = \dot{q}_i
$$

得到：

$$
\dot{q}_i \geq -\gamma_{low}\left(q_i - q_{min,i} - d_{safe}\right)
$$

关节上限安全函数：

$$
h_{up}(q_i) = (q_{max,i} - d_{safe}) - q_i
$$

由于：

$$
\dot{h}_{up} = -\dot{q}_i
$$

得到：

$$
\dot{q}_i \leq \gamma_{up}\left(q_{max,i} - d_{safe} - q_i\right)
$$

实现文件：

- `joint_limit_damper.cpp`
- `JointLimitDamper::countActiveRows()`
- `JointLimitDamper::appendConstraints()`

只有当关节进入 influence zone 时才生成对应约束：

$$
q_i \leq q_{min,i} + d_{influence}
$$

或：

$$
q_i \geq q_{max,i} - d_{influence}
$$

### 4.5 障碍物 CBF

对每个障碍约束，定义：

$$
h(q) = d(q) - d_{safe}
$$

其中 $d(q)$ 是机器人 link 椭球表面到障碍的距离。

CBF 条件：

$$
\dot{h} \geq -\gamma h
$$

因为：

$$
\dot{h} = \dot{d} = \mathbf{n}^T \mathbf{J}_{point} \dot{\mathbf{q}}
$$

所以约束为：

$$
\mathbf{n}^T \mathbf{J}_{point} \dot{\mathbf{q}}
\geq
-\gamma\left(d - d_{safe}\right)
$$

实现文件：

- `obstacle_damper.cpp`
- `ObstacleDamper::countActiveRows()`
- `ObstacleDamper::appendConstraints()`

当：

$$
d \leq d_{influence}
$$

时，障碍约束激活。

### 4.6 障碍 slack

障碍 CBF 可以启用独立 slack：

$$
\mathbf{a}_i\dot{\mathbf{q}} + r_i \geq -\gamma h_i
$$

$$
0 \leq r_i \leq r_{max}
$$

其目标函数惩罚：

$$
w_r\left\|\mathbf{r}\right\|^2
$$

设计原因：

- 多个障碍物约束可能瞬时冲突
- 如果没有 obstacle slack，QP 可能直接 infeasible
- 把 obstacle slack 和 task slack 分开，避免任务 slack 影响安全约束语义

所以，障碍约束不是严格硬CBF，而是 soft CBF / relaxed CBF。它通过高权重、上界受限的slack提高实时可行性，因此它不是唯一的硬安全边界。系统另有规划阶段 hard clearance 检查和运行阶段 hard collision monitor，但 hard clearance 尚未作为 QP 内部的硬 CBF 边界使用。

---

## 5. 全身障碍约束生成

NEO 本身只消费 `ObstacleConstraintInputList`，不直接读取点云或 ESDF。全身避障约束由 `BodyObstacleConstraintBuilder` 生成。

### 5.1 link-ellipsoid 碰撞模型

<table align="center">
  <tr>
    <td align="center"><img src="../../diagrams/robot_model.png" alt="robot model" width="450"></td>
    <td align="center"><img src="../../diagrams/robot_ellipsoid_model.png" alt="robot ellipsoid model" width="450"></td>
  </tr>
</table>

碰撞模型是一个 `LinkCollisionEllipsoidList`。列表里的每个条目都是一个 collision ellipsoid，同一个 `link_name` 可以对应多个椭球，用多个局部椭球分段覆盖较长或形状复杂的 link。

每个 collision ellipsoid 条目包含：

- link 名称
- link 局部坐标系下的椭球中心
- 三轴半径
- debug 名称

运行时先对每个 link 通过 FK 得到：

$$
T_{world}^{link}
$$

然后对挂在该 link 上的每个椭球，分别计算椭球中心世界坐标：

$$
\mathbf{p}_{world}^{ellipsoid} = T_{world}^{link}\mathbf{p}_{link}^{ellipsoid}
$$

### 5.2 距离场查询

对每个椭球中心查询距离场：

```cpp
distance_field->queryDistanceAndGradientBatch(query_points)
```

得到：

- 点到障碍的距离 $d_{center}$
- 距离场梯度 $\nabla d$

法向为：

$$
\mathbf{n} = \frac{\nabla d}{\|\nabla d\|}
$$

### 5.3 椭球等效半径

由于每个碰撞条目不是点，而是挂在 link 上的椭球，需要计算沿法向方向的等效半径：

$$
r_{eff} =
\sqrt{(r_x n_x^{link})^2 + (r_y n_y^{link})^2 + (r_z n_z^{link})^2}
$$

实际用于 CBF 的距离为：

$$
d = d_{center} - r_{eff}
$$

### 5.4 法向雅可比

对椭球中心点计算 point Jacobian：

$$
\mathbf{J}_{point} \in \mathbb{R}^{3 \times n}
$$

法向雅可比：

$$
\mathbf{a} = \mathbf{n}^T\mathbf{J}_{point}
$$

最终生成：

```cpp
ObstacleConstraintInput {
    normal_jacobian = n_world.transpose() * J_point.topRows(3)
    linear_jacobian = J_point.topRows(3)
    normal_world = n_world
    distance = distance_field_distance - effective_radius
}
```

### 5.5 约束过滤策略

`BodyObstacleConstraintBuilder` 不会把每个椭球查询结果都无条件变成 QP 约束，而是按以下策略过滤：

1. **椭球预处理过滤**

   在生成查询点之前，先跳过半径非法、半径非有限值、或找不到对应 FK link pose 的椭球条目。只有有效的 collision ellipsoid 才会进入距离场批量查询。

2. **距离场结果过滤**

   对每个椭球中心查询距离场后，只保留同时满足以下条件的结果：

   - 距离场 cell 已观测
   - 距离值有效
   - 距离值是有限数
   - 梯度有效且梯度范数足够大

   梯度会被归一化为世界系避障法向：

   $$
   \mathbf{n}_{world} = \frac{\nabla d}{\|\nabla d\|}
   $$

3. **自体最近点过滤**

   距离场返回的是“椭球中心到最近障碍”的距离和梯度。代码会反推出最近障碍点：

   $$
   \mathbf{p}_{nearest}
   =
   \mathbf{p}_{ellipsoid}
   -
   d_{center}\mathbf{n}_{world}
   $$

   如果这个最近障碍点落在机器人任意一个膨胀椭球内部，则认为它很可能来自机器人自身点云、传感器残留或距离场误检，跳过该约束：

   ```cpp
   pointInsideInflatedRobotEllipsoid(...)
   ```

   当前实现使用固定膨胀量：

   ```cpp
   kSelfNearestRejectPaddingM = 0.05
   ```

   也就是把每个椭球的三个半径都额外扩大 5 cm 后再做 inside test。判断方式等价于：

   $$
   \left\|
   \frac{(R_{world}^{link})^T(\mathbf{p}_{nearest} - \mathbf{p}_{center})}
        {\mathbf{r} + \mathbf{r}_{padding}}
   \right\|^2
   \leq 1
   $$

4. **几何与 Jacobian 有效性过滤**

   自体过滤通过后，再计算沿法向的椭球等效半径和椭球中心点 Jacobian。若等效半径、Jacobian、法向雅可比或最终距离中存在非有限值，或者 Jacobian 维度不匹配，也会跳过该条约束。

最终，只有通过上述过滤的 collision ellipsoid 才会生成 `ObstacleConstraintInput`。这套策略的目标是降低机器人自身点云、未观测区域、距离场异常和运动学异常被误当成外部障碍约束的概率。

---

## 6. QP 求解器

### 6.1 OSQP 集成

NEO 使用 OSQP 作为 QP 求解器。求解器封装在：

```text
src/arm_controller/src/algorithm/neo/reactive_qp_solver.hpp
src/arm_controller/src/algorithm/neo/reactive_qp_solver.cpp
```

求解流程：

1. 校验 `ReactiveQpProblem`
2. 取 Hessian 上三角
3. 将 dense Eigen 矩阵转为 compressed sparse column 格式
4. 填充 OSQP data
5. 设置 OSQP 参数
6. 调用 `osqp_setup`
7. 调用 `osqp_solve`
8. 检查 solved status
9. 拷贝解向量并做 finite / magnitude 检查

### 6.2 OSQP 参数

默认配置：

```cpp
struct ReactiveQpSolverConfig {
    bool warm_start{true};
    bool verbose{false};
    int max_iterations{4000};
    double absolute_tolerance{1e-5};
    double relative_tolerance{1e-5};
};
```

### 6.3 兼容 OSQP 新旧 API

代码通过：

```cpp
#if defined(OSQP_VERSION)
```

区分 legacy API 和新版 API，分别适配：

- `OSQPWorkspace`
- `OSQPSolver`

这样可以兼容不同系统环境中的 OSQP 版本。

### 6.4 失败处理

求解失败时，`ReactiveQpSolver::solve()` 返回 false，并通过 `error` 输出原因。主要失败来源：

- QP problem 结构不合法
- OSQP data 配置失败
- OSQP 返回非 solved 状态
- OSQP solution 为空
- 解中存在 NaN / Inf
- 解的绝对值异常过大

---

## 7. 代码框架与文件组织

NEO 模块位于：

```text
src/arm_controller/src/algorithm/neo/
```

核心文件如下：

| 文件 | 职责 |
|------|------|
| `reactive_qp_builder.hpp/.cpp` | QP 总装配入口，负责构造 Hessian、gradient、约束矩阵和上下界 |
| `hessian_builder.hpp/.cpp` | 构造目标函数二次项和线性项 |
| `reactive_qp_problem.hpp/.cpp` | QP 问题数据结构与结构合法性检查 |
| `reactive_qp_solver.hpp/.cpp` | OSQP 求解器封装 |
| `reactive_qp_validator.hpp/.cpp` | 输入和 QP problem 校验 |
| `joint_limit_adapter.hpp` | 关节限位数据结构和 damper 配置 |
| `joint_limit_damper.cpp` | 关节限位 CBF 约束生成 |
| `obstacle_damper.hpp/.cpp` | 障碍物 CBF 约束生成 |
| `body_obstacle_constraint_builder.hpp/.cpp` | 从 link-ellipsoid 列表、距离场和 Jacobian 生成 obstacle constraints |
| `task_velocity_generator.hpp/.cpp` | 由当前/目标末端位姿生成期望任务 twist |
| `manipulability_gradient.hpp/.cpp` | 计算 manipulability 梯度 |
| `joint_preference_loader.hpp/.cpp` | 从 YAML 加载关节偏好和 QP 配置 |

### 7.1 主要数据结构

#### ReactiveQpBuildInput

`ReactiveQpBuildInput` 是 QP 构造的主输入：

```cpp
struct ReactiveQpBuildInput {
    Eigen::VectorXd q_current;
    Eigen::MatrixXd jacobian_task;
    Eigen::VectorXd desired_twist;
    Eigen::VectorXd manipulability_gradient;
    Eigen::VectorXd posture_velocity_reference;
    Eigen::VectorXd previous_qdot_reference;
    Eigen::VectorXd posture_joint_weights;
    Eigen::VectorXd qd_min;
    Eigen::VectorXd qd_max;
    JointLimitData joint_limits;
    ObstacleConstraintInputList obstacle_constraints;
};
```

#### ReactiveQpBuildConfig

`ReactiveQpBuildConfig` 决定目标函数和约束是否启用：

```cpp
struct ReactiveQpBuildConfig {
    HessianBuilderConfig hessian;
    JointLimitDamperConfig joint_limit_damper;
    ObstacleDamperConfig obstacle_damper;
    bool enable_joint_limit_damper;
    bool enable_obstacle_damper;
    double slack_abs_bound;
    bool enable_obstacle_slack;
    double obstacle_slack_abs_bound;
    double obstacle_slack_weight;
};
```

#### ReactiveQpProblem

`ReactiveQpProblem` 是 OSQP 前的标准 QP 表达：

```cpp
struct ReactiveQpProblem {
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd constraint_matrix;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
};
```

---

## 8. 使用方式

### 8.1 最小调用流程

NEO 的最小使用流程如下：

```cpp
rq::ReactiveQpBuildInput input;
input.q_current = q_now;
input.jacobian_task = J_task;
input.desired_twist = v_des;
input.qd_min = qd_min;
input.qd_max = qd_max;
input.joint_limits = joint_limits;
input.previous_qdot_reference = qdot_prev;
input.posture_velocity_reference = qdot_posture_ref;
input.posture_joint_weights = posture_weights;
input.manipulability_gradient = grad_log_m;
input.obstacle_constraints = obstacle_constraints;

rq::ReactiveQpBuildConfig config = reactive_cfg.qp_build;

rq::ReactiveQpProblem problem;
std::string error;
if (!rq::ReactiveQpBuilder::build(input, config, problem, &error)) {
    // handle build failure
}

rq::ReactiveQpSolver solver;
Eigen::VectorXd solution;
if (!solver.solve(problem, solution, &error)) {
    // handle solve failure
}

Eigen::VectorXd qdot_cmd = solution.head(q_now.size());
```

### 8.2 在 reactive_task 中的集成

在 reactive_task 控制器中，NEO 并不是直接散落在主循环里，而是通过：

```text
ReactiveTaskNeoPipeline
ReactiveTaskSafetyPolicy
```

协作使用。

典型数据流如下：

<div align="center">

![robot model](../../diagrams/neo_reactive_task_integration.png)

</div>

### 8.3 配置来源

NEO 配置来自：

```text
src/arm_controller/config/reactive_task_config.yaml
```

主要配置块：

```yaml
neo_example:
  task_velocity:
    kp_pos: [...]
    ko_ori: [...]
    max_linear_speed: ...
    max_angular_speed: ...

  qp_build:
    enable_joint_limit_damper: true
    enable_obstacle_damper: true
    slack_abs_bound: 1.0

    hessian:
      task_tracking_weight: ...
      joint_velocity_weight: ...
      slack_weight: ...
      qdot_smoothing_weight: ...
      posture_weight: ...
      manipulability_weight: ...

    joint_limit_damper:
      safety_distance: ...
      influence_distance: ...
      cbf_gain_lower: ...
      cbf_gain_upper: ...

    obstacle_damper:
      safety_distance: ...
      influence_distance: ...
      cbf_gain: ...

  manipulability:
    determinant_damping: ...
```

---

## 9. 设计取舍与边界

### 9.1 为什么使用 QP

相比阻尼伪逆，QP 的优势是可以自然表达：

- 关节速度上下界
- 关节限位 CBF
- 障碍物 CBF
- task slack
- obstacle slack
- 姿态偏好和平滑项
- 可操作度

这些约束和目标在一个统一优化问题中求解，避免在伪逆后再做多层裁剪导致方向不可控。

### 9.2 为什么是速度级而不是轨迹级

速度级 QP 的优点：

- 每个 tick 可快速重算
- 能直接响应动态障碍
- 与实时控制循环天然匹配
- 适合 CBF 这类一阶安全约束

代价：

- 只保证局部即时行为
- 不保证全局最优路径
- 需要 global planner / local planner 提供长期参考

因此，在 reactive_task 中，NEO 和 RRT / TrajOpt 是互补关系：

```text
RRT / TrajOpt:
    给出较长期的可行参考

NEO:
    在每个 tick 上安全、平滑、反应式地跟踪参考
```

### 9.3 为什么不直接发布规划器输出的 q

global planner / local planner 的输出通常是较低频的参考轨迹或候选关节路径，它回答的是“从当前状态到目标，大体应该经过哪里”。reactive_task 没有直接把规划器输出的 $\mathbf{q}_{ref}$ 当成硬命令发布，而是先通过 FK 得到参考末端位姿 / twist，再交给 NEO 求解当前 tick 的 $\dot{\mathbf{q}}$，主要有几个原因：

1. **规划结果是参考，不是实时安全命令**

   规划器生成轨迹时使用的是某一时刻的地图、机器人状态和约束快照。执行过程中障碍物、点云、距离场、关节状态都会变化。如果直接发布规划器给出的 $\mathbf{q}$，控制命令会绕过 NEO 里的实时 CBF、速度边界、关节限位和 slack 机制。

2. **速度命令更适合实时闭环修正**

   NEO 每个 tick 根据当前 $\mathbf{q}$、当前 Jacobian、当前障碍约束重新求 $\dot{\mathbf{q}}$。这样即使参考轨迹不变，实际命令也会随着机器人反馈和环境变化连续调整。直接发布 $\mathbf{q}_{ref}$ 更像开环轨迹跟随，对动态障碍和局部误差的反应能力弱。

3. **末端任务语义比某个 IK 解更稳定**

   同一个末端位姿可能对应多个关节解。规划器输出的 $\mathbf{q}_{ref}$ 只是其中一个解，不一定在当前 tick 下最适合避障、远离关节限位或保持可操作度。把参考先表达成末端位姿 / twist，再由 NEO 在当前约束下选择 $\dot{\mathbf{q}}$，可以保留任务目标，同时允许关节空间自由度用于安全和偏好优化。

4. **避免规划层和控制层争夺最终命令权**

   如果规划器直接发布 $\mathbf{q}$，那么局部避障、速度限幅、姿态偏好、可操作度优化都只能在发布后再裁剪或覆盖，容易出现命令方向被破坏的问题。当前设计让规划器提供长期参考，NEO 负责实时可执行命令，职责边界更清晰。

因此，reactive_task 中的数据流更接近：

```text
planner q / Cartesian reference
    -> FK / local reference sample
    -> target pose + target twist
    -> NEO velocity-level QP
    -> qdot command
```

这并不表示 $\mathbf{q}_{ref}$ 没有用。它仍然可以作为 joint anchor、posture reference 或局部参考的一部分参与 NEO 构造，但最终发给硬件的是经过实时约束优化后的 $\dot{\mathbf{q}}$。

### 9.4 不承诺的职责

NEO 不负责：

- 构建地图或点云滤波
- 选择全局路径
- 生成局部 TrajOpt 轨迹
- 读取硬件反馈
- 发布关节速度
- 判断任务 phase
- 处理移动底盘模型

这些由控制器层、规划层或未来 whole-body controller 负责。

---

## 10. 调试与常见失败原因

### 10.1 Build 阶段失败

常见原因：

- `q_current / qd_min / qd_max / joint_limits` 维度不一致
- `jacobian_task.cols() != q_current.size()`
- `desired_twist.size() != jacobian_task.rows()`
- obstacle constraint 的 `normal_jacobian` 维度错误
- joint limit CBF 与速度边界冲突
- 关闭 obstacle slack 后，多障碍 CBF 不可行

### 10.2 Solve 阶段失败

常见原因：

- Hessian 或 constraint matrix 含 NaN / Inf
- OSQP data 结构非法
- OSQP 返回 non-solved status
- 约束上下界存在 `lb > ub`
- 问题尺度异常导致数值不稳定

### 10.3 运行时表现异常

| 现象 | 可能原因 |
|------|----------|
| 任务跟踪慢 | `task_tracking_weight` 太低，或 `joint_velocity_weight` / `posture_weight` 太高 |
| 速度抖动 | `qdot_smoothing_weight` 太低，障碍约束频繁激活/失活 |
| 靠近关节限位停住 | joint limit CBF 与任务方向冲突 |
| 避障过保守 | `obstacle_damper.safety_distance` 或 `influence_distance` 太大 |
| QP infeasible | obstacle slack 关闭，或 CBF 约束与速度边界冲突 |
| 姿态偏好太强 | `posture_weight` 或 posture joint weights 太大 |
| IK 分支跳变，没有走最优路径 | `posture_weight` 或 posture joint weights 太小 |

---

## 11. 扩展方向

### 11.1 Whole-body QP

未来接入底盘时，NEO 的核心形式可以扩展为：

$$
u =
\begin{bmatrix}
\mathbf{v}_{base} \\
\dot{\mathbf{q}}_{arm}
\end{bmatrix}
$$

并将任务雅可比扩展为：

$$
\mathbf{J}_{whole} =
\begin{bmatrix}
\mathbf{J}_{base} & \mathbf{J}_{arm}
\end{bmatrix}
$$

NEO 本身并不要求变量一定是机械臂关节速度；真正需要扩展的是上层对状态、Jacobian、速度边界和命令输出的组织。

### 11.2 更强的安全控制

可进一步补强：

- 高阶 CBF
- CLF-CBF-QP
- 动态障碍速度预测
- obstacle slack 自适应权重
- 距离场置信度加权

### 11.3 求解器优化

当前每次 `solve()` 都重新配置 OSQP workspace。未来如果问题维度和稀疏结构稳定，可以考虑：

- workspace 复用
- warm start 解复用
- 只更新数值不重建结构
- sparse Hessian/constraint 直接构造

这可以降低实时控制尾延迟。
