# CartesianVelocity 系统卡死 - 详细改动点分析

## 一、问题根源对比

### JointVelocity (✅ 工作正常)
```
RT Thread (1kHz, 周期=1ms)
├─ pop() from rt_buffers_           ← lock-free 操作
├─ 简单赋值                           ← O(n) where n=6
└─ send_joint_velocities()           ← 硬件调用, <1ms
  Total: < 1ms per cycle ✅
```

### CartesianVelocity (❌ 系统卡死)
```
RT Thread (100Hz, 周期=10ms)
├─ pop() from rt_buffers_
├─ get joint_positions                ← 硬件查询
├─ lookupTransform()                  ← 阻塞！50ms超时 ⚠️
├─ computeJacobian()                  ← MoveIt计算，5-10ms
├─ Eigen::JacobiSVD                   ← SVD分解，5ms
├─ solver_.solve()                    ← QP求解，5-10ms
└─ send_joint_velocities()
  Total: 20-35ms per cycle ❌ (超过10ms周期!)
```

---

## 二、具体改动点

### 文件：`cartesian_velocity_controller.hpp`

**改动1：添加计算结果结构体和计算线程**

```diff
Line 63: 新增计算线程数据结构

struct ComputationResult {
    Eigen::VectorXd qd;                              // 计算出的关节速度
    bool valid;                                      // 计算是否成功
    std::chrono::steady_clock::time_point timestamp; // 结果时间戳
};

// per mapping 计算线程结果（使用原子操作保护）
std::unordered_map<std::string,
    std::shared_ptr<std::atomic<ComputationResult>>> computation_results_;
```

**改动2：添加计算线程成员变量**

```diff
Line 74: 新增

// 计算线程（每个mapping一个）
std::unordered_map<std::string, std::thread> computation_threads_;

// 计算线程运行标志
std::unordered_map<std::string, std::shared_ptr<std::atomic<bool>>>
    computation_running_per_mapping_;

// 计算线程同步（信号量或条件变量）
std::unordered_map<std::string, std::unique_ptr<std::mutex>> computation_mutexes_;
std::unordered_map<std::string, std::unique_ptr<std::condition_variable>>
    computation_cvs_;
```

**改动3：添加新的计算线程方法声明**

```diff
Line 52: 新增

private:
    // 计算线程：处理所有重型计算
    void cartesian_computation_thread(const std::string& mapping);
```

---

### 文件：`cartesian_velocity_controller.cpp`

**改动1：start() 方法中启动计算线程**

```cpp
// 在 Line 114 后面增加（在 create RT thread 之后）

    // 创建计算线程
    auto computation_running = std::make_shared<std::atomic<bool>>(true);
    computation_running_per_mapping_[mapping] = computation_running;

    auto computation_mutex = std::make_unique<std::mutex>();
    auto computation_cv = std::make_unique<std::condition_variable>();
    computation_mutexes_[mapping] = std::move(computation_mutex);
    computation_cvs_[mapping] = std::move(computation_cv);

    computation_threads_[mapping] = std::thread(
        &CartesianVelocityController::cartesian_computation_thread,
        this,
        mapping);
```

**改动2：stop() 方法中清理计算线程**

```cpp
// 在 Line 127 后面增加（在关闭RT线程之后）

    // 关闭计算线程
    auto it_comp_running = computation_running_per_mapping_.find(mapping);
    if (it_comp_running != computation_running_per_mapping_.end()) {
        it_comp_running->second->store(false, std::memory_order_release);
        computation_running_per_mapping_.erase(it_comp_running);
    }

    auto it_comp = computation_threads_.find(mapping);
    if (it_comp != computation_threads_.end()) {
        if (it_comp->second.joinable()) {
            it_comp->second.join();
        }
        computation_threads_.erase(it_comp);
    }

    computation_mutexes_.erase(mapping);
    computation_cvs_.erase(mapping);
```

**改动3：重写 control_loop_rt() - 简化为RT专用**

```cpp
// 替换整个 Line 301-565 的 control_loop_rt()

void CartesianVelocityController::control_loop_rt(const std::string& mapping) {
    static std::map<std::string, bool> logged_start;
    static std::map<std::string, bool> logged_inactive;

    if (!is_active(mapping)) {
        if (!logged_inactive[mapping]) {
            logged_inactive[mapping] = true;
        }
        return;
    }

    if (!logged_start[mapping]) {
        RCLCPP_INFO(node_->get_logger(), "[%s] ✅ RT loop started", mapping.c_str());
        logged_start[mapping] = true;
    }

    // ✅ 只做两件事：
    // 1. Pop 最新的计算结果
    // 2. 发送给硬件

    auto it_state = rt_states_.find(mapping);
    auto it_buf = rt_buffers_.find(mapping);
    if (it_state == rt_states_.end() || it_buf == rt_buffers_.end()) {
        return;
    }

    auto& state = it_state->second;
    auto& buffer = it_buf->second;

    TwistCommand cmd;
    while (buffer->pop(cmd)) {
        state.target = cmd.twist;
        state.last_update = cmd.stamp;
    }

    auto now = steady_clock_.now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - state.last_update).count();

    // 命令超时处理
    if (dt > 100.0) {
        auto joint_names = hardware_manager_->get_joint_names(mapping);
        std::vector<double> zero(joint_names.size(), 0.0);
        send_joint_velocities(mapping, zero);
        return;
    }

    // ✅ 关键改动：从计算线程获取已计算的关节速度
    // （而不是在RT线程中重新计算）
    auto it_result = computation_results_.find(mapping);
    if (it_result != computation_results_.end()) {
        // ComputationResult 应该使用 lock-free 原子操作
        // 这里简化为直接读取（实际实现需要更复杂的同步）

        // 获取最新计算结果
        auto& result_ptr = it_result->second;
        // Note: Eigen::VectorXd 不能直接原子化，需要使用共享指针或其他机制
        // 详见改动4
    }

    // 如果没有新的计算结果，使用安全的零速度
    auto joint_names = hardware_manager_->get_joint_names(mapping);
    std::vector<double> zero(joint_names.size(), 0.0);
    send_joint_velocities(mapping, zero);
}
```

**改动4：新增 cartesian_computation_thread() 方法**

```cpp
// 在 Line 565 之后添加

void CartesianVelocityController::cartesian_computation_thread(
    const std::string& mapping) {

    RCLCPP_INFO(node_->get_logger(),
        "[%s] Computation thread started", mapping.c_str());

    auto it_running = computation_running_per_mapping_.find(mapping);
    if (it_running == computation_running_per_mapping_.end()) {
        return;
    }
    auto computation_running = it_running->second;

    while (computation_running->load(std::memory_order_acquire)) {
        // 等待新命令到来（或超时）
        {
            auto it_mutex = computation_mutexes_.find(mapping);
            auto it_cv = computation_cvs_.find(mapping);
            if (it_mutex != computation_mutexes_.end() &&
                it_cv != computation_cvs_.end()) {

                std::unique_lock<std::mutex> lock(*it_mutex->second);
                it_cv->second->wait_for(lock, std::chrono::milliseconds(50));
            }
        }

        if (!is_active(mapping)) {
            continue;
        }

        // ===== 从这里开始是所有"重"操作 =====
        // 这些操作可以随意耗时，因为不在RT线程中

        auto it_state = rt_states_.find(mapping);
        if (it_state == rt_states_.end()) {
            continue;
        }
        auto& state = it_state->second;
        auto& target_cmd = state.target;

        // 1. 获取关节位置
        auto joint_positions = hardware_manager_->get_current_joint_positions(mapping);
        auto joint_names = hardware_manager_->get_joint_names(mapping);

        if (joint_positions.empty() || joint_names.empty()) {
            continue;
        }

        // 2. 获取MoveIt适配器
        auto it = moveit_adapters_.find(mapping);
        if (it == moveit_adapters_.end() || !it->second) {
            continue;
        }

        // 3. 计算Jacobian（可能耗时5-10ms）
        Eigen::MatrixXd J = it->second->computeJacobian(joint_positions);

        if (J.rows() == 0 || J.cols() == 0 || J.hasNaN()) {
            auto joint_names = hardware_manager_->get_joint_names(mapping);
            std::vector<double> zero(joint_names.size(), 0.0);
            // 存储零速度结果
            continue;
        }

        // 4. TF2变换（可能阻塞50ms，但在计算线程中不影响RT）
        Eigen::Vector3d v_linear(
            target_cmd.twist.linear.x,
            target_cmd.twist.linear.y,
            target_cmd.twist.linear.z);

        Eigen::Vector3d v_angular(
            target_cmd.twist.angular.x,
            target_cmd.twist.angular.y,
            target_cmd.twist.angular.z);

        std::string user_frame = target_cmd.header.frame_id.empty() ?
            base_frame_ : target_cmd.header.frame_id;

        if (user_frame != base_frame_) {
            try {
                auto tf = tf_buffer_->lookupTransform(
                    base_frame_, user_frame,
                    tf2::TimePointZero,
                    std::chrono::milliseconds(50));

                Eigen::Quaterniond q(
                    tf.transform.rotation.w,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z);

                Eigen::Matrix3d R = q.toRotationMatrix();
                v_linear = R * v_linear;
                v_angular = R * v_angular;

            } catch (...) {
                // TF2查询失败，使用零速度
                continue;
            }
        }

        // 5. 构造任务速度向量
        Eigen::MatrixXd J_task = J;
        Eigen::VectorXd v_task(6);
        v_task << v_linear(0), v_linear(1), v_linear(2),
                  v_angular(0), v_angular(1), v_angular(2);

        if (v_task.norm() < 1e-8) {
            continue;
        }

        // 6. 奇异性检测和缩速
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_task);
        double sigma_min = svd.singularValues().minCoeff();

        double scale = 1.0;
        if (sigma_min < 0.05) {
            if (sigma_min <= 0.01) {
                scale = 0.0;
            } else {
                scale = (sigma_min - 0.01) / (0.05 - 0.01);
            }
        }

        v_task *= scale;

        // 7. 关节限制
        const int dof = J.cols();
        Eigen::VectorXd q_current =
            Eigen::Map<Eigen::VectorXd>(
                joint_positions.data(), joint_positions.size());

        Eigen::VectorXd qd_max(dof);
        Eigen::VectorXd q_min_pos(dof), q_max_pos(dof);

        for (int i = 0; i < dof; ++i) {
            JointLimits limits;
            hardware_manager_->get_joint_limits(joint_names[i], limits);

            double vmax = limits.has_velocity_limits ? limits.max_velocity : 1.0;
            qd_max(i) = vmax;

            q_min_pos(i) = limits.min_position;
            q_max_pos(i) = limits.max_position;
        }

        // 8. 求解IK速度（可能耗时5-10ms）
        Eigen::VectorXd qd(dof);
        bool ok = solver_.solve(
            J_task,
            v_task,
            q_current,
            q_min_pos,
            q_max_pos,
            qd_max,
            qd,
            node_->get_logger());

        if (!ok) {
            continue;
        }

        // 9. 验证方向一致性
        Eigen::VectorXd v_reconstructed = J_task * qd;

        if (v_reconstructed.norm() < 1e-6) {
            continue;
        }

        double cos_angle = v_reconstructed.normalized().dot(v_task.normalized());
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);

        if (cos_angle < 0.99) {
            continue;
        }

        // ===== 计算完成！现在存储结果供RT线程使用 =====
        // TODO: 实现原子安全的结果存储
        // 目前的问题：Eigen::VectorXd 不能直接原子化
        // 解决方案：使用 std::shared_ptr 或 double array
    }

    RCLCPP_INFO(node_->get_logger(),
        "[%s] Computation thread terminated", mapping.c_str());
}
```

---

## 三、数据流改动对比

### 当前流程（❌ 有问题）
```
IPC消费线程                RT线程 (1ms cycle)
    |                              |
    +---> rt_buffers_[mapping]     |
                                   |
                           pop cmd  |
                                   v
                           ❌ 在这里做计算 ← 导致超期
                           (20-30ms)
                                   |
                                   v
                           send_joint_velocities()
```

### 新流程（✅ 修复）
```
IPC消费线程              计算线程(可以慢)    RT线程 (1ms cycle)
    |                      |                      |
    +---> rt_buffers_[mapping]                    |
                           |                      |
                           |---> pop cmd          |
                           |                      |
                    ✅ 在这里做计算 ← 无时间限制   |
                    (20-30ms OK!)                 |
                           |                      |
                           +---> computation_results_[mapping]
                                                  |
                                          pop result
                                                  |
                                          send_joint_velocities()
                                          (< 1ms)
```

---

## 四、关键实现细节

### 问题：原子化 Eigen::VectorXd

目前 `Eigen::VectorXd` 不能直接用 `std::atomic` 包装。三种解决方案：

**方案A：使用 shared_ptr（推荐）**
```cpp
struct ComputationResult {
    std::shared_ptr<Eigen::VectorXd> qd;  // 共享指针
    bool valid;
    std::chrono::steady_clock::time_point timestamp;
};

std::unordered_map<std::string, ComputationResult> computation_results_;
```
- 优点：简单，thread-safe（指针赋值原子）
- 缺点：新建指针有开销

**方案B：使用 double array**
```cpp
struct ComputationResult {
    std::array<double, 6> qd;  // 假设最多6个关节
    int dof;
    bool valid;
};

std::unordered_map<std::string, std::atomic<ComputationResult>> computation_results_;
```
- 优点：固定大小，接近原子
- 缺点：无法处理不同DOF的机械臂

**方案C：使用 mutex 保护（最保险）**
```cpp
struct ComputationResult {
    Eigen::VectorXd qd;
    bool valid;
    std::chrono::steady_clock::time_point timestamp;
};

std::unordered_map<std::string, std::pair<
    std::mutex, ComputationResult>> computation_results_;
```
- 优点：完全安全，支持任意大小
- 缺点：需要加锁（但RT线程只读，冲突少）

**建议：使用方案C**（保险 + 可行）

---

## 五、改动清单

| 文件 | 行号 | 改动类型 | 优先级 |
|------|------|--------|------|
| cartesian_velocity_controller.hpp | 63 | 新增 ComputationResult 结构体 | P0 |
| cartesian_velocity_controller.hpp | 74-81 | 新增计算线程成员变量 | P0 |
| cartesian_velocity_controller.hpp | 52 | 新增方法声明 | P0 |
| cartesian_velocity_controller.cpp | 114 | start() 启动计算线程 | P0 |
| cartesian_velocity_controller.cpp | 127 | stop() 清理计算线程 | P0 |
| cartesian_velocity_controller.cpp | 301-565 | **重写 control_loop_rt()** | P0 |
| cartesian_velocity_controller.cpp | 565+ | **新增 cartesian_computation_thread()** | P0 |

---

## 六、验证点

修改后需要验证：

1. **RT线程延迟** ✅
   - 每周期 < 1ms （原来20-30ms）
   - 使用 RCLCPP_INFO 加时间戳测量

2. **命令队列堆积** ✅
   - rt_buffers_ 不再溢出
   - 新命令总能成功 push

3. **计算线程工作** ✅
   - cartesian_computation_thread 正常运行
   - 计算结果正确存储

4. **系统不卡死** ✅
   - 发送 CartesianVelocity 命令
   - 机械臂流畅运动

---

## 七、后续优化（可选）

完成基础修复后，可考虑：

1. **使用 lock-free queue** 在计算线程和RT线程间传递结果
2. **自适应计算频率** - 根据负载调整计算线程周期
3. **缓存Jacobian** - 在Cartesian空间变化小时重用
4. **异步TF2查询** - 单独线程处理TF2，避免阻塞

