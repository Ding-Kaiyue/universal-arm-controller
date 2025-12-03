# IPC API 状态转移使用说明

## 核心设计

**队列化 + 自动转移**：多个命令可以快速连续发送，执行进程自动按顺序执行并完成状态转移。

### 状态定义

```cpp
enum class ControllerState : int32_t {
    IDLE = 0,       // 闲置
    MOVING_J = 1,   // 关节运动中
    MOVING_L = 2,   // 直线运动中
    MOVING_C = 3,   // 圆弧运动中
    HOLDING = 4,    // 保持位置（自动过渡状态）
    ERROR = -1,     // 出错
};
```

## 用户使用方式（队列化）

### 场景1：简单的单个命令

```cpp
auto& api = ArmControllerAPI::getInstance();
api.initialize();

// 发送 MoveJ 命令（立即返回）
if (api.moveJ({0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, "left_arm")) {
    std::cout << "✅ MoveJ 命令已入队" << std::endl;
}
```

### 场景2：命令序列（不需要 sleep！）

```cpp
// 快速连续发送多个命令（都会被加入队列）
api.moveJ({0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, "left_arm");
std::cout << "✅ MoveJ 命令已入队" << std::endl;

api.moveL(0.5, 0.6, 0.7, 0, 0, 0, 1, "left_arm");
std::cout << "✅ MoveL 命令已入队（将在 MoveJ 完成后自动转移执行）" << std::endl;

api.moveC({0.4, 0.5, 0.6, 0, 0, 0, 1, 0.5, 0.5, 0.5, 0, 0, 0, 1}, "left_arm");
std::cout << "✅ MoveC 命令已入队" << std::endl;

// 立即返回，命令在后台执行
```

**执行流程（自动在执行进程中进行）**：
1. 取出 MoveJ 命令，执行
2. MoveJ 完成 → 自动转移到 HOLDING 状态
3. 位置稳定 → 检查是否有下一个命令
4. 如果有 MoveL，执行状态转移逻辑
5. 执行 MoveL，以此类推

## 架构细节

### 1. ControllerStateManager（客户端状态管理）

**位置**：API 进程内存（每个 mapping 一个实例）

**职责**：
- 追踪当前控制器状态
- 管理状态转移逻辑
- 处理 hook 状态（HOLDING）

```cpp
class ControllerStateManager {
    ControllerState getCurrentState() const;
    bool transitionToState(ControllerState target, timeout);
    void enterHookState(ControllerState target);
    void updateFromExecutor(const ExecutorControllerState&);
};
```

### 2. 状态转移规则

```
规则：MOVING_J/L/C → 新的 MOVING_J/L/C

步骤1：检测冲突
  ├─ 如果已经是目标状态 → 直接返回成功
  ├─ 如果在 hook 状态 → 更新目标，等待执行完成
  └─ 如果需要停止 → 进入 HOLDING 状态

步骤2：等待执行进程确认
  └─ 执行进程反馈已进入 HOLDING → 退出 hook 状态

步骤3：发送新命令
  └─ moveJ/moveL/moveC 发送命令到队列
```

### 3. 执行进程该怎么做？

执行进程应该：

1. **检测 STOP 命令**
   ```cpp
   if (cmd.get_mode() == "STOP") {
       // 调用当前控制器的 stop() 方法
       current_controller->stop();
   }
   ```

2. **进入 HOLDING 状态**
   ```cpp
   // 位置保持，等待位置稳定
   // 然后反馈执行状态：HOLDING
   ```

3. **反馈状态到共享内存**
   ```cpp
   ExecutorControllerState state;
   state.state = (int32_t)ControllerState::HOLDING;
   update_to_shm(state);
   ```

## 工作流程详解

### API 层（非阻塞）
```cpp
api.moveJ(...);   // → 更新目标状态为 MOVING_J → 命令入队 → 立即返回
api.moveL(...);   // → 更新目标状态为 MOVING_L → 命令入队 → 立即返回
```

**特点**：API 调用是非阻塞的（timeout=0），只是更新状态管理器的目标状态和入队命令。

### 执行进程（阻塞式处理）
```cpp
while (true) {
    cmd = dequeue_command();  // 取出第一个命令（MoveJ）
    execute(cmd);              // 执行

    // 命令完成后，检查是否需要状态转移
    if (has_next_command() && need_state_transition()) {
        enter_holding_state();  // 进入 HOLDING（位置保持）
        wait_for_stability();   // 等待稳定
    }

    // 继续取下一个命令
}
```

## 关键特性

### ✅ 快速入队（no blocking）
```cpp
// 所有命令都会立即返回，不会阻塞 API
api.moveJ(...);  // 返回：✅ 已入队
api.moveL(...);  // 返回：✅ 已入队
api.moveC(...);  // 返回：✅ 已入队
// 总耗时：~1ms（只是验证 + 入队）
```

### ✅ 自动状态转移（在执行进程中）
```cpp
// MoveJ 执行完 → 自动转移到 HOLDING → MoveL 开始
// 不需要用户调用 stop() 或其他方法
```

### ✅ 过渡状态（HOLDING）保证安全
- 位置锁定，机械臂停留在原位置
- 充足的时间让系统准备下一个运动
- 类似 ROS 的 `hold_state_controller`

## 扩展建议

### 1. 添加状态查询接口
```cpp
ControllerState ArmControllerAPI::getCurrentControllerState(const std::string& mapping) {
    auto mgr = getStateManager(mapping);
    return mgr->getCurrentState();
}
```

### 2. 添加超时处理
```cpp
// 如果超过 5 秒还没进入 HOLDING，触发错误恢复
if (!state_mgr->transitionToState(..., seconds(5))) {
    // 考虑紧急停止？
}
```

### 3. 添加过渡监视线程
```cpp
// 后台线程定期检查 hook 状态是否卡住
void status_monitoring_thread() {
    while (running) {
        for (auto& [mapping, mgr] : state_managers_) {
            if (mgr->isInHookState()) {
                // 检查是否已经等待超过阈值
            }
        }
        sleep(100ms);
    }
}
```

## 与 ROS 层的对比

| 特性 | ROS | IPC |
|------|-----|-----|
| 自动状态转移 | ✅ 框架处理 | ✅ ControllerStateManager |
| Hook 状态 | ✅ hold_state_controller | ✅ HOLDING 枚举值 |
| 命令中断 | ✅ 自动 | ✅ 需要调用 API |
| 代码复杂度 | 低 | 中（但已隐藏在 API 内） |

## 常见问题

### Q: 如果执行进程没有反馈 HOLDING 状态怎么办？

A: API 会超时（默认 5 秒），返回错误。这表示执行进程可能卡住了，需要重启或重新检查。

### Q: 为什么需要 HOLDING 状态？

A: 这是安全的。MoveJ 突然停止后，机械臂需要时间稳定。HOLDING 状态确保位置锁定，然后才能转移到新的运动模式。

### Q: 用户可以同时对两个 mapping（left_arm, right_arm）发送命令吗？

A: 可以！每个 mapping 有独立的 ControllerStateManager，互不影响。
