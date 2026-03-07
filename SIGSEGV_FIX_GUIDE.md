# SIGSEGV 段错误 - 快速诊断与修复参考

> **重要**：详细的 IPC 架构设计说明请参考 [IPC_ARCHITECTURE.md](docs/developer/algorithms/IPC_ARCHITECTURE.md)
>
> 本文档提供快速诊断流程和修复清单，用于现场排查问题。

**最后更新**: 2026-03-04
**修复版本**: feature/ipc

---

## 🚨 问题症状

| 症状 | 现象 | 原因 |
|-----|------|------|
| **崩溃** | 按 Ctrl-C 后程序崩溃，exit code -11 | 访问已删除的 SHM |
| **资源泄漏** | `ipcs -m` 显示 `dest` 状态资源 | close() 未彻底释放系统资源 |
| **重启失败** | 第二次启动报错 "No such file or directory" | 旧资源仍被标记为 dest，无法重建 |
| **RViz 卡顿** | RViz 进程进入 D 状态（不可中断睡眠） | 关闭顺序错误，消费线程访问已删除资源 |

---

## 🔍 根本原因（简述）

跨进程共享内存（SHM）的生命周期管理有四个主要陷阱：

1. **Owner/Participant 角色混乱**
   - 多个进程都有创建/删除权限 → 竞态条件 → SIGSEGV

2. **清理不彻底**
   - 只调用 `close()`（清理本地指针）而不调用 `cleanup()`（删除系统资源）
   - 导致资源标记为 `dest`，无法回收

3. **关闭顺序错误**
   - 先删除 SHM，后销毁节点 → 消费线程访问已删除资源 → SIGSEGV

4. **多线程初始化冲突**
   - 多个地方调用 `rclcpp::init()` → ROS 内部死锁

---

## ✅ 修复清单

### 修复 1：Role-Based 权限模型

**状态**：✅ 已实现
**文件**：`shm_manager.hpp`, `shm_manager.cpp`, `ipc_context.{hpp,cpp}`
**目标**：强制只有 Owner 能创建/删除 SHM

```
□ 添加 enum class Role { Owner, Participant }
□ initialize() 方法检查角色，Participant 禁止调用
□ Consumer 使用 initializeAsConsumer()
□ Producer 使用 initialize()（仅 open，无 create）
```

### 修复 2：Owner 调用 cleanup() 删除系统资源（⭐ 关键）

**状态**：✅ 已实现
**文件**：`ipc_context.cpp` 的 `shutdown()` 方法
**目标**：Owner 关闭时必须彻底删除系统资源

```
□ Owner shutdown() 中调用 shm_manager_->cleanup()
  （cleanup() = 调用 boost::interprocess::remove() 删除系统资源）
□ 禁止只调用 close()（这仅清理本地指针）
□ Participant shutdown() 仅调用 reset()，不清理系统资源
```

### 修复 3：CommandQueueIPC 资源清理

**状态**：✅ 已实现
**文件**：`command_queue_ipc.hpp`
**目标**：清理本地 shm_manager_ 引用，允许 IPCContext 完全删除系统资源

```
□ shutdown() 中添加 shm_manager_.reset()
□ popWithFilter() 每轮循环重新获取 SHM 指针（防止悬空）
```

### 修复 4：正确的关闭顺序

**状态**：✅ 已实现
**文件**：`main.cpp`
**目标**：先停止消费线程，再删除 SHM

```
顺序很关键（从 executor.spin() 返回后）：
  1. 销毁节点 (controller_manager.reset(), trajectory_controller.reset())
  2. 等待线程完全退出 (sleep 100ms)
  3. 清理 IPC 资源 (IPCLifecycle::shutdown())

□ 实现正确的关闭顺序
□ 在 shutdown 前等待 100ms
```

### 修复 5：Producer 不应清理

**状态**：✅ 已实现
**文件**：`example_velocity_control.cpp`
**目标**：Producer 只 attach，不清理系统资源

```
□ 删除 IPCLifecycle::shutdown() 调用
□ Producer 的资源在程序退出时自动清理
```

### 修复 6：删除 ROS 初始化冲突

**状态**：✅ 已实现
**文件**：`ipc_context.cpp`
**目标**：让 main.cpp 单独负责 ROS 初始化

```
□ initialize() 和 initializeAsConsumer() 中删除 rclcpp::init() 调用
□ main.cpp 负责唯一的 rclcpp::init()
```

### 修复 7：静默 open() 失败日志

**状态**：✅ 已实现
**文件**：`shm_manager.cpp`
**目标**：open() 失败不打印错误（第一次启动是正常的）

```
□ open() 失败时不输出 cerr，仅返回 false
□ 让调用者根据返回值决定是否创建
```

---

## 🚀 快速诊断流程

### 第 1 步：确认问题类型

```bash
# 查看 exit code
echo $?  # -11 = SIGSEGV, 其他 = 其他原因

# 查看日志
ros2 launch ... 2>&1 | tail -50
```

### 第 2 步：检查 SHM 资源状态

```bash
ipcs -m

# 诊断：
# nattch=0 && status=(空)  ──► 正常释放 ✅
# nattch=1+ && status=dest ──► 资源泄漏 ❌（修复 2/3 的问题）
# key=0xXXX 和预期不符     ──► 多次创建（修复 1 的问题）
```

### 第 3 步：验证修复清单

逐项检查上述修复 1-7 是否都已实现：

```bash
# 检查 Role enum
grep -n "enum class Role" src/arm_controller/include/arm_controller/ipc/shm_manager.hpp

# 检查 cleanup() 调用
grep -n "cleanup()" src/arm_controller/src/ipc/ipc_context.cpp

# 检查关闭顺序
grep -A5 "Executor stopped" src/arm_controller/src/main.cpp

# 检查 Producer 不调用 shutdown()
grep -n "shutdown()" src/arm_controller/example/example_velocity_control.cpp
# 应该看不到（或只在注释中出现）
```

### 第 4 步：编译并测试

```bash
# 清理旧编译
rm -rf build install

# 重新编译
colcon build --packages-select arm_controller

# 测试：第一次启动和关闭
ros2 launch arm_controller dual_arm_bringup.launch.py &
sleep 2
pkill -SIGINT arm_controller  # 或 Ctrl-C

# 预期：无 SIGSEGV，正常退出
echo $?  # 应为 0 或 143（SIGTERM）

# 测试：第二次启动（验证资源完全释放）
ros2 launch arm_controller dual_arm_bringup.launch.py
# 预期：无 "No such file or directory" 错误

# 验证资源清理
ipcs -m
# 预期：没有我们的 SHM 资源（或有但不是 dest 状态）
```

---

## 📚 相关文件

| 组件 | 文件 | 修复项 |
|------|------|--------|
| Role 定义 | shm_manager.hpp | 修复 1 |
| 初始化逻辑 | shm_manager.cpp, ipc_context.cpp | 修复 1, 2, 6, 7 |
| 消费者清理 | command_queue_ipc.hpp | 修复 3 |
| 主程序关闭 | main.cpp | 修复 4 |
| Producer | example_velocity_control.cpp | 修复 5 |

详细的 IPC 架构设计和工作原理，请参考：[IPC_ARCHITECTURE.md](docs/developer/algorithms/IPC_ARCHITECTURE.md)

---

## 💡 核心要点

1. **Owner/Participant 是必需的**
   - Owner（Consumer 主进程）有权创建和删除 SHM
   - Participant（Producer 外部进程）只能 open 和 attach，禁止删除

2. **cleanup() vs close()**
   - `close()` 仅清理本地指针，不删除系统资源
   - `cleanup()` 调用系统 API 彻底删除 SHM/mutex/condition
   - **Owner 必须调用 cleanup()**，否则资源标记为 `dest` 无法回收

3. **关闭顺序：线程 → 资源**
   - 先销毁节点，等待消费线程 join
   - 再清理 IPC 资源
   - 反向会导致 SIGSEGV

4. **诊断资源泄漏**
   ```bash
   ipcs -m | grep dest  # 如果有输出，说明修复 2 或 3 不完整
   ```

---

## 🔗 更多信息

- **IPC 架构详解**：[IPC_ARCHITECTURE.md](docs/developer/algorithms/IPC_ARCHITECTURE.md)
  - Owner/Participant 权限模型
  - 两阶段清理的必要性
  - 多生产者并发安全
  - 故障恢复机制

- **Boost.Interprocess 文档**：https://www.boost.org/doc/libs/1_79_0/doc/html/interprocess.html

---

**文档版本**: 2.0 (快速参考指南)
**维护人员**: Claude Code
