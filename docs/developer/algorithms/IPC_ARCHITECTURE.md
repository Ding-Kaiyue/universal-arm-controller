# 进程间通信（IPC）机制设计说明

> 本文档描述 Universal Arm Controller 系统中，进程间通信（IPC）模块的架构设计、生命周期管理与跨进程协调机制。

## 1. 功能定位与系统角色

### 1.1 核心问题

Universal Arm Controller 的多控制模式和灵活的接口需求导致一个根本性的架构问题：

> **如何让多个外部进程（如用户应用、遥控程序、OpenClaw 插件）与实时控制循环安全地交互，而不中断 100Hz 实时保证？**

IPC 机制的设计目标：
- **实时性保证**：ROS 2 多线程执行器的控制循环不被外部进程调用阻塞
- **进程隔离**：外部进程的崩溃不直接影响实时控制器
- **异步通信**：命令生产者（外部进程）与命令消费者（实时控制器）异步操作
- **安全共享**：多个生产者同时推送命令，消费者安全消费，无竞态条件

### 1.2 在系统架构中的位置

IPC 机制位于 **系统接口层（System Interface Layer）**，处于以下角色：

```
┌─────────────────────────────────────────────────────┐
│  外部应用层（External Applications）                │
│  ├── OpenClaw Plugin                               │
│  ├── HTTP Server                                   │
│  ├── ROS 2 Nodes                                   │
│  └── Command Line Tools                            │
└──────────────────┬──────────────────────────────────┘
                   │ IPC
┌──────────────────▼──────────────────────────────────┐
│  系统接口层（System Interface Layer）               │
│  ├── CommandQueue (生产者)                          │
│  ├── SharedMemory Manager                          │
│  └── IPC Lifecycle                                 │
└──────────────────┬──────────────────────────────────┘
                   │ ROS 2 Topics
┌──────────────────▼──────────────────────────────────┐
│  控制策略层（Control Strategy Layer）               │
│  ├── MoveJ, MoveL, MoveC Controllers               │
│  ├── JointVelocity, CartesianVelocity Controllers  │
│  └── Real-time Executor (100Hz)                    │
└─────────────────────────────────────────────────────┘
```

---

## 2. 架构设计

### 2.1 跨进程生产者-消费者模式

IPC 采用经典的**生产者-消费者模式**，但针对实时系统进行了特殊设计：

```
外部进程 1          外部进程 2          实时控制进程
  (Producer)         (Producer)        (Consumer)
     │                   │                  │
     │ 1. push cmd       │                  │
     └──────────────────►├─ 命令队列 ◄──────┤ 3. pop cmd
                         │ (共享内存)       │
                    2. notify          4. consume
```

**关键特征**：
- **异步性**：生产者 push 后立即返回，无需等待消费者处理
- **单消费者**：只有一个实时控制进程消费命令，避免争用
- **多生产者**：多个外部进程可同时 push，通过互斥锁序列化
- **无阻塞优先**：消费者不因等待命令而阻塞，采用轮询 + 条件变量

### 2.2 Role-Based 权限模型

跨进程共享资源的核心困难：**多个进程有权创建和删除同一个共享资源，导致所有权混乱和竞态条件**。

解决方案：**Role-Based 权限模型**，明确区分两种角色：

#### Owner 角色（消费者）

- **身份**：实时控制进程（main.cpp）
- **权限**：
  - 有权**创建**共享内存（SHM）、互斥锁（named_mutex）、条件变量（named_condition）
  - 有权**删除**这些资源（在系统关闭时）
  - 有权调用 `cleanup()` 彻底删除系统资源
- **职责**：
  - 程序启动时：如果资源不存在，创建新的；如果存在，检验完整性后继续使用
  - 程序关闭时：负责清理所有共享资源，确保系统回收完全

#### Participant 角色（生产者）

- **身份**：外部进程（example_velocity_control, OpenClaw 插件等）
- **权限**：
  - 只能**打开**现有的共享资源（open_only 模式）
  - **禁止**创建新资源（避免所有权冲突）
  - **禁止**删除资源（防止破坏消费者仍在使用的资源）
- **职责**：
  - 启动前：确保 Owner 已启动（资源存在）
  - 运行中：只 push 命令，不涉及资源生命周期
  - 关闭时：资源自动清理，无需手动 cleanup

### 2.3 共享资源的完整生命周期

理解 SHM 资源的两个清理阶段至关重要：

#### 第一阶段：本地指针清理（close）

```
内存中的对象         操作系统的系统表
┌────────────┐      ┌──────────────┐
│ segment_   │      │ SHM ref_cnt  │
│ queue_     │─────►│ = 2          │
│ mutex_     │      │ status=active│
└────────────┘      └──────────────┘

调用 close() 后：

┌────────────┐      ┌──────────────┐
│ nullptr    │      │ SHM ref_cnt  │
│ nullptr    │      │ = 1          │
│ nullptr    │      │ status=active│
└────────────┘      └──────────────┘

本地指针清理，但系统资源仍存在
```

**仅 close() 不足以释放系统资源**。如果多个引用都只调用 close()，最后一个 close() 之后系统仍会标记资源为"dest"（待删除）。

#### 第二阶段：系统资源删除（cleanup）

```
系统表中的资源       操作系统
┌──────────────┐   ┌────────────────┐
│ SHM_NAME:    │   │ /dev/shm/...   │
│ status=dest  │   │ (file deleted) │
│ ref_cnt=1    │   │                │
└──────────────┘   └────────────────┘

调用 cleanup() 后：

┌──────────────┐   ┌────────────────┐
│ (removed)    │   │ (not found)    │
│              │   │                │
└──────────────┘   └────────────────┘

系统完全释放，可在下次启动时重新创建
```

**关键发现**：
- `close()` = 本地清理（清理指针引用）
- `cleanup()` = 系统清理（调用 `boost::interprocess::shared_memory_object::remove()` 等系统 API 删除）
- Owner 必须调用 `cleanup()`，才能实现**完全释放**
- 只调用 `close()` 会导致资源标记为"dest"且无法彻底回收

---

## 3. 关闭顺序的重要性

### 3.1 为什么顺序很关键

多线程实时系统中，IPC 资源被多个线程使用。关闭顺序错误导致的常见问题：

**错误顺序**（❌）：
```
1. executor.spin() ──► 消费线程运行中...
2. IPCContext::shutdown() ──► 删除 SHM
3. 节点析构... ──► 消费线程尝试访问已删除 SHM
                    ↓ SIGSEGV 段错误
```

**正确顺序**（✅）：
```
1. executor.spin() ──► 消费线程运行中...
2. 节点析构 ──► 消费线程逐个 join() 并停止
3. sleep(100ms) ──► 确保所有线程完全退出
4. IPCContext::shutdown() ──► 安全删除 SHM
   （无线程再访问资源）
```

### 3.2 关键的同步点

```
主线程                      消费线程 1-N
  │
  ├─ executor.spin()
  │  └─ add_node(controller) ──► 启动消费循环
  │
  ├─ executor.spin_some()         (pop command loop)
  │  │                            (while not shutdown)
  │  │                            {
  │  │                              lock mutex
  │  │                              pop from queue
  │  │                              execute command
  │  │                            }
  │  └─ (return on Ctrl-C)
  │
  ├─ node.reset() ───────────────► (析构触发 join)
  │  │                            (等待循环终止)
  │  │                            (释放 lock)
  │
  ├─ sleep(100ms) ───────────────► (确保全部退出)
  │
  ├─ shutdown() ──────────────────► cleanup()
  │  └─ 删除 SHM/mutex/condition   (现在安全)
  │
  └─ return
```

---

## 4. 多生产者的并发安全性

### 4.1 命令队列的线程安全设计

```
Producer-1        Producer-2        Consumer
   │                  │                │
   ├─ lock mutex      │                │
   │  ├─ push cmd ────┤                │
   │  └─ notify   ◄───┤                │
   └─ unlock          │                ├─ lock mutex
                      │                │  ├─ pop cmd
                      │                │  └─ unlock
                      ├─ lock mutex    │
                      │  ├─ push cmd   ├─ execute
                      │  └─ notify ───►│
                      └─ unlock        │
```

**同步机制**：
1. **Mutex（命名互斥锁）**：序列化对队列的访问，防止 push/pop 时的数据竞争
2. **Condition Variable**：
   - Producer push 后 notify_all()，唤醒等待的消费者
   - Consumer 可选择阻塞等待或轮询检查

### 4.2 避免死锁的设计

在关闭过程中，由于 Owner 会删除 mutex/condition，必须特别小心：

**风险**：
- Consumer 可能在尝试获取已被删除的 mutex
- Mutex 在关闭时可能被 cleanup 线程删除，导致未来的 lock() 失败

**解决方案**：
1. Consumer 采用 **try_to_lock**（非阻塞）而非阻塞 lock
2. Consumer 每次轮询迭代都重新检查 SHM 有效性
3. Owner shutdown() 前，先设置 shutdown flag，让消费线程主动退出
4. 等待 100ms 确保所有消费线程已释放所有锁

---

## 5. 初始化策略

### 5.1 第一次启动 vs 恢复启动

IPC 资源的初始化采用"尝试打开，失败则创建"策略：

```
Owner（Consumer 主进程）
  │
  ├─ 尝试 open() 现有的 SHM
  │  ├─ 成功 ──► 检验 header 完整性
  │  │           ├─ 有效 ──► 直接使用
  │  │           └─ 无效 ──► 删除重建
  │  │
  │  └─ 失败 ──► 是否第一次启动？
  │              └─ 是 ──► initialize(Role::Owner)
  │                        创建新 SHM + header + queue
  │
  └─ 准备完毕

Participant（Producer 外部进程）
  │
  ├─ 尝试 open() 现有的 SHM
  │  ├─ 成功 ──► 检验 header 有效性
  │  │           ├─ 有效 ──► 直接使用
  │  │           └─ 无效 ──► 报错，等待 Owner 修复
  │  │
  │  └─ 失败 ──► 报错："Owner 未启动"
  │
  └─ 准备完毕或报错
```

**header 的作用**：
- 版本号检验：确保生产者和消费者的 protocol 兼容
- 完整性标记：防止使用被破坏的 SHM
- 数据结构大小：运行时验证 queue, mutex, condition 的偏移和大小

### 5.2 为什么不自动创建

Participant（Producer）**禁止调用 initialize()**，即使资源不存在：

**原因**：
1. **多进程竞态**：多个 Producer 同时启动时，不知道谁应该创建
2. **所有权混乱**：如果 Producer 创建了资源，Owner 关闭时删除，Producer 仍在使用 ──► SIGSEGV
3. **清晰的依赖关系**：Producer 应该依赖 Owner，而不是独立存在

---

## 6. 故障恢复机制

### 6.1 不完整关闭的恢复

场景：前次关闭异常，资源残留

```
系统状态                Owner 行为
┌────────────────────┐
│ SHM exists (old)   │
│ ref_cnt = dest     │  ├─ open() 失败
│ (无法 attach)      │  │
│                    │  └─ initialize(Role::Owner)
│                    │     ├─ remove() 旧资源
│                    │     ├─ create_only() 新资源
│                    │     └─ 重新初始化
│
└────────────────────┘
```

**清理步骤**（Owner 初始化时自动执行）：
1. 尝试 remove(SHM_NAME) - 清理旧的 SHM
2. 尝试 remove(MUTEX_NAME) - 清理旧的互斥锁
3. 尝试 remove(COND_NAME) - 清理旧的条件变量
4. create_only() 创建新的、干净的资源

这确保了即使前次 cleanup() 不完整，Owner 启动时也能"自愈"。

### 6.2 Producer 应对资源不存在

```
场景：Owner 未启动

Producer 的行为：
  ├─ open() 失败
  ├─ 报错："Shared memory not found"
  ├─ 等待用户启动 Owner
  └─ （可选）定期重试
```

Producer **不应该**尝试创建资源。这是一个**设计约束**，用来强制依赖关系。

---

## 7. 性能考虑

### 7.1 轮询 vs 阻塞等待

```
消费者等待命令的两种策略：

轮询（Non-blocking）：
  while (!shutdown) {
    if (pop(cmd)) {
      execute(cmd)
    }
    sleep(1ms)  // 频繁检查
  }
  优点：快速响应 Ctrl-C，避免死锁
  缺点：CPU 占用率高

阻塞等待（Blocking）：
  while (!shutdown) {
    if (pop(cmd, timeout=1000)) {
      execute(cmd)
    }
  }
  优点：CPU 占用率低，无命令时休眠
  缺点：可能延迟响应信号
```

当前设计采用**轮询** with **短睡眠 (1ms)**：
- 响应时间：最多 1ms（相对于 Ctrl-C）
- CPU 占用：约 100 × 1ms/100ms = 1% overhead
- 死锁风险：极低（非阻塞）

### 7.2 命令队列大小

当前设计使用 **deque**（双端队列）with 固定深度，存储命令的 **副本**：

```
队列结构：
┌─────────────────────────────────┐
│ [cmd0] [cmd1] [cmd2] ... [cmdN] │  (最多 N 个命令)
└─────────────────────────────────┘
```

**权衡**：
- 深度太小（如 1）：高吞吐场景下，后续 push 可能因队列满而失败
- 深度太大（如 1000）：内存占用增加，且在高延迟时难以诊断问题

---

## 8. 监控与诊断

### 8.1 资源泄漏的诊断

```bash
# 查看系统资源
ipcs -m  # 共享内存
ipcs -S  # 信号量

# 标志：
# nattch=0  && status=(空) ──► 无进程持有（正常释放）
# nattch=1+ && status=dest  ──► 有进程持有但标记删除（泄漏）
```

### 8.2 死锁的诊断

```bash
# 查看进程状态
ps aux | grep arm_controller

# 状态标记：
# S ──► 可中断睡眠（正常）
# D ──► 不可中断睡眠（可能死锁）
# Z ──► 僵尸进程（清理失败）
```

---

## 9. 相关文件与代码组织

| 组件 | 文件 | 职责 |
|------|------|------|
| SharedMemoryManager | `ipc/shm_manager.{hpp,cpp}` | 直接操作 Boost.Interprocess，管理 SHM/mutex/cond |
| IPCContext | `ipc/ipc_context.{hpp,cpp}` | 高级接口，负责初始化策略和生命周期 |
| CommandQueueIPC | `ipc/command_queue_ipc.hpp` | 队列操作接口（push/pop），消费者逻辑 |
| CommandProducer | `ipc/command_producer.hpp` | 生产者端的命令构建和推送接口 |
| TrajectoryCommand | `ipc/ipc_types.hpp` | 命令的数据结构定义 |

---

## 10. 最后的设计原则

1. **明确的所有权**：Owner/Participant 角色必须严格区分，无例外
2. **两阶段清理**：close() 清理指针，cleanup() 删除系统资源，缺一不可
3. **关闭顺序**：线程 → 资源，绝不反向
4. **防守式设计**：消费者假设 SHM 可能在任何时刻被删除，每轮都重新检查
5. **单一消费者**：避免消费端的竞争，降低死锁风险
6. **多生产者序列化**：通过 mutex 将并行的 push 操作序列化

遵循这些原则，跨进程 IPC 可以做到既安全又高效。
