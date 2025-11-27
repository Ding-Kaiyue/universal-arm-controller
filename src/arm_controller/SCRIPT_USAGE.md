# IPC 管理脚本使用手册

## 📍 脚本位置

所有脚本位于：`scripts/` 目录

```bash
cd src/arm_controller
ls -la scripts/
```

## 三个脚本详解

### 1️⃣ `shm_inspect.sh` - IPC 资源监控

**功能**：查看 IPC 系统状态

#### 基础用法
```bash
$ ./scripts/shm_inspect.sh

📊 IPC 资源状态

✅ 共享内存 (SHM)
   名称:   arm_controller_shm_v1
   大小:   16M
   所有者: dyk:dyk
   修改时间: 2024-11-27 14:20:00

✅ 命名互斥量 (Mutex)
   名称: arm_controller_mutex

✅ 命名条件变量 (Condition)
   名称: arm_controller_cond
```

#### 详细模式
```bash
$ ./scripts/shm_inspect.sh -v

# 显示：
# - 共享内存详细信息
# - 所有 IPC 资源列表
# - 资源统计数据
```

#### 常见输出解读

| 输出 | 含义 | 处理 |
|------|------|------|
| ✅ 共享内存存在 | ROS2 系统正常运行 | 无需处理 |
| ❌ 共享内存不存在 | ROS2 系统未启动或已关闭 | 启动 ROS2 系统 |
| ⚠️  资源数异常多 | 可能有孤立进程 | 运行 `shm_clean.sh` |

---

### 2️⃣ `shm_clean.sh` - IPC 资源清理

**功能**：清理异常退出留下的 IPC 资源

#### 模拟清理（推荐先用）
```bash
$ ./scripts/shm_clean.sh --dry-run

📋 模拟运行模式（不会删除任何文件）
将删除:
  /dev/shm/psm_arm_controller_shm_v1
  /dev/shm/sem.arm_controller_mutex
  /dev/shm/sem.arm_controller_cond

运行 'shm_clean.sh' 执行真实清理
```

#### 实际清理
```bash
$ ./scripts/shm_clean.sh

⚠️  警告
此操作将删除以下所有资源:
  - 共享内存: arm_controller_shm_v1
  - 互斥量:   arm_controller_mutex
  - 条件变量: arm_controller_cond

请确保没有 arm_controller 进程在运行！

继续清理? (y/N): y

✅ 清理完成！
```

#### ⚠️ 使用前检查清单

```bash
# 1. 确认没有 ROS2 进程运行
ps aux | grep ros2
ps aux | grep arm_controller

# 2. 如果有进程，关闭它
killall ros2
# 或在启动终端按 Ctrl+C

# 3. 模拟清理确认无误
./scripts/shm_clean.sh --dry-run

# 4. 执行清理
./scripts/shm_clean.sh
```

---

### 3️⃣ `test_ipc_flow.sh` - IPC 流程诊断

**功能**：测试完整的 IPC 执行流程

#### 使用
```bash
$ ./scripts/test_ipc_flow.sh

========================================
IPC 流程诊断工具
========================================

[步骤 1] 检查消费端进程...
✓ 消费端进程运行中
  1234 ros2 launch

[步骤 2] 运行 example_single_arm...
✅ ArmControllerAPI initialized successfully
✅ SharedMemoryManager opened successfully
📥 MoveJ: Received IPC command (ID: cmd_...)
✅ Command executed successfully

[步骤 3] 检查 IPC 资源状态...
✅ 共享内存 (SHM) - 存在
✅ 命名互斥量 (Mutex) - 存在
✅ 命名条件变量 (Condition) - 存在

========================================
诊断完成
========================================
```

---

## 🔄 常见工作流程

### 场景 1：日常开发

```bash
# 1. 启动 ROS2 系统（终端 1）
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py

# 2. 检查 IPC 状态（终端 2）
cd src/arm_controller
./scripts/shm_inspect.sh

# 期望输出：✅ 所有资源都存在

# 3. 运行测试（终端 2）
./scripts/test_ipc_flow.sh

# 期望输出：完整的执行流程
```

### 场景 2：程序崩溃恢复

```bash
# 1. 检查问题
./scripts/shm_inspect.sh
# 可能显示：❌ 共享内存不存在或孤立

# 2. 模拟清理看看会删除什么
./scripts/shm_clean.sh --dry-run

# 3. 确认后清理
./scripts/shm_clean.sh

# 4. 重新启动
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

### 场景 3：排查通信问题

```bash
# 1. 详细检查
./scripts/shm_inspect.sh -v

# 2. 诊断完整流程
./scripts/test_ipc_flow.sh

# 3. 查看日志
tail -f /tmp/arm_controller.log
```

---

## 🎯 脚本命令速查表

```bash
# 监控
./scripts/shm_inspect.sh           # 查看状态
./scripts/shm_inspect.sh -v        # 详细查看

# 诊断
./scripts/test_ipc_flow.sh         # 测试流程

# 清理（谨慎使用）
./scripts/shm_clean.sh --dry-run   # 模拟清理
./scripts/shm_clean.sh             # 实际清理
```

---

## 💡 小技巧

### 持续监控 IPC 状态
```bash
watch ./scripts/shm_inspect.sh
# 每 2 秒自动刷新显示
```

### 检查特定资源
```bash
# 只看共享内存
ls -lh /dev/shm/psm_*

# 只看互斥量
ls -lh /dev/shm/sem.arm_controller*

# 检查大小
du -sh /dev/shm/psm_arm_controller_shm_v1
```

### 手动清理资源（高级用户）
```bash
# 不推荐！使用脚本更安全
rm /dev/shm/psm_arm_controller_shm_v1
rm /dev/shm/sem.arm_controller_mutex
rm /dev/shm/sem.arm_controller_cond
```

---

## ❌ 常见错误

### 错误 1：权限不足
```bash
$ ./scripts/shm_clean.sh
❌ rm: cannot remove '/dev/shm/...': Permission denied
```
**解决**：
```bash
sudo ./scripts/shm_clean.sh
# 或者运行整个用户组有权限的操作
```

### 错误 2：脚本找不到
```bash
$ ./scripts/shm_inspect.sh
bash: ./scripts/shm_inspect.sh: No such file or directory
```
**解决**：
```bash
cd src/arm_controller
./scripts/shm_inspect.sh
```

### 错误 3：脚本没有执行权限
```bash
$ ./scripts/shm_inspect.sh
bash: ./scripts/shm_inspect.sh: Permission denied
```
**解决**：
```bash
chmod +x scripts/*.sh
./scripts/shm_inspect.sh
```

---

## 📞 故障排查流程

```
问题: IPC 通信失败

1. 运行诊断
   ./scripts/test_ipc_flow.sh
   ↓
2. 检查资源
   ./scripts/shm_inspect.sh -v
   ↓
3. 资源异常?
   YES → 清理资源
         ./scripts/shm_clean.sh --dry-run
         ./scripts/shm_clean.sh
         重启 ROS2 系统
   NO → 检查日志
        查看消费端进程是否正常
        检查网络/驱动
```

---

## ✅ 清单

- [ ] 了解三个脚本的用途
- [ ] 会运行 `shm_inspect.sh` 检查状态
- [ ] 会运行 `test_ipc_flow.sh` 诊断问题
- [ ] 知道何时使用 `shm_clean.sh`
- [ ] 知道清理前要做什么
- [ ] 能够快速恢复异常状态
