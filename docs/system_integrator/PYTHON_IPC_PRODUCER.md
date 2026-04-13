# Python 直连 IPC Producer

本文档说明如何让 Python 进程直接作为 IPC Producer（Participant）发送控制命令。

## 1. 构建绑定模块

`arm_controller` 已支持可选构建 pybind11 模块 `arm_controller_ipc`。

```bash
colcon build --packages-select arm_controller \
  --cmake-args -DBUILD_PYTHON_IPC_BINDINGS=ON
```

若系统未安装 `pybind11`，构建会跳过 Python 模块并给出 warning。

构建完成后执行：

```bash
source install/setup.bash
```

## 2. 运行前提

先启动控制进程（IPC Owner / Consumer），再启动 Python Producer。

```bash
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

## 3. Python 最小示例

示例脚本：

`src/arm_controller/example/python/example_ipc_producer.py`

核心流程：

1. `initialize_producer()` attach 到现有 IPC 共享资源  
2. 创建 `MoveJ/MoveL/...` 接口实例并 `execute(...)`  
3. 可选轮询 `get_execution_state(...)`  
4. 结束时调用 `shutdown()` 清理本进程上下文

## 4. 可用接口

- `MoveJ`
- `MoveL`
- `MoveC`
- `JointVelocity`
- `CartesianVelocity`
- `MinkServo`
- `TrajectoryRecord`
- `TrajectoryReplay`
- `BasicOps`

以及全局函数：

- `initialize_producer()`
- `is_initialized()`
- `shutdown()`
