# 快速开始

5分钟快速上手 Universal Arm Controller！

## 前提条件

- ✅ Ubuntu 22.04 LTS 或更高版本
- ✅ 根据 [安装指南](INSTALLATION.md) 配置好环境
- ✅ 网络连接正常

## 第一步：安装

```bash
# 1. 创建工作空间
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src

# 2. 克隆仓库与依赖
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller/src
sudo apt install python3-vcstool
vcs import < ../deps.repos --recursive

# 3. 编译
cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 第二步：配置 CAN 接口

```bash
# 配置 CAN0（根据你的硬件修改）
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 \
  dbitrate 5000000 dsample-point 0.75 fd on loopback off restart-ms 100

# 验证配置
ip link show can0
```

## 第三步：启动系统

```bash
# 启动完整的机械臂控制系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

## 第四步：测试基本功能

在另一个终端中：

```bash
# 切换到 MoveJ 模式
ros2 service call /controller_api/controller_mode \
  controller_interfaces/srv/WorkMode "{mode: 'MoveJ', mapping: 'single_arm'}"

# 发送关节目标位置
ros2 topic pub /controller_api/movej_action/single_arm sensor_msgs/msg/JointState \
  "{position: [0.2618, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## 成功标志

- ✅ 系统启动无错误
- ✅ 机械臂响应控制命令
- ✅ ROS Topics 正常发布

## 下一步

- 📖 学习 [系统概览](COMPONENTS.md)
- ⚙️ 查看 [Arm Controller 配置指南](../src/arm_controller/docs/CONFIGURATION.md)
- 🏗️ 理解 [系统架构](ARCHITECTURE.md)

## 常见问题

**Q: 编译失败？**
A: 确保已安装所有依赖。运行 `rosdep install --from-paths src --ignore-src -r -y`

**Q: CAN 接口配置失败？**
A: 检查系统权限。可能需要 `sudo` 或将用户添加到 dialout 组。

**Q: 机械臂不动？**
A: 检查 CAN 接口是否启动，电机是否上电。查看 [故障排除](TROUBLESHOOTING.md)。

---

更多详细信息请查看 [完整文档中心](README.md)。
