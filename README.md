# Universal Arm Controller

[![ROS Version](https://img.shields.io/badge/ROS-ROS2%20Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/Ding-Kaiyue/universal-arm-controller/actions/workflows/colcon-build.yml/badge.svg?branch=master)](https://github.com/Ding-Kaiyue/universal-arm-controller/actions/workflows/colcon-build.yml)

完整的机械臂控制系统解决方案。基于 ROS2 的模块化架构，集成轨迹规划、轨迹插值、硬件驱动等核心功能。

## 核心特性

- **13+ 控制模式**: MoveJ、MoveL、MoveC、JointVelocity、CartesianVelocity 等
- **全 6D 方向控制**: 完整的末端执行器位姿控制
- **双臂原生支持**: 单臂/双臂协同控制
- **微秒级延迟**: <200μs 实时控制响应
- **高速通信**: CAN-FD 5000 kbit/s
- **工业级安全**: 多层安全检查、关节限位保护、紧急停止

## 快速开始

### Docker 部署（推荐）

**从 Docker Hub 拉取：**

```bash
# 配置 CAN 接口
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on loopback off
# 双臂请同时配置 can1
# sudo ip link set can1 txqueuelen 1000
# sudo ip link set can1 up type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on loopback off

# 可选：隔离 ROS 网络，避免同网段其他机器干扰
export ROS_DOMAIN_ID=78

xhost +local:docker
docker pull dingkaiyue/robotic-arm-controller:latest
docker run -dit --name robotic_arm \
  --network=host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  dingkaiyue/robotic-arm-controller:latest

docker exec -it robotic_arm bash
source /opt/robotic_arm_ws/install/setup.bash

# 启动系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

**国内用户：** 下载 [robotic-arm-controller-latest.tar.gz](链接: https://pan.baidu.com/s/165rKOZsq94QM9LzqbSPaEg?pwd=cjhk 提取码: cjhk)，然后加载镜像：

```bash
docker load < robotic-arm-controller-latest.tar.gz
docker run -dit --name robotic_arm \
  --network=host \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  dingkaiyue/robotic-arm-controller:latest
```
其余与上面从 Docker Hub 拉取相同。
### 本地编译

```bash
# 配置 CAN 接口
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on loopback off restart-ms 100
(If you are using dual arm, please don't forget to set can1)

mkdir -p ~/robotic_arm_ws/src && cd ~/robotic_arm_ws/src
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller/src
sudo apt install python3-vcstool
vcs import < ../deps.repos --recursive

cd ~/robotic_arm_ws
rosdep install --from-paths src --ignore-src -r -y
./src/universal-arm-controller/build.sh
source install/setup.bash

# 启动系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

---

## 完整文档

👉 **[进入文档中心](docs/README.md)** - 详细的安装指南、使用教程、架构设计、故障排除

## 许可证

MIT License - 详见 [LICENSE](LICENSE) 文件

## 联系方式

- **GitHub Issues**: [提交问题](https://github.com/Ding-Kaiyue/universal-arm-controller/issues)
  - 使用预定义的 Issue 模板报告 Bug、功能请求或安全问题
- **Email**: <kaiyue.ding@raysense.com>

## 贡献

欢迎贡献！详见 [CONTRIBUTING.md](.github/CONTRIBUTING.md)

---

⭐ **如果这个项目对你有帮助，请给我们一个星标！**
