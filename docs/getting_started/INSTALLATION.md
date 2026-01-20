# 安装指南

详细的 Universal Arm Controller 环境配置步骤。

## 目录

- [系统要求](#系统要求)
- [前置准备](#前置准备)
- [标准安装流程](#标准安装流程)
- [安装验证](#安装验证)
---

## 系统要求

### 操作系统

- **Ubuntu 22.04 LTS** 或更高版本（推荐 Ubuntu 22.04）
- 需要 Linux 内核 4.4+ 以支持 CAN

### 软件依赖

- **ROS2 Humble** 或更高版本
- **GCC 10+** 或 **Clang 12+**（支持 C++17）
- **CMake 3.10+**
- **Python 3.10+**

### 工具软件

```bash
# 必需工具
sudo apt install python3-vcstool python3-colcon-common-extensions
sudo apt install build-essential cmake git

# 依赖库
sudo apt install libyaml-cpp-dev libeigen3-dev libfmt-dev
```

### 硬件要求

- **CAN 接口**: 兼容 SocketCAN 的 CAN-FD 接口
- **内存**: 编译时需要 16GB+ RAM（或 8GB RAM + 8GB+ Swap）；运行时最少 4GB RAM
- **CPU**: 四核或以上

---

## 前置准备

### 1. 安装 ROS2 Humble

如果还未安装 ROS2，按照 [ROS2 官方文档](https://docs.ros.org/en/humble/Installation.html) 安装。

```bash
# 验证 ROS2 安装
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2. 安装系统依赖

```bash
# 更新包管理器
sudo apt-get update

# 安装必需的系统工具和库
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-dev \
  python3-numpy \
  python3-scipy \
  python3-pip \
  libeigen3-dev \
  libboost-all-dev \
  liborocos-kdl-dev \
  libnlopt0 \
  libnlopt-dev \
  libyaml-cpp-dev \
  liburdfdom-headers-dev \
  liburdfdom-dev

# 安装 ROS2 MoveIt 完整包
sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-tf2-kdl \
  ros-humble-kdl-parser \
  ros-humble-control-msgs
```

### 3. 安装 Python 依赖

```bash
python3 -m pip install --no-cache-dir qdldl
```

### 4. 安装 eigenpy（C++ 版本）

```bash
cd /tmp
git clone --depth 1 https://github.com/stack-of-tasks/eigenpy.git
cd eigenpy
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 5. 安装 OSQP 和 OsqpEigen

```bash
# 安装 OSQP
cd /tmp
git clone --depth 1 https://github.com/osqp/osqp.git
cd osqp
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
sudo ldconfig
```
### 6. 安装 OsqpEigen

```bash
cd /tmp
git clone --depth 1 https://github.com/gbionics/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 7. 安装 Pinocchio（C++ 版本）

```bash
cd /tmp
git clone --depth 1 https://github.com/stack-of-tasks/pinocchio.git
cd pinocchio
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_COLLISION_SUPPORT=OFF \
  -DBUILD_WITH_URDF_SUPPORT=ON \
  -DBUILD_PYTHON_INTERFACE=ON \
  -DBUILD_WITH_PARSERS=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTING=OFF ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 8. 安装 NLopt（从源码）

```bash
cd /tmp
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DNLOPT_CXX=ON ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 9. 安装 TracIK

```bash
mkdir -p ~/trac_ik_ws/src
cd ~/trac_ik_ws/src
git clone https://github.com/aprotyas/trac_ik.git

cd ~/trac_ik_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-Wno-error=maybe-uninitialized'
sudo ldconfig

# 添加到 bashrc
echo "source ~/trac_ik_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 标准安装流程

### 步骤 1：创建工作空间

```bash
mkdir -p ~/robotic_arm_ws/src
cd ~/robotic_arm_ws/src
```

### 步骤 2：克隆仓库

```bash
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller
```

### 步骤 3：导入依赖

```bash
# 进入源码目录
cd src

# 导入依赖（包括 hardware_driver、trajectory_interpolator、trajectory_planning）
vcs import < ../deps.repos --recursive

# 验证导入
ls
# 应该看到：arm_controller, controller_interfaces, robotic_arm_bringup,
# hardware_driver, trajectory_interpolator, trajectory_planning, csaps
```

### 步骤 4：安装 ROS 依赖

```bash
cd ~/robotic_arm_ws

# 自动安装所有依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 步骤 5：编译

```bash
cd ~/robotic_arm_ws

# 使用官方 build.sh 脚本（推荐）
# 该脚本会自动检查内存、优化编译参数
./src/universal-arm-controller/build.sh

# 或者手动使用 colcon build
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**build.sh 脚本特性：**
- 自动检查系统内存（RAM + Swap）
- 内存不足时给出明确提示和解决方案
- 使用内存优化的链接器标志
- 顺序编译避免 OOM
- 支持 `--dev` 模式用于开发调试

### 步骤 6：环境配置

```bash
# 设置环境变量
source install/setup.bash

# 添加到 bashrc（可选但推荐）
echo "source ~/robotic_arm_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 安装验证

### 检查编译结果

```bash
# 验证文件夹存在
ls ~/robotic_arm_ws/install/

# 应该看到：
# arm_controller  controller_interfaces  hardware_driver
# robotic_arm_bringup  trajectory_interpolator  trajectory_planning csaps
```

### 验证 ROS 包

```bash
# 检查 arm_controller 是否可被 ROS 找到
ros2 pkg list | grep arm_controller

# 应该输出：
# arm_controller
```

### 验证启动文件

```bash
# 检查启动文件是否存在
ls ~/robotic_arm_ws/install/robotic_arm_bringup/share/robotic_arm_bringup/

# 应该看到 robotic_arm_real.launch.py
```

---

**遇到问题？** 查看 **[故障排除](TROUBLESHOOTING.md)** 或提交 **[Issue](https://github.com/Ding-Kaiyue/universal-arm-controller/issues/new?template=installation_issue.md)**
