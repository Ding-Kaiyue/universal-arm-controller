# Docker 快速开始指南

本项目提供完整的 Docker 支持，用户无需本地编译即可直接使用。**特别推荐给内存受限系统**（<16GB RAM）。

## 🎯 为什么使用 Docker？

| 场景 | 本地编译 | Docker 构建 |
|------|---------|------------|
| 内存不足（<16GB） | ❌ 系统卡死 | ✅ 安全稳定 |
| 首次建立环境 | ⚠️ 困难 | ✅ 自动完成 |
| 开发人员冲突 | 可能 | 完全隔离 |
| 硬件调试 | 需要映射设备 | 支持 USB/CAN |
| 性能 | 原生运行 | 容器运行（<2% 开销） |

## ⚡ 快速开始（3 分钟）

### 前置要求

- **Docker 19.03+** 和 **Docker Compose 1.29+**
- **硬盘空间**: 20GB（构建时），最终镜像 2-3GB
- **网络**: 稳定的互联网（下载 ROS 基础镜像和依赖）

### 安装 Docker

```bash
# Ubuntu 22.04 LTS（推荐）
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# 将当前用户添加到 docker 组（避免 sudo）
sudo usermod -aG docker $USER
newgrp docker

# 验证
docker --version
docker-compose --version
```

其他系统：参考 [Docker 官方文档](https://docs.docker.com/engine/install/)

### 一键启动

```bash
# 1. 克隆项目
git clone https://github.com/Ding-Kaiyue/universal-arm-controller.git
cd universal-arm-controller

# 2. 启动容器（首次会自动构建镜像，耗时 30-60 分钟）
docker-compose up -d

# 3. 进入容器
docker-compose exec robotic-arm bash

# 4. 在容器内启动系统
source /opt/robotic_arm_ws/install/setup.bash
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

**就这么简单！** 源代码已在 `/opt/robotic_arm_ws/install` 完全编译完成。

## 🔧 详细配置说明

### ✅ Docker 的优势

| 优势 | 描述 |
|------|------|
| **无需本地编译** | 预编译的二进制，开箱即用 |
| **内存安全** | Docker 容器内存可独立限制，不会导致主机卡死 |
| **源码隐私** | 镜像仅包含编译后的库，不含源代码 |
| **环境隔离** | 避免本地依赖冲突和版本不兼容 |
| **跨平台一致性** | 所有用户在相同编译环境中运行 |
| **快速迭代** | 后续开发在容器内进行，不需要重新构建基础镜像 |
| **灵活部署** | 支持 USB/CAN 设备映射，可用于实机控制 |

### 镜像大小

- 构建阶段（中间层，不保留）: ~8-10GB
- 最终运行时镜像: **2-3GB**（通过多阶段构建优化）

## 🚨 内存受限系统的特殊处理

### 💡 如果你的系统 < 16GB

本项目在本地编译时会导致 Linux 内存压力过大（峰值链接器内存需求 ~10-12GB）。Docker 是最好的解决方案：

| 配置 | 本地编译 | Docker 构建 |
|------|---------|------------|
| 8GB RAM | ❌ 频繁卡死 | ✅ 安全（Docker 可限制容器内存）|
| 8GB RAM + 16GB Swap | ⚠️ 缓慢且卡死 | ✅ 稳定 |
| 16GB RAM | ✅ 勉强可以 | ✅ 推荐 |

### 解决方案优先级

#### 1️⃣ 推荐：使用 Docker（本指南）
**优点**:
- 构建时容器内存隔离，不会影响主机
- 构建过程稳定，无卡死风险
- 后续开发在容器内进行

**缺点**: 首次构建仍需 30-60 分钟

#### 2️⃣ 备选：本地添加 Swap（快速修补）
如果必须本地编译，添加 16GB Swap：
```bash
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
free -h  # 验证
```

然后运行本地构建：
```bash
cd ~/robotic_arm_ws
./src/universal-arm-controller/build.sh
```

#### 3️⃣ 部分构建（快速验证）
如果只想快速验证，跳过大型依赖：
```bash
# 仅编译核心控制器（跳过 trajectory_planning_v3）
cd ~/robotic_arm_ws
colcon build --packages-select arm_controller controller_interfaces robotic_arm_bringup
```

### Docker 内存管理

即使在 4GB RAM 的系统上运行，Docker 也能安全构建：

```bash
# 在 docker-compose.yml 中限制容器内存（可选）
services:
  robotic-arm:
    deploy:
      resources:
        limits:
          memory: 4G      # 严格限制容器内存
        reservations:
          memory: 2G      # 预留最少内存
```

这样容器即使占用过多内存也不会导致主机卡死（会进入容器内 OOM）。

### docker-compose.yml 配置详解

#### 基础配置（推荐）

```yaml
version: '3.8'

services:
  robotic-arm:
    build:
      context: .
      dockerfile: Dockerfile
    image: dyk/robotic-arm-controller:latest
    container_name: robotic_arm_controller

    # 网络：ROS 通信需要主机网络
    network_mode: host

    # IPC：进程间通信（ROS 需要）
    ipc: host

    # USB 和 CAN 设备映射（用于硬件控制）
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0  # 串口设备
      - /dev/ttyUSB1:/dev/ttyUSB1
      - /dev/can0:/dev/can0         # CAN 接口
      - /dev/can1:/dev/can1

    # 卷挂载
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw  # X11 GUI 支持
      - ./data:/root/data                   # 数据持久化

    # 环境变量
    environment:
      - DISPLAY=${DISPLAY}          # X11 显示
      - ROS_DOMAIN_ID=0             # ROS2 通信域

    # 容器运行策略
    stdin_open: true
    tty: true
    restart: unless-stopped
```

#### 内存受限系统配置

如果系统内存 < 16GB，在 `robotic-arm` 服务下添加内存限制：

```yaml
    deploy:
      resources:
        limits:
          memory: 4G                 # 硬限制（4GB）
        reservations:
          memory: 2G                 # 软保留（至少 2GB）
```

#### 启用 GPU（可选）

如果有 NVIDIA GPU：

```yaml
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
```

#### 开发模式（挂载源代码）

如果需要在容器内频繁修改代码：

```yaml
    volumes:
      - ./src:/root/robotic_arm_ws/src        # 挂载源代码
      - ./install:/root/robotic_arm_ws/install # 挂载编译结果
      - ./data:/root/data
```

然后在容器内编译：
```bash
docker-compose exec robotic-arm bash
cd /root/robotic_arm_ws
./src/universal-arm-controller/build.sh
```

## 💻 其他启动方式

### 方式 2: 手动 Docker 命令（无需 Docker Compose）

如果不使用 Docker Compose，可以手动运行 Docker 命令：

```bash
# 1. 构建镜像
docker build -t dyk/robotic-arm-controller:latest .

# 2. 运行容器
docker run -it \
  --name robotic_arm_controller \
  --network host \
  --ipc host \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  --device /dev/can0:/dev/can0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=${DISPLAY} \
  -e ROS_DOMAIN_ID=0 \
  dyk/robotic-arm-controller:latest \
  bash

# 3. 在容器内启动系统
source /opt/robotic_arm_ws/install/setup.bash
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

**注意**: 后续要进入同一容器，使用：
```bash
docker exec -it robotic_arm_controller bash
```

### 方式 3: 预构建镜像（即将推出）

一旦镜像发布到 Docker Hub，可以直接拉取而无需本地构建：

```bash
# 拉取预构建镜像（快速，~30 秒）
docker pull dyk/robotic-arm-controller:latest

# 运行容器
docker run -it \
  --name robotic_arm_controller \
  --network host \
  --ipc host \
  dyk/robotic-arm-controller:latest \
  bash
```

## 📊 常见操作

### 查看状态

```bash
# 查看正在运行的容器
docker-compose ps

# 查看容器日志（实时）
docker-compose logs -f robotic-arm

# 查看资源使用情况
docker stats robotic_arm_controller
```

### 进入/退出容器

```bash
# 进入容器
docker-compose exec robotic-arm bash

# 或使用 docker 命令
docker exec -it robotic_arm_controller bash

# 退出容器（在容器内）
exit
```

### 容器生命周期管理

```bash
# 启动容器（首次构建）
docker-compose up -d

# 停止容器
docker-compose stop

# 重启容器
docker-compose restart

# 停止并删除容器
docker-compose down

# 完全清理（删除容器、镜像、数据）
docker-compose down -v
```

### 清理 Docker 资源

```bash
# 删除未使用的镜像
docker image prune -a

# 删除未使用的容器
docker container prune

# 完整清理（谨慎！）
docker system prune -a
```

## ❓ 常见问题

### Q1: 如何在容器内访问硬件（CAN/USB）？

确保 docker-compose.yml 中的 `devices` 部分正确配置：

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
  - /dev/can0:/dev/can0
```

验证容器内能看到设备：
```bash
ls -la /dev/ttyUSB0
ls -la /dev/can0
```

如果看不到，检查：
1. 主机上设备是否存在：`ls -la /dev/ttyUSB0`
2. 权限：`sudo usermod -aG dialout $USER`
3. 重启容器：`docker-compose restart`

### Q2: 无法访问主机上的设备

**症状**: 容器内看不到 `/dev/ttyUSB0` 或 `/dev/can0`

**解决方案**:

```bash
# 1. 检查主机设备
ls -la /dev/ttyUSB*
ip link show type can

# 2. 检查权限
sudo usermod -aG dialout $USER
sudo usermod -aG plugdev $USER
newgrp dialout

# 3. 重启 Docker daemon
sudo systemctl restart docker

# 4. 重启容器
docker-compose restart
```

### Q3: 如何修改代码并重新编译？

**快速方式**（用于频繁开发）:

在 `docker-compose.yml` 中挂载源代码：

```yaml
volumes:
  - ./src:/root/robotic_arm_ws/src
  - ./install:/root/robotic_arm_ws/install
  - ./data:/root/data
```

然后在容器内编译：

```bash
docker-compose exec robotic-arm bash
cd /root/robotic_arm_ws
./src/universal-arm-controller/build.sh
```

**重新构建镜像方式**（用于完全更新）:

```bash
docker-compose build --no-cache
docker-compose up -d
```

### Q4: ROS 通信无法工作

**症状**: 容器内的 ROS2 nodes 看不到主机上的 topics/services

**解决方案**:

确保 docker-compose.yml 中配置了：

```yaml
network_mode: host     # 必须有
ipc: host             # 必须有
environment:
  - ROS_DOMAIN_ID=0   # 必须与主机一致
```

验证：

```bash
# 在主机上
ros2 topic list

# 在容器内
docker-compose exec robotic-arm bash
source /opt/robotic_arm_ws/install/setup.bash
ros2 topic list  # 应该看到相同的 topics
```

### Q5: 构建镜像时内存不足

**症状**: `docker build` 过程中卡死或 killed

**解决方案**:

1. **增加可用内存**:
   ```bash
   docker system prune -a   # 清理所有未使用的镜像
   ```

2. **等待主机释放内存**:
   ```bash
   free -h
   ps aux | sort -rn -k 4 | head -5  # 查看占内存最多的进程
   ```

3. **重试构建** （通常会使用缓存，速度快）:
   ```bash
   docker-compose build
   ```

### Q6: 如何将镜像推送到 Docker Hub？

```bash
# 登录 Docker Hub
docker login

# 标记镜像
docker tag dyk/robotic-arm-controller:latest your-username/robotic-arm-controller:latest

# 推送
docker push your-username/robotic-arm-controller:latest

# 其他用户可以直接拉取
docker pull your-username/robotic-arm-controller:latest
```

### Q7: 硬盘空间占用过多，如何清理？

```bash
# 查看 Docker 占用空间
docker system df

# 清理
docker-compose down          # 停止并删除容器
docker image prune -a        # 删除所有未使用的镜像
docker volume prune          # 删除所有未使用的卷
docker system prune -a --volumes  # 完整清理
```

## ⚡ 性能优化与最佳实践

### 构建优化

#### 使用 BuildKit 加快构建

Docker BuildKit 提供更快的构建和更好的缓存：

```bash
export DOCKER_BUILDKIT=1
docker-compose build --no-cache
```

#### 理解 Dockerfile 的多阶段构建

我们的 Dockerfile 使用多阶段构建：

- **Builder 阶段**: 编译所有源代码（~8-10GB，临时）
- **Runtime 阶段**: 仅复制编译后的二进制和库（~2-3GB，最终镜像）

这样可以：
- ✅ 隐藏源代码
- ✅ 减小最终镜像大小
- ✅ 加快 CI/CD 推送速度

#### 利用 Docker 缓存

```bash
# 首次构建（使用缓存）
docker-compose build

# 如果依赖未改变，后续构建会很快（几秒钟）
# 如果需要完整重新构建，使用：
docker-compose build --no-cache
```

### 容器运行时优化

#### 内存限制（用于内存受限系统）

在 docker-compose.yml 中限制容器内存：

```yaml
deploy:
  resources:
    limits:
      memory: 4G
    reservations:
      memory: 2G
```

运行时内存需求：
- **空闲**: ~200MB
- **运行控制器**: ~500MB-1GB（取决于加载的轨迹）
- **运行规划**: ~2GB+（MoveIt2 很消耗内存）

#### CPU 限制（可选）

```yaml
deploy:
  resources:
    limits:
      cpus: '2'         # 限制使用 2 个 CPU 核心
```

## 🔄 开发工作流

### 推荐流程

#### 场景 1: 快速原型开发

1. 启动容器并挂载源代码：
   ```bash
   # docker-compose.yml 中配置卷
   docker-compose up -d
   ```

2. 在容器内编译和测试：
   ```bash
   docker-compose exec robotic-arm bash
   cd /root/robotic_arm_ws
   colcon build --packages-select arm_controller
   ```

3. 修改代码后自动编译：
   ```bash
   # 在容器内连续监视和构建
   watch -n 5 "colcon build --packages-select arm_controller"
   ```

#### 场景 2: 最终发布

1. 确保代码在容器内测试通过
2. 重新构建干净的镜像（无源代码挂载）：
   ```bash
   # 移除 docker-compose.yml 中的源代码卷
   docker-compose down
   docker-compose build --no-cache
   ```

3. 打标记并推送：
   ```bash
   docker tag dyk/robotic-arm-controller:latest dyk/robotic-arm-controller:v1.0
   docker push dyk/robotic-arm-controller:v1.0
   ```

## 🔒 安全最佳实践

### ✅ 当前实现

- **源代码隐藏**: 使用多阶段构建，镜像中不包含源代码
- **最小化镜像**: 仅包含必需的运行时库和依赖
- **官方基础镜像**: 使用 `ros:humble-ros-base`
- **定期更新**: Dockerfile 中 `apt-get` 自动更新依赖

### 🔐 推荐额外措施

#### 定期更新基础镜像

```bash
# 重新构建以获取最新的 ROS 和系统补丁
docker-compose build --no-cache
```

#### 镜像签名（企业/公司内部使用）

```bash
# 使用 Docker Content Trust 对镜像签名
export DOCKER_CONTENT_TRUST=1
docker push dyk/robotic-arm-controller:v1.0
```

#### 容器运行时安全

```bash
# 以只读方式运行根文件系统（增加安全性）
# docker-compose.yml:
read_only: true
tmpfs:
  - /tmp
  - /run
```

## 📚 更多资源

- [Docker 官方文档](https://docs.docker.com/)
- [Docker Compose 文档](https://docs.docker.com/compose/)
- [ROS 2 Docker 镜像](https://hub.docker.com/_/ros)
- [Universal Arm Controller GitHub](https://github.com/Ding-Kaiyue/universal-arm-controller)

## 🆘 获取帮助

遇到问题？

1. **查看日志**: `docker-compose logs -f robotic-arm`
2. **提交 Issue**: [GitHub Issues](https://github.com/Ding-Kaiyue/universal-arm-controller/issues)
3. **发送邮件**: <kaiyue.ding@raysense.com>

---

## 💡 总结

| 方面 | 优势 |
|------|------|
| **内存受限系统** | ✅ Docker 是唯一稳定方案（避免卡死） |
| **快速部署** | ✅ 一条命令启动，无需配置 |
| **环境一致性** | ✅ 所有用户运行相同的编译配置 |
| **开发效率** | ✅ 容器内编译快速，不影响主机 |
| **安全性** | ✅ 源代码隐藏，镜像精简 |

**最后的建议**: 如果你的系统内存 < 16GB，请使用 Docker。这是最稳定、最可靠的部署方式。

