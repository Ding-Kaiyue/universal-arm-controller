# Docker 快速开始指南

本项目提供 Docker 镜像，用户无需本地编译即可直接使用。

## 快速开始

### 方式 1: 使用 Docker Compose（推荐）

```bash
# 构建镜像
docker-compose build

# 启动容器
docker-compose up -d

# 进入容器
docker-compose exec robotic-arm bash

# 在容器内启动系统
ros2 launch robotic_arm_bringup robotic_arm_real.launch.py
```

### 方式 2: 手动 Docker 命令

```bash
# 构建镜像
docker build -t dyk/robotic-arm-controller:latest .

# 运行容器
docker run -it \
  --network host \
  --ipc host \
  --device /dev/ttyUSB0 \
  --device /dev/can0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=${DISPLAY} \
  dyk/robotic-arm-controller:latest bash
```

## 镜像特性

### ✅ 优势

- **无需编译**: 预编译的二进制，开箱即用
- **源码保护**: Docker 镜像只包含编译后的库，不包含源代码
- **环境隔离**: 避免本地依赖冲突
- **一致性**: 所有用户都在同样的编译环境中运行
- **轻量级**: 多阶段构建，镜像仅包含运行时库

### 镜像大小

- Builder 阶段: ~3GB（包含编译工具）
- Final 镜像: ~800MB（仅包含运行时）

## 配置说明

### docker-compose.yml 配置

#### 网络模式
```yaml
network_mode: host  # ROS 通信需要主机网络
```

#### 设备访问（硬件通信）
```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0  # 串口设备
  - /dev/can0:/dev/can0         # CAN 接口
```

#### GPU 支持（可选）
如果使用 NVIDIA GPU，取消以下注释：
```yaml
runtime: nvidia
```

#### 显示支持（GUI）
```yaml
volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
environment:
  - DISPLAY=${DISPLAY}
```

## 常见问题

### Q1: 如何在容器内访问硬件（CAN/USB）？

确保 docker-compose.yml 中的 `devices` 部分正确配置：

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
  - /dev/can0:/dev/can0
```

然后在容器内应该能看到这些设备：
```bash
ls -la /dev/ttyUSB0
ls -la /dev/can0
```

### Q2: 如何查看容器日志？

```bash
# 实时查看日志
docker-compose logs -f robotic-arm

# 查看特定行数的日志
docker-compose logs --tail 100 robotic-arm
```

### Q3: 如何修改代码并重新编译？

由于源代码不在镜像中，如果需要修改代码：

1. **方式 A**: 重新构建镜像
   ```bash
   docker-compose build --no-cache
   docker-compose up -d
   ```

2. **方式 B**: 挂载源代码（开发模式）

   修改 docker-compose.yml，添加源代码卷：
   ```yaml
   volumes:
     - ./src:/root/robotic_arm_ws/src
     - ./install:/root/robotic_arm_ws/install
   ```

   然后在容器内重新编译：
   ```bash
   cd /root/robotic_arm_ws
   colcon build --packages-select arm_controller
   ```

### Q4: 如何推送镜像到 Docker Hub？

```bash
# 登录
docker login

# 构建
docker build -t your-username/robotic-arm-controller:latest .

# 推送
docker push your-username/robotic-arm-controller:latest
```

### Q5: 容器占用过多磁盘空间，如何清理？

```bash
# 停止并删除容器
docker-compose down

# 删除镜像
docker rmi dyk/robotic-arm-controller:latest

# 清理所有未使用的镜像
docker image prune -a
```

## 性能优化

### 内存使用

Docker 容器默认继承主机内存限制。如果需要限制内存：

```yaml
services:
  robotic-arm:
    mem_limit: 4g  # 限制到 4GB
```

### 构建优化

如果镜像构建太慢，可以用缓存的预构建基础镜像：

```dockerfile
# Dockerfile 中已使用多阶段构建优化
# Builder 阶段的中间层会被缓存
```

## 开发工作流

### 推荐开发流程

1. **本地开发**: 在本地机器上开发代码
2. **容器测试**: 在 Docker 中测试
3. **发布镜像**: 构建并推送镜像供用户使用

```bash
# 开发阶段
docker-compose up -d
docker-compose exec robotic-arm bash
# 在容器内进行测试

# 准备发布
docker build -t dyk/robotic-arm-controller:v1.0 .
docker push dyk/robotic-arm-controller:v1.0
```

## 故障排除

### 问题: 容器启动失败

```bash
# 查看详细日志
docker-compose logs robotic-arm

# 检查镜像是否正确构建
docker images | grep robotic-arm
```

### 问题: ROS 节点无法通信

确保：
1. `network_mode: host` 已设置
2. `ipc: host` 已设置
3. 所有容器都在同一主机上

### 问题: 无法访问硬件设备

```bash
# 检查设备是否存在
ls -la /dev/ttyUSB0

# 检查权限
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0
```

## 安全考虑

### ✅ 最佳实践

- 源代码未打包在镜像中（通过多阶段构建）
- 镜像仅包含必需的运行时库
- 使用官方 ROS 基础镜像
- 定期更新基础镜像和依赖

### 镜像签名（可选）

如果在公司内部使用，可以对镜像进行签名：

```bash
docker trust signer add --key ~/.docker/notary-keys/root_keys/root_key.key dyk/robotic-arm-controller
```

---

有问题？提交 Issue 或联系维护者。
