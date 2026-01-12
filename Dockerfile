# 多阶段构建：第一阶段编译，第二阶段只保留运行时
FROM ros:humble-ros-base as builder

LABEL maintainer="dyk@example.com"
LABEL description="Universal Arm Controller - Builder Stage"

# 安装构建依赖
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
WORKDIR /root/robotic_arm_ws/src

# 克隆源代码
RUN git clone --depth 1 https://github.com/Ding-Kaiyue/universal-arm-controller.git

# 导入依赖
WORKDIR /root/robotic_arm_ws/src/universal-arm-controller
RUN vcs import . < deps.repos --recursive

# 安装 ROS 依赖
WORKDIR /root/robotic_arm_ws
RUN rosdep install --from-paths src --ignore-src -r -y

# 编译（使用单线程避免内存问题）
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
    --executor sequential \
    --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1 -DCMAKE_BUILD_TYPE=Release

# ============================================================================
# 第二阶段：运行时镜像（只包含编译后的二进制和库，不含源码）
# ============================================================================
FROM ros:humble-ros-base

LABEL maintainer="dyk@example.com"
LABEL description="Universal Arm Controller - Runtime"

# 安装运行时依赖（精简版）
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    # MoveIt 运行时依赖
    libmoveit-core0d \
    libmoveit-ros-planning \
    moveit-resources-prbt-moveit-config \
    python3-moveit-commander \
    # 其他工具
    nano \
    curl \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间目录结构
RUN mkdir -p /opt/robotic_arm_ws/install

# 从构建阶段复制编译后的 install 目录
COPY --from=builder /root/robotic_arm_ws/install /opt/robotic_arm_ws/install

# 设置环境
ENV ROS_DISTRO=humble
ENV COLCON_CURRENT_PREFIX=/opt/robotic_arm_ws/install

# 设置启动脚本
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.sh && \
    echo 'source /opt/robotic_arm_ws/install/setup.bash' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# 工作目录
WORKDIR /root
