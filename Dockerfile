# ============================================================
# Stage 1: Builder
# ============================================================
FROM ros:humble-ros-base AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8

# 接收代理参数
ARG HTTP_PROXY=""
ARG HTTPS_PROXY=""
ARG NO_PROXY=""
ARG APT_MIRROR=""

# 设置环境变量（如果提供了参数）
ENV http_proxy=${HTTP_PROXY} \
    https_proxy=${HTTPS_PROXY} \
    HTTP_PROXY=${HTTP_PROXY} \
    HTTPS_PROXY=${HTTPS_PROXY} \
    no_proxy=${NO_PROXY} \
    NO_PROXY=${NO_PROXY}

# 如果提供了镜像源，配置 apt
RUN if [ -n "$APT_MIRROR" ]; then \
      sed -i "s|http://archive.ubuntu.com/ubuntu|$APT_MIRROR|g" /etc/apt/sources.list && \
      sed -i "s|http://security.ubuntu.com/ubuntu|$APT_MIRROR|g" /etc/apt/sources.list && \
      sed -i "s|https://mirrors.aliyun.com/ubuntu/ubuntu|$APT_MIRROR|g" /etc/apt/sources.list; \
    fi

# 配置 apt 使用代理（如果提供了）
RUN if [ -n "$HTTP_PROXY" ]; then \
      echo "Acquire::http::Proxy \"$HTTP_PROXY\";" > /etc/apt/apt.conf.d/proxy.conf && \
      echo "Acquire::https::Proxy \"$HTTPS_PROXY\";" >> /etc/apt/apt.conf.d/proxy.conf; \
    fi

# 配置 apt 重试机制
RUN echo 'APT::Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries && \
    echo 'Acquire::http::Timeout "30";' >> /etc/apt/apt.conf.d/80-retries && \
    echo 'Acquire::https::Timeout "30";' >> /etc/apt/apt.conf.d/80-retries

# 配置 Git 以支持网络访问
RUN git config --global url."https://".insteadOf git:// && \
    git config --global http.sslVerify false && \
    git config --global http.connectTimeout 30 && \
    git config --global http.lowSpeedLimit 0 && \
    git config --global http.lowSpeedTime 999999

# ---------- 系统依赖（不含 eigenpy） ----------
RUN apt-get update && apt-get install -y \
    ca-certificates \
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
    ros-humble-moveit-core \
    ros-humble-moveit-msgs \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-tf2-kdl \
    ros-humble-kdl-parser \
    ros-humble-control-msgs \
    liburdfdom-headers-dev \
    liburdfdom-dev \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# 1. NLopt（离线）
# ============================================================
COPY deps/nlopt /tmp/nlopt
RUN cmake -S /tmp/nlopt -B /tmp/nlopt/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DNLOPT_CXX=ON \
      -DNLOPT_PYTHON=OFF && \
    cmake --build /tmp/nlopt/build --parallel 1 && \
    cmake --install /tmp/nlopt/build && \
    ldconfig && rm -rf /tmp/nlopt

# ============================================================
# 2. QDLDL（离线）
# ============================================================
COPY deps/qdldl /tmp/qdldl
RUN rm -rf /tmp/qdldl/build && \
    cmake -S /tmp/qdldl -B /tmp/qdldl/build \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/qdldl/build --parallel 1 && \
    cmake --install /tmp/qdldl/build && \
    ldconfig && rm -rf /tmp/qdldl

# ============================================================
# 3. OSQP（离线 + 使用系统 QDLDL）
# ============================================================
COPY deps/osqp /tmp/osqp
# patch OSQP CMakeLists.txt 防止 FetchContent 下载 qdldl
RUN sed -i '/FetchContent_Declare(qdldl/d' /tmp/osqp/CMakeLists.txt && \
    sed -i '/FetchContent_MakeAvailable(qdldl)/d' /tmp/osqp/CMakeLists.txt
RUN rm -rf /tmp/osqp/build && \
    cmake -S /tmp/osqp -B /tmp/osqp/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DOSQP_USE_SYSTEM_QDLDL=ON \
      -DOSQP_BUILD_SHARED_LIB=ON \
      -DOSQP_BUILD_STATIC_LIB=OFF \
      -DOSQP_ENABLE_TESTING=OFF && \
    cmake --build /tmp/osqp/build --parallel 1 && \
    cmake --install /tmp/osqp/build && \
    ldconfig && rm -rf /tmp/osqp

# ============================================================
# 4. OsqpEigen（离线）
# ============================================================
COPY deps/osqp-eigen /tmp/OsqpEigen
RUN rm -rf /tmp/OsqpEigen/build && \
    cmake -S /tmp/OsqpEigen -B /tmp/OsqpEigen/build \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/OsqpEigen/build --parallel 1 && \
    cmake --install /tmp/OsqpEigen/build && \
    ldconfig && rm -rf /tmp/OsqpEigen

# ============================================================
# 5. EigenPy（在线 clone jrl-cmakemodules）
# ============================================================
COPY deps/eigenpy /tmp/eigenpy
RUN rm -rf /tmp/eigenpy/build && \
    cmake -S /tmp/eigenpy -B /tmp/eigenpy/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DCMAKE_FIND_DEBUG_MODE=OFF && \
    cmake --build /tmp/eigenpy/build --parallel 1 -- VERBOSE=1 && \
    cmake --install /tmp/eigenpy/build && \
    ldconfig && rm -rf /tmp/eigenpy

# ============================================================
# 6. Pinocchio（使用系统 eigenpy）
# ============================================================
COPY deps/pinocchio /tmp/pinocchio
RUN rm -rf /tmp/pinocchio/build && \
    cmake -S /tmp/pinocchio -B /tmp/pinocchio/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_PYTHON_INTERFACE=ON \
      -DBUILD_TESTING=OFF && \
    cmake --build /tmp/pinocchio/build --parallel 1 && \
    cmake --install /tmp/pinocchio/build && \
    ldconfig && rm -rf /tmp/pinocchio

# ============================================================
# 7. Workspace
# ============================================================
WORKDIR /opt/robotic_arm_ws/src

# ---------- TRAC-IK ----------
COPY deps/trac_ik trac_ik

# ---------- 你的 ROS 包 ----------
COPY src/arm_controller arm_controller
COPY src/controller_interfaces controller_interfaces
COPY src/robotic_arm_bringup robotic_arm_bringup
COPY src/trajectory_planning trajectory_planning
COPY src/hardware_driver hardware_driver
COPY src/trajectory_interpolator trajectory_interpolator
COPY src/csaps csaps

# ============================================================
# 8. colcon build
# ============================================================
WORKDIR /opt/robotic_arm_ws
ENV CXXFLAGS="-Wno-error=maybe-uninitialized"
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --executor sequential \
      --parallel-workers 1 \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
"

# ============================================================
# Stage 2: Runtime
# ============================================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8

# ---------- 最小运行时依赖 ----------
RUN apt-get update && apt-get install -y \
    liborocos-kdl1.5 \
    libnlopt0 \
    ros-humble-control-msgs \
    python3-numpy \
    python3-scipy \
    git \
    ca-certificates \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ---------- 拷贝 Stage 1 构建好的 workspace ----------
COPY --from=builder /opt/robotic_arm_ws/install /opt/robotic_arm_ws/install

# ---------- 入口脚本 ----------
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.sh && \
    echo 'source /opt/robotic_arm_ws/install/setup.bash' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

WORKDIR /root
