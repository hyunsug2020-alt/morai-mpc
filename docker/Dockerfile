# ============================================================
# MORAI MPC - Ubuntu 22.04 / ROS2 Humble
# 차량: Ioniq 5 SUV
# 제어: 전진=LTV-MPC  /  후진주차=RTI-NMPC
# 경로: Hybrid A* (주차), 글로벌 경로 (전진)
# 항법: ESKF (GPS+IMU)
# GUI: Iridescence (pyridescence)
# ============================================================
FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 ROS_DISTRO=humble

# ── 시스템 패키지 ──────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl \
    python3-pip python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libopencv-dev python3-opencv \
    libglfw3-dev libpng-dev libjpeg-dev libglm-dev \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-nav2-msgs \
    ros-humble-ament-cmake \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ── OSQP ──────────────────────────────────────────────────
RUN pip3 install osqp

# ── Iridescence ───────────────────────────────────────────
RUN add-apt-repository -y ppa:koide3/iridescence && \
    apt-get update && \
    apt-get install -y libiridescence-dev && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install pyridescence

# ── Python 패키지 ─────────────────────────────────────────
RUN pip3 install numpy scipy pyproj transforms3d matplotlib

# ── MORAI 메시지 ──────────────────────────────────────────
RUN mkdir -p /home/moraimpc/src && \
    cd /home/moraimpc/src && \
    git clone https://github.com/morai-developergroup/morai_msgs.git \
    || echo "[WARN] morai_msgs 수동 설치 필요"

# ── 워크스페이스 복사 ─────────────────────────────────────
WORKDIR /home/moraimpc
COPY src/ /home/moraimpc/src/

# rosdep
RUN rosdep init 2>/dev/null || true && rosdep update

# ── 빌드 ──────────────────────────────────────────────────
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    " || echo "[WARN] 빌드 일부 실패"

# ── 환경 ──────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/moraimpc/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
