# Project-owned ROS stack for MORAI autonomous path following + ego/route recording.
# Ubuntu 20.04 / ROS noetic, own roscore + rosbridge, moraimpc LTV-MPC follower, morai_msgs (bundled version).
FROM ros:noetic-ros-base-focal
ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1
RUN sed -i "s|http://archive.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g; s|http://security.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g" /etc/apt/sources.list \
 && apt-get -o Acquire::Retries=5 update && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
      build-essential cmake \
      ros-noetic-rosbridge-server ros-noetic-osqp-vendor ros-noetic-tf2-msgs ros-noetic-tf \
      ros-noetic-velodyne-pointcloud \
      libeigen3-dev libjsoncpp-dev \
      python3-numpy python3-yaml \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/autodrive/ws
COPY ws/src ./src
RUN bash -lc 'set -o pipefail; source /opt/ros/noetic/setup.bash && catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5'
COPY ros /opt/autodrive/ros
RUN chmod +x /opt/autodrive/ros/*.sh /opt/autodrive/ros/*.py
ENV ROS_MASTER_URI=http://127.0.0.1:11312 ROS_IP=127.0.0.1
ENTRYPOINT ["/opt/autodrive/ros/entrypoint.sh"]
CMD ["stack"]
