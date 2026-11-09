FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
      python3-shapely python3-scipy python3-pandas python3-matplotlib \
      liburdfdom-tools libegl1 libgl1-mesa-dri mesa-utils sudo git \
    && rm -rf /var/lib/apt/lists/* \
    && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
