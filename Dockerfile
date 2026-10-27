# ur5e-teaching-controllers — reproducible ROS 2 Lyrical environment.
# Base tag verified through the Docker Hub registry API on 2026-10-28 (see NOTES.md).
FROM ros:lyrical-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dev

# Simulation, control and dynamics packages used by this repository (all apt binaries on Lyrical).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-lyrical-ur-simulation-gz \
      ros-lyrical-ur-description \
      ros-lyrical-gz-ros2-control \
      ros-lyrical-ros2-control \
      ros-lyrical-ros2-controllers \
      ros-lyrical-ros2controlcli \
      ros-lyrical-ros-gz-bridge \
      ros-lyrical-ros-gz-sim \
      ros-lyrical-ros-gz-interfaces \
      ros-lyrical-pinocchio \
      ros-lyrical-generate-parameter-library \
      ros-lyrical-realtime-tools \
      ros-lyrical-kinematics-interface \
      ros-lyrical-kinematics-interface-kdl \
      ros-lyrical-xacro \
      ros-lyrical-robot-state-publisher \
      ros-lyrical-ros2-control-test-assets \
      ros-lyrical-ament-cmake-gmock \
      python3-colcon-common-extensions \
      python3-numpy \
      python3-matplotlib \
      python3-opencv \
      ffmpeg \
      xvfb \
      git \
    && rm -rf /var/lib/apt/lists/*

# Non-root user for interactive (devcontainer) use. Ubuntu 26.04 images already ship a
# uid-1000 'ubuntu' user, so no fixed uid is requested here.
RUN useradd -m -s /bin/bash ${USERNAME} \
    && echo "source /opt/ros/lyrical/setup.bash" >> /home/${USERNAME}/.bashrc

ENV ROS_DOMAIN_ID=11 \
    GZ_PARTITION=r1 \
    MAKEFLAGS=-j4

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
