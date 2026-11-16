# ROS 2 Jazzy on Ubuntu 24.04, for the Apple Silicon Mac this repo is built on.
# Gazebo is not installed. This image only runs the speed decision.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV MOWER_ROOT=/work
ENV PYTHONPATH=/work:/work/ros2/mower_supervisor

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-jazzy-sensor-msgs \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends python3-shapely \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY mower /work/mower
COPY ros2/mower_supervisor /work/ros2/mower_supervisor

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && python3 -m mower_supervisor.check"]
