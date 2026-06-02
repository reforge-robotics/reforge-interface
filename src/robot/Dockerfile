# {~.~} Include for robots that require ROS2
# FROM ros:jazzy
FROM python:3.11-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC

RUN apt-get update && \
    apt-get install -y \
      build-essential \
      cmake \
      git \
      pkg-config \
      wget \
      unzip \
      ca-certificates \
      zlib1g-dev \
      libeigen3-dev \
      libfftw3-dev

# {~.~} Use ROS2 packages only when needed
# RUN apt-get install -y \
#       ros-jazzy-rmw-cyclonedds-cpp \
#       ros-jazzy-trajectory-msgs \
#       ros-jazzy-std-msgs \
#       ros-jazzy-builtin-interfaces \
#       ros-jazzy-geometry-msgs \
#       ros-jazzy-sensor-msgs \
#       ros-jazzy-rclcpp

RUN rm -rf /var/lib/apt/lists/*

RUN mkdir -p /control-box-bot/reforge-interface/src/robot
WORKDIR /control-box-bot/reforge-interface/src/robot

ENV VIRTUAL_ENV=/opt/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN python -m venv "$VIRTUAL_ENV"

COPY requirements.txt pyproject.toml README.md ./
RUN python -m pip install --no-cache-dir -r requirements.txt

# {~.~} source ROS2 packages
# RUN . /opt/ros/jazzy/setup.sh

COPY . .
RUN python -m pip install --no-cache-dir -e .

# {~.~} Set environment variables for ROS2 and Python
ENV PYTHONPATH=/control-box-bot/reforge-interface/src/robot
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=""
ENV PYTHONUNBUFFERED=1

# Enable core dumps for debugging.
RUN echo 'ulimit -c unlimited' >> /etc/bash.bashrc
ENV SEGFAULT_SIGNALS="all"
