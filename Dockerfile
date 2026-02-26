# {~.~} Include for robots that require ROS2 
FROM ros:jazzy 

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC

RUN apt-get update && \
    apt-get install -y \
      python3 \
      python3-pip \
      python3-venv \
      python3-dev \
      build-essential \
      cmake \
      git \
      pkg-config \
      wget \
      unzip \
      ca-certificates \
      zlib1g-dev

# {~.~} Use ROS2 packages only when needed
RUN apt-get install -y \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-trajectory-msgs \
      ros-jazzy-std-msgs \
      ros-jazzy-builtin-interfaces \
      ros-jazzy-geometry-msgs \
      ros-jazzy-sensor-msgs \
      ros-jazzy-rclcpp 

RUN rm -rf /var/lib/apt/lists/*

# {~.~} Change the name of the directory as needed
RUN mkdir -p /control-box-bot/reforge-interface
WORKDIR /control-box-bot/reforge-interface

ENV VIRTUAL_ENV=/opt/venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN python3 -m venv "$VIRTUAL_ENV"

COPY requirements.txt pyproject.toml MANIFEST.in ./
RUN python -m pip install --no-cache-dir -r requirements.txt

# {~.~} source ROS2 packages
RUN . /opt/ros/jazzy/setup.sh

COPY . .
RUN python -m pip install --no-cache-dir .

# {~.~} Set environment variables for ROS2 and Python
ENV PYTHONPATH=/control-box-bot/reforge-interface:/control-box-bot/reforge-interface/src
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI="/etc/standardbots/configuration/cyclonedds.xml"
ENV PYTHONUNBUFFERED=1

# Enable core dumps for debugging
RUN echo 'ulimit -c unlimited' >> /etc/bash.bashrc
ENV SEGFAULT_SIGNALS="all"
