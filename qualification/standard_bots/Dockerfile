# syntax=docker/dockerfile:1.7
FROM ros@sha256:bab7e640bf79cd84957e4e18fcba7d87efc3385b4e3f36a32eeca01638e43206

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake g++ python3-colcon-common-extensions \
      ros-jazzy-ament-cmake-gtest ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-sensor-msgs ros-jazzy-std-msgs ros-jazzy-trajectory-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY --from=reforge_artifacts . /tmp/reforge-artifacts/
RUN apt-get update && apt-get install -y --no-install-recommends \
      /tmp/reforge-artifacts/reforge-core-shaper_2.0.9-908_amd64.deb \
      /tmp/reforge-artifacts/reforge-core-shaper-ros2_2.0.9-908_amd64.deb \
    && rm -rf /var/lib/apt/lists/* /tmp/reforge-artifacts

WORKDIR /workspace
COPY . /workspace
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --base-paths qualification/standard_bots \
       --packages-select reforge_standard_bots_qualification \
       --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && source install/setup.bash \
    && colcon test --base-paths qualification/standard_bots \
       --packages-select reforge_standard_bots_qualification \
       --event-handlers console_direct+ \
    && colcon test-result --verbose

ENTRYPOINT ["/bin/bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["standard_bots_qualification_cli", "--help"]
