# OpenVINS (filter-based MSCKF VIO) on ROS1 Noetic, headless; native build for the host arch (tested on arm64 / Mac M4; amd64 untested).
# Pinned: rpng/open_vins v2.7 (commit 93adc241390d13e99232652cf05cbe18a93c7bea)
FROM ros:noetic-ros-base

ARG OV_TAG=v2.7
ARG OV_COMMIT=93adc241390d13e99232652cf05cbe18a93c7bea
ARG JOBS=2

RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential cmake python3-catkin-tools python3-osrf-pycommon \
      libeigen3-dev libboost-all-dev libceres-dev libopencv-dev libopencv-contrib-dev \
      ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-tf ros-noetic-rosbag \
      ros-noetic-cmake-modules ros-noetic-nav-msgs ros-noetic-visualization-msgs \
      ros-noetic-sensor-msgs ros-noetic-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Fetch pinned source, build only ov_core/ov_init/ov_msckf (no ov_eval/rviz), then drop build tree.
RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src \
 && git clone https://github.com/rpng/open_vins.git \
 && cd open_vins && git checkout ${OV_COMMIT} && git describe --tags --exact-match | grep -x ${OV_TAG} \
 && cd /catkin_ws && . /opt/ros/noetic/setup.sh \
 && catkin_make -j${JOBS} -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_WHITELIST_PACKAGES="ov_core;ov_init;ov_msckf" \
 && rm -rf build /catkin_ws/src/open_vins/.git

COPY launch /opt/vio/launch
COPY scripts /opt/vio/scripts
COPY config /opt/vio/config
COPY run.sh /opt/vio/run.sh
RUN chmod +x /opt/vio/run.sh /opt/vio/scripts/*

WORKDIR /work
ENTRYPOINT ["/opt/vio/run.sh"]
