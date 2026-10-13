# ORB-SLAM3 (pinned), headless; native build for the host arch (tested on arm64 / Mac M4; amd64 untested). Build context = methods/orbslam3
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
# Pinned versions (see README)
ARG ORBSLAM3_COMMIT=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
ARG PANGOLIN_TAG=v0.8
ARG BUILD_JOBS=2

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git ca-certificates pkg-config \
      libeigen3-dev libopencv-dev libboost-serialization-dev libssl-dev \
      libgl1-mesa-dev libglew-dev libepoxy-dev libegl1-mesa-dev \
      libwayland-dev libxkbcommon-dev wayland-protocols \
      libpng-dev libjpeg-dev gawk \
    && rm -rf /var/lib/apt/lists/*

# Pangolin: only needed to compile/link (viewer is never instantiated at runtime)
RUN git clone --depth 1 --branch ${PANGOLIN_TAG} https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin \
 && cmake -S /opt/Pangolin -B /opt/Pangolin/build -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_TESTS=OFF \
 && cmake --build /opt/Pangolin/build -j${BUILD_JOBS} && cmake --install /opt/Pangolin/build && ldconfig \
 && rm -rf /opt/Pangolin/build

# ORB-SLAM3 at pinned commit + patches
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /opt/ORB_SLAM3 \
 && git -C /opt/ORB_SLAM3 checkout ${ORBSLAM3_COMMIT}
COPY patches/ /tmp/patches/
RUN cd /opt/ORB_SLAM3 && for p in /tmp/patches/*.patch; do echo "apply $p"; git apply "$p"; done

WORKDIR /opt/ORB_SLAM3
RUN cmake -S Thirdparty/DBoW2 -B Thirdparty/DBoW2/build -DCMAKE_BUILD_TYPE=Release && cmake --build Thirdparty/DBoW2/build -j${BUILD_JOBS} \
 && cmake -S Thirdparty/g2o -B Thirdparty/g2o/build -DCMAKE_BUILD_TYPE=Release && cmake --build Thirdparty/g2o/build -j${BUILD_JOBS} \
 && tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j${BUILD_JOBS} \
 && ls -la Examples/Monocular-Inertial/mono_inertial_euroc Examples/Stereo-Inertial/stereo_inertial_euroc Vocabulary/ORBvoc.txt

COPY run.sh /usr/local/bin/run_orbslam3
COPY config/ /opt/vio-config/
RUN chmod +x /usr/local/bin/run_orbslam3
ENV ORB_SLAM3_ROOT=/opt/ORB_SLAM3
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/run_orbslam3"]
