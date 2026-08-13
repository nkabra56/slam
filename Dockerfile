# Multi-stage build for the slam project.
#
#   docker build --target runtime -t slam:runtime .   (default target)
#   docker build --target ros2    -t slam:ros2 .      (adds the Phase 6
#                                                       Part B ROS2 overlay)
#
# The `deps` stage does the actual C++ build AND runs the full test suite
# (`ctest`) as part of the image build -- a successful `docker build` is
# therefore real build+test verification of this project, something it did
# not have before this Dockerfile existed (everything up to this point was
# written and reasoned through without a compiler in the loop). If any
# test fails, the build fails.
#
# Caveat: none of this has been built here -- this environment has no
# Docker install. Written from documented apt/vcpkg/ROS2 packaging
# conventions (base image tags, apt package names), which is a different,
# generally lower-risk kind of unverified than slam_node.cpp's ROS2 C++
# library API usage (see ROADMAP.md's Phase 6 section) -- but it is still
# unverified until someone actually runs `docker build`.

# ---------------------------------------------------------------------------
# Stage: deps -- builds slam_core + apps and runs the test suite.
# ---------------------------------------------------------------------------
FROM ubuntu:22.04 AS deps

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    autoconf \
    automake \
    libtool \
    m4 \
    nasm \
    python3 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ENV VCPKG_ROOT=/opt/vcpkg
RUN git clone --depth 1 https://github.com/microsoft/vcpkg.git "${VCPKG_ROOT}" \
    && "${VCPKG_ROOT}/bootstrap-vcpkg.sh" -disableMetrics

WORKDIR /workspace

# Install dependencies before copying the rest of the source, so this slow
# layer (OpenCV et al. built from source by vcpkg -- can take a while) is
# cached independently of ordinary source-code changes.
COPY vcpkg.json ./
RUN "${VCPKG_ROOT}/vcpkg" install --triplet x64-linux

COPY . .

RUN cmake --preset default \
    && cmake --build build --parallel \
    && ctest --test-dir build --output-on-failure

RUN cmake --install build --prefix /opt/slam

# ---------------------------------------------------------------------------
# Stage: runtime -- slim image with the built demo apps, for running
# against a mounted KITTI dataset (see README.md). Default build target.
# ---------------------------------------------------------------------------
FROM ubuntu:22.04 AS runtime

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=deps /opt/slam /opt/slam
COPY --from=deps /workspace/build/apps /opt/slam/bin

ENV PATH=/opt/slam/bin:${PATH}
ENV LD_LIBRARY_PATH=/opt/slam/lib:${LD_LIBRARY_PATH}
WORKDIR /data

CMD ["/bin/bash"]

# ---------------------------------------------------------------------------
# Stage: ros2 -- the Phase 6 Part B ROS2 overlay (ros2_ws/src/slam_ros2),
# built against the `deps` stage's installed slam_core. See
# PHASE6_PLAN.md section 3.6 for the find_package(slam CONFIG REQUIRED)
# design this relies on, and ROADMAP.md's Phase 6 section for why
# slam_node.cpp specifically carries more risk than everything else here.
# ---------------------------------------------------------------------------
FROM ros:humble-ros-base AS ros2

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

COPY --from=deps /opt/slam /opt/slam

WORKDIR /ros2_ws
COPY ros2_ws/src ./src

RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_PREFIX_PATH=/opt/slam

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["/bin/bash"]
