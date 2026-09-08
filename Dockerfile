# Multi-stage build for the slam project.
#
#   docker build --target runtime -t slam:runtime .   (default target)
#   docker build --target ros2    -t slam:ros2 .      (adds the ROS2 overlay)
#
# The `deps` stage builds slam_core + apps and runs the full test suite
# (`ctest`) as part of the image build -- `docker build` fails if any test
# fails.

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
    autoconf-archive \
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
# Stage: ros2 -- the ROS2 overlay (ros2_ws/src/slam_ros2), built against the
# `deps` stage's installed slam_core via find_package(slam CONFIG REQUIRED)
# (see PHASE6_PLAN.md section 3.6).
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
# slamConfig.cmake's find_dependency(Eigen3/Sophus/OpenCV) needs these
# findable too; `cmake --install` only exports slam_core itself, so copy
# vcpkg's installed tree to keep slam_core linked against the exact OpenCV
# build it was compiled against, not apt's (e.g. cv_bridge's below --
# a cv::Mat crossing that boundary still spans two OpenCV builds).
COPY --from=deps /workspace/vcpkg_installed/x64-linux /opt/vcpkg_installed/x64-linux

WORKDIR /ros2_ws
COPY ros2_ws/src ./src

RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args "-DCMAKE_PREFIX_PATH=/opt/slam;/opt/vcpkg_installed/x64-linux"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["/bin/bash"]
