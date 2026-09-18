# Multi-stage build: deps (build+test) -> runtime (slim demo image) ->
# ros2 (ROS2 overlay). See README.md for docker build/run commands.

# deps: builds slam_core + apps, runs the full test suite via ctest.
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

# Install deps before copying the rest of the source, so this slow
# vcpkg-builds-OpenCV-from-source layer is cached independently of it.
COPY vcpkg.json ./
RUN "${VCPKG_ROOT}/vcpkg" install --triplet x64-linux

COPY . .

RUN cmake --preset default \
    && cmake --build build --parallel \
    && ctest --test-dir build --output-on-failure

RUN cmake --install build --prefix /opt/slam

# runtime: slim image with the built demo apps (default target).
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

# ros2: ROS2 overlay (ros2_ws/src/slam_ros2) built against deps' installed
# slam_core via find_package(slam CONFIG REQUIRED).
FROM ros:humble-ros-base AS ros2

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

COPY --from=deps /opt/slam /opt/slam
# slamConfig.cmake's find_dependency() needs Eigen3/Sophus/OpenCV findable;
# copy vcpkg's tree so cv_bridge and slam_core share one OpenCV build.
COPY --from=deps /workspace/vcpkg_installed/x64-linux /opt/vcpkg_installed/x64-linux

WORKDIR /ros2_ws
COPY ros2_ws/src ./src

RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args "-DCMAKE_PREFIX_PATH=/opt/slam;/opt/vcpkg_installed/x64-linux"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["/bin/bash"]
