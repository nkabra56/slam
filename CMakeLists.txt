cmake_minimum_required(VERSION 3.20)
project(slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default single-config generators to Release -- otherwise this dense
# Eigen/Sophus math silently compiles at -O0.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(SLAM_BUILD_TESTS "Build unit tests" ON)
option(SLAM_BUILD_APPS "Build command-line apps" ON)
option(SLAM_BUILD_PYTHON_BINDINGS "Build pybind11 Python bindings" OFF)
# Off by default: pulls in OpenGL/X11 deps + the vcpkg "viz" feature.
# See slam::viz::PangolinViewer's doc comment for why it's opt-in.
option(SLAM_BUILD_VIZ "Build the Pangolin live viewer" OFF)
# Off by default: requires the vcpkg "cuda" feature and a local NVIDIA
# CUDA Toolkit install -- see vcpkg.json and the with-cuda preset.
option(SLAM_USE_CUDA_OPTFLOW "Use OpenCV CUDA KLT tracking in the VIO frontend" OFF)

find_package(Eigen3 CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

if(SLAM_USE_CUDA_OPTFLOW AND NOT OpenCV_CUDA_VERSION)
  message(FATAL_ERROR "SLAM_USE_CUDA_OPTFLOW requires an OpenCV build with CUDA (use the with-cuda preset)")
endif()

add_subdirectory(src)

if(SLAM_BUILD_VIZ)
  find_package(Pangolin CONFIG REQUIRED)
  add_subdirectory(src/viz)
endif()

if(SLAM_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(SLAM_BUILD_TESTS)
  enable_testing()
  find_package(GTest CONFIG REQUIRED)
  add_subdirectory(tests)
endif()

if(SLAM_BUILD_PYTHON_BINDINGS)
  find_package(pybind11 CONFIG REQUIRED)
  add_subdirectory(bindings)
endif()

# Install/export so external CMake projects (e.g. ros2_ws's ament_cmake
# overlay, PHASE6_PLAN.md section 3.6) can find_package(slam CONFIG REQUIRED).
include(GNUInstallDirs)

install(TARGETS slam_core
  EXPORT slamTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT slamTargets
  FILE slamTargets.cmake
  NAMESPACE slam::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/slam
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/slamConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/slamConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/slam
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/slamConfigVersion.cmake
  VERSION 0.1.0
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/slamConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/slamConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/slam
)
