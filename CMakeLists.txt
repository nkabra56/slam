cmake_minimum_required(VERSION 3.20)
project(slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SLAM_BUILD_TESTS "Build unit tests" ON)
option(SLAM_BUILD_APPS "Build command-line apps" ON)
option(SLAM_BUILD_PYTHON_BINDINGS "Build pybind11 Python bindings" OFF)
# Off by default: pulls in OpenGL/X11 dependencies nothing else needs, and
# requires the vcpkg manifest's "viz" feature (not installed by default --
# see vcpkg.json and README.md). See slam::viz::PangolinViewer's doc
# comment for why this is opt-in rather than part of the default build.
option(SLAM_BUILD_VIZ "Build the Pangolin live viewer" OFF)

find_package(Eigen3 CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

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
