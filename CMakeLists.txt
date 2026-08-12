cmake_minimum_required(VERSION 3.20)
project(slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SLAM_BUILD_TESTS "Build unit tests" ON)
option(SLAM_BUILD_APPS "Build command-line apps" ON)
option(SLAM_BUILD_PYTHON_BINDINGS "Build pybind11 Python bindings" OFF)

find_package(Eigen3 CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(src)

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
