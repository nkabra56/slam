cmake_minimum_required(VERSION 3.20)
project(slam_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# slam_core from the main (non-ROS) build: a plain CMake package, not an
# ament one. Requires CMAKE_PREFIX_PATH to include wherever
# `cmake --install` put it (see PHASE6_PLAN.md section 3.6 and this repo's
# root README.md's ROS2 section) -- e.g.:
#   colcon build --cmake-args -DCMAKE_PREFIX_PATH=/path/to/slam/install
find_package(slam CONFIG REQUIRED)

add_library(slam_ros2_lib
  src/message_adapters.cpp
  src/slam_node.cpp
)
target_include_directories(slam_ros2_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(slam_ros2_lib PUBLIC slam::slam_core)
ament_target_dependencies(slam_ros2_lib PUBLIC
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
  message_filters
)

add_executable(slam_node src/slam_node_main.cpp)
target_link_libraries(slam_node PRIVATE slam_ros2_lib)

install(TARGETS slam_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS slam_ros2_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_message_adapters test/test_message_adapters.cpp)
  target_link_libraries(test_message_adapters slam_ros2_lib)
endif()

ament_package()
