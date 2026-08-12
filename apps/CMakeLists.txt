add_executable(slam_kitti_demo main_kitti_demo.cpp)
target_link_libraries(slam_kitti_demo PRIVATE slam_core)

add_executable(slam_vio_demo main_vio_demo.cpp)
target_link_libraries(slam_vio_demo PRIVATE slam_core)

add_executable(slam_lidar_demo main_lidar_demo.cpp)
target_link_libraries(slam_lidar_demo PRIVATE slam_core)

add_executable(slam_backend_demo main_backend_demo.cpp)
target_link_libraries(slam_backend_demo PRIVATE slam_core)

add_executable(slam_mapping_demo main_mapping_demo.cpp)
target_link_libraries(slam_mapping_demo PRIVATE slam_core)

add_executable(slam_eval_demo main_eval_demo.cpp)
target_link_libraries(slam_eval_demo PRIVATE slam_core)

if(SLAM_BUILD_VIZ)
  add_executable(slam_viz_demo main_viz_demo.cpp)
  target_link_libraries(slam_viz_demo PRIVATE slam_core slam_viz)
endif()
