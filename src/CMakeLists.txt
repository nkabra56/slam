add_library(slam_core
  sensors/kitti_dataset.cpp
  sensors/kitti_raw_imu.cpp
  frontend_vio/feature_tracker.cpp
  frontend_vio/imu_preintegrator.cpp
  frontend_vio/stereo_geometry.cpp
  frontend_vio/vio_frontend.cpp
  frontend_lidar/voxel_grid.cpp
  frontend_lidar/ground_removal.cpp
  frontend_lidar/kdtree.cpp
  frontend_lidar/scan_features.cpp
  frontend_lidar/scan_matcher.cpp
  frontend_lidar/lidar_frontend.cpp
  backend/pose_graph.cpp
  backend/loop_closure.cpp
  backend/optimizer.cpp
  backend/nav_state.cpp
  backend/imu_factor.cpp
  backend/nav_state_graph.cpp
  backend/vio_initializer.cpp
  backend/tightly_coupled_optimizer.cpp
  mapping/point_cloud_map.cpp
  mapping/map.cpp
  eval/trajectory_metrics.cpp
)

target_include_directories(slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(slam_core PUBLIC
  Eigen3::Eigen
  Sophus::Sophus
  ${OpenCV_LIBS}
)

target_compile_features(slam_core PUBLIC cxx_std_17)
