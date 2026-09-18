add_executable(slam_tests
  common/types_test.cpp
  sensors/kitti_dataset_test.cpp
  sensors/kitti_raw_imu_test.cpp
  frontend_vio/feature_tracker_test.cpp
  frontend_vio/imu_preintegrator_test.cpp
  frontend_vio/stereo_geometry_test.cpp
  frontend_lidar/voxel_grid_test.cpp
  frontend_lidar/ground_removal_test.cpp
  frontend_lidar/kdtree_test.cpp
  frontend_lidar/scan_features_test.cpp
  frontend_lidar/scan_matcher_test.cpp
  frontend_lidar/lidar_frontend_test.cpp
  backend/pose_graph_test.cpp
  backend/loop_closure_test.cpp
  backend/optimizer_test.cpp
  backend/nav_state_test.cpp
  backend/imu_factor_test.cpp
  backend/nav_state_graph_test.cpp
  backend/vio_initializer_test.cpp
  backend/tightly_coupled_optimizer_test.cpp
  mapping/point_cloud_map_test.cpp
  mapping/map_test.cpp
  eval/trajectory_metrics_test.cpp
)

target_link_libraries(slam_tests PRIVATE slam_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(slam_tests)
