add_executable(slam_tests
  common/types_test.cpp
  sensors/kitti_dataset_test.cpp
  frontend_vio/feature_tracker_test.cpp
  frontend_vio/imu_preintegrator_test.cpp
  frontend_vio/stereo_geometry_test.cpp
)

target_link_libraries(slam_tests PRIVATE slam_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(slam_tests)
