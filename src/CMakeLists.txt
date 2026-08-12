add_library(slam_core
  sensors/kitti_dataset.cpp
  frontend_vio/vio_frontend.cpp
  frontend_lidar/lidar_frontend.cpp
  backend/optimizer.cpp
  mapping/map.cpp
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

target_include_directories(slam_core PUBLIC ${OpenCV_INCLUDE_DIRS})

target_compile_features(slam_core PUBLIC cxx_std_17)
