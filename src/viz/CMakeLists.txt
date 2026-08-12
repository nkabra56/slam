add_library(slam_viz pangolin_viewer.cpp)

target_link_libraries(slam_viz PUBLIC slam_core Pangolin::Pangolin)

target_include_directories(slam_viz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(slam_viz PUBLIC cxx_std_17)
