#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace slam::viz {

// Minimal live 3D viewer for the trajectory + point-cloud map, built with
// Pangolin. Opt-in (SLAM_BUILD_VIZ); PIMPL keeps Pangolin/OpenGL out of this header.
class PangolinViewer {
 public:
  explicit PangolinViewer(std::string window_name = "slam");
  ~PangolinViewer();

  PangolinViewer(const PangolinViewer&) = delete;
  PangolinViewer& operator=(const PangolinViewer&) = delete;

  // Redraws the window with the given trajectory and point clouds. Call
  // once per keyframe/update from the main loop.
  void Update(const std::vector<Sophus::SE3d>& trajectory,
              const std::vector<Eigen::Vector3d>& landmark_points,
              const std::vector<Eigen::Vector3d>& lidar_points);

  // True once the user has closed the window.
  bool ShouldClose() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace slam::viz
