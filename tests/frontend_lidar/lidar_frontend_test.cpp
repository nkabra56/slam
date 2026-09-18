#include "slam/frontend_lidar/lidar_frontend.hpp"

#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::frontend_lidar {
namespace {

constexpr double kPi = 3.14159265358979323846;

// One ring (all z=0) with a sharp corner, so extraction yields edge and planar points.
LidarScan MakeSingleRingWithCorner() {
  constexpr int kNumPoints = 60;
  LidarScan scan;
  for (int i = 0; i < kNumPoints; ++i) {
    const double theta = 2.0 * kPi * i / kNumPoints;
    const double r = (i == 30) ? 5.0 : 10.0;
    scan.points.push_back({static_cast<float>(r * std::cos(theta)),
                            static_cast<float>(r * std::sin(theta)), 0.0f, 0.0f});
  }
  return scan;
}

TEST(LidarFrontend, MovesFeaturesIntoTheCameraFrame) {
  Eigen::Isometry3d lidar_to_camera = Eigen::Isometry3d::Identity();
  lidar_to_camera.linear() =
      Eigen::AngleAxisd(1.2, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
  lidar_to_camera.translation() = Eigen::Vector3d(0.1, -0.3, 0.7);

  const LidarScan scan = MakeSingleRingWithCorner();
  LidarFrontend in_lidar_frame;
  LidarFrontend in_camera_frame({}, {}, lidar_to_camera);
  in_lidar_frame.ProcessScan(scan);
  in_camera_frame.ProcessScan(scan);

  const ScanFeatures& expected = in_lidar_frame.LastFeatures();
  const ScanFeatures& actual = in_camera_frame.LastFeatures();
  ASSERT_FALSE(expected.edge_points.empty());
  ASSERT_FALSE(expected.planar_points.empty());
  ASSERT_EQ(actual.edge_points.size(), expected.edge_points.size());
  ASSERT_EQ(actual.planar_points.size(), expected.planar_points.size());
  for (std::size_t i = 0; i < expected.edge_points.size(); ++i) {
    EXPECT_LT((actual.edge_points[i] - lidar_to_camera * expected.edge_points[i]).norm(), 1e-12);
  }
  for (std::size_t i = 0; i < expected.planar_points.size(); ++i) {
    EXPECT_LT((actual.planar_points[i] - lidar_to_camera * expected.planar_points[i]).norm(), 1e-12);
  }
}

}  // namespace
}  // namespace slam::frontend_lidar
