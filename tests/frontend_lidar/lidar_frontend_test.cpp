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

// A flat wall 10m ahead of a level, x-forward sensor at `sensor_x`, seen by 24 rings
// over +-30 degrees of azimuth. A wall fully constrains forward motion.
LidarScan MakeWallScan(double sensor_x) {
  constexpr double kWallX = 10.0;
  LidarScan scan;
  for (int ring = 40; ring < 64; ++ring) {
    const double elevation = (-24.8 + (ring + 0.5) * 26.8 / 64) * kPi / 180.0;
    for (double azimuth_deg = -30.0; azimuth_deg <= 30.0; azimuth_deg += 0.25) {
      const double azimuth = azimuth_deg * kPi / 180.0;
      const Eigen::Vector3d d(std::cos(elevation) * std::cos(azimuth),
                              std::cos(elevation) * std::sin(azimuth), std::sin(elevation));
      const Eigen::Vector3d p = ((kWallX - sensor_x) / d.x()) * d;
      scan.points.push_back(
          {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()), 0.0f});
    }
  }
  return scan;
}

// A highway-speed first step (1.5m) exceeds the 1m correspondence radius, and
// there is no motion prior yet, so the frontend must search wider on this match.
TEST(LidarFrontend, RecoversALargeFirstMotionWithoutAPrior) {
  LidarFrontend frontend;
  frontend.ProcessScan(MakeWallScan(0.0));
  const auto result = frontend.ProcessScan(MakeWallScan(1.5));

  ASSERT_TRUE(result.has_pose);
  // p_curr = relative_pose * p_prev, so a sensor that moved +1.5m in x reports -1.5m.
  EXPECT_NEAR(result.relative_pose.translation().x(), -1.5, 0.05);
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
