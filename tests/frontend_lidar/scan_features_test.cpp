#include "slam/frontend_lidar/scan_features.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace slam::frontend_lidar {
namespace {

constexpr double kPi = 3.14159265358979323846;

// All points at z=0 share the same vertical angle (0 deg), so they all fall
// into a single estimated ring regardless of the exact ring-bucket math --
// avoids needing to hand-compute which of num_rings the points land in.
LidarScan MakeSingleRingWithCorner(int n, double radius, int corner_index, double corner_radius) {
  LidarScan scan;
  for (int i = 0; i < n; ++i) {
    const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
    const double r = (i == corner_index) ? corner_radius : radius;
    LidarPoint p;
    p.x = static_cast<float>(r * std::cos(theta));
    p.y = static_cast<float>(r * std::sin(theta));
    p.z = 0.0f;
    p.intensity = 0.0f;
    scan.points.push_back(p);
  }
  return scan;
}

TEST(ExtractFeatures, ProducesBoundedFeatureCounts) {
  const LidarScan scan = MakeSingleRingWithCorner(60, 10.0, 30, 5.0);
  const FeatureExtractionParams params;

  const ScanFeatures features = ExtractFeatures(scan, params);

  EXPECT_GT(features.edge_points.size(), 0u);
  EXPECT_GT(features.planar_points.size(), 0u);
  EXPECT_LE(features.edge_points.size(),
            static_cast<std::size_t>(params.num_subregions * params.edge_points_per_subregion));
  EXPECT_LE(features.planar_points.size(),
            static_cast<std::size_t>(params.num_subregions * params.planar_points_per_subregion));
}

TEST(ExtractFeatures, FlagsSharpCornerAsEdgePoint) {
  const int n = 60;
  const int corner_index = 30;
  const double corner_radius = 5.0;
  const LidarScan scan = MakeSingleRingWithCorner(n, 10.0, corner_index, corner_radius);

  const ScanFeatures features = ExtractFeatures(scan);

  const double theta = 2.0 * kPi * corner_index / n;
  const Eigen::Vector3d expected(corner_radius * std::cos(theta), corner_radius * std::sin(theta),
                                  0.0);

  bool found = false;
  for (const auto& p : features.edge_points) {
    if ((p - expected).norm() < 1e-3) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

TEST(ExtractFeatures, IgnoresRingsWithTooFewPoints) {
  LidarScan scan;
  LidarPoint p{1.0f, 0.0f, 0.0f, 0.0f};
  scan.points.push_back(p);  // one point, one ring: far below curvature_window

  const ScanFeatures features = ExtractFeatures(scan);
  EXPECT_TRUE(features.edge_points.empty());
  EXPECT_TRUE(features.planar_points.empty());
}

}  // namespace
}  // namespace slam::frontend_lidar
