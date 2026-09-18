#include "slam/frontend_lidar/scan_features.hpp"

#include <algorithm>
#include <cmath>

namespace slam::frontend_lidar {

namespace {

constexpr double kPi = 3.14159265358979323846;

int RingIndex(const Eigen::Vector3d& p, const FeatureExtractionParams& params) {
  const double range_xy = std::sqrt(p.x() * p.x() + p.y() * p.y());
  const double vertical_deg = std::atan2(p.z(), range_xy) * 180.0 / kPi;
  const double span = params.max_vertical_deg - params.min_vertical_deg;
  const int ring =
      static_cast<int>((vertical_deg - params.min_vertical_deg) / span * params.num_rings);
  return std::clamp(ring, 0, params.num_rings - 1);
}

}  // namespace

ScanFeatures ExtractFeatures(const LidarScan& scan, const FeatureExtractionParams& params) {
  ScanFeatures features;

  std::vector<std::vector<Eigen::Vector3d>> rings(static_cast<std::size_t>(params.num_rings));
  for (const auto& lp : scan.points) {
    const Eigen::Vector3d p(lp.x, lp.y, lp.z);
    if (p.norm() < params.min_range_m) continue;
    rings[static_cast<std::size_t>(RingIndex(p, params))].push_back(p);
  }

  const int w = params.curvature_window;

  for (auto& ring : rings) {
    const int n = static_cast<int>(ring.size());
    if (n < 2 * w + 1) continue;

    std::sort(ring.begin(), ring.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
      return std::atan2(a.y(), a.x()) < std::atan2(b.y(), b.x());
    });

    std::vector<double> curvature(static_cast<std::size_t>(n), -1.0);
    for (int i = 0; i < n; ++i) {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      for (int j = -w; j <= w; ++j) {
        if (j == 0) continue;
        // Wrap: the ring is a closed loop, not a line (atan2 sort seam).
        const int neighbor = ((i + j) % n + n) % n;
        sum += ring[static_cast<std::size_t>(i)] - ring[static_cast<std::size_t>(neighbor)];
      }
      const double range = ring[static_cast<std::size_t>(i)].norm();
      curvature[static_cast<std::size_t>(i)] = range > 1e-6 ? sum.norm() / (2 * w * range) : 0.0;
    }

    for (int region = 0; region < params.num_subregions; ++region) {
      const int region_start = n * region / params.num_subregions;
      const int region_end = n * (region + 1) / params.num_subregions;
      if (region_start >= region_end) continue;

      std::vector<int> indices;
      indices.reserve(static_cast<std::size_t>(region_end - region_start));
      for (int i = region_start; i < region_end; ++i) indices.push_back(i);

      std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return curvature[static_cast<std::size_t>(a)] > curvature[static_cast<std::size_t>(b)];
      });

      const int num_edge =
          std::min<int>(params.edge_points_per_subregion, static_cast<int>(indices.size()));
      for (int i = 0; i < num_edge; ++i) {
        features.edge_points.push_back(ring[static_cast<std::size_t>(indices[static_cast<std::size_t>(i)])]);
      }

      const int num_planar =
          std::min<int>(params.planar_points_per_subregion, static_cast<int>(indices.size()));
      for (int i = 0; i < num_planar; ++i) {
        const std::size_t idx = indices[indices.size() - 1 - static_cast<std::size_t>(i)];
        features.planar_points.push_back(ring[idx]);
      }
    }
  }

  return features;
}

}  // namespace slam::frontend_lidar
