#pragma once

#include <vector>

#include <Eigen/Core>

#include "slam/common/types.hpp"

namespace slam::frontend_lidar {

struct ScanFeatures {
  std::vector<Eigen::Vector3d> edge_points;
  std::vector<Eigen::Vector3d> planar_points;
};

struct FeatureExtractionParams {
  int num_rings = 64;
  double min_vertical_deg = -24.8;
  double max_vertical_deg = 2.0;
  int curvature_window = 5;  // neighbors considered on each side, along the ring
  int num_subregions = 6;    // per ring, to spread feature picks around it
  int edge_points_per_subregion = 2;
  int planar_points_per_subregion = 4;
  double min_range_m = 1.0;  // drop points too close to the sensor
};

// Extracts LOAM-style edge (high local curvature) and planar (low local
// curvature) feature points from a raw Velodyne scan.
//
// Ring indices aren't present in KITTI's .bin files, so they're estimated
// from each point's vertical angle -- a documented approximation, not the
// true per-beam calibration. Occlusion/near-parallel-beam filtering (a
// further LOAM refinement) is not implemented; the curvature-ranking
// selection alone is the Phase 2 scope.
ScanFeatures ExtractFeatures(const LidarScan& scan, const FeatureExtractionParams& params = {});

}  // namespace slam::frontend_lidar
