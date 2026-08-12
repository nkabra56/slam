#pragma once

#include <filesystem>
#include <vector>

#include <Eigen/Core>

#include "slam/mapping/point_cloud_map.hpp"

namespace slam::mapping {

// Owns the two world-frame point-cloud maps from ROADMAP.md's Phase 4: a
// sparse landmark map fed by VIO's triangulated landmarks
// (VioFrontend::LastLandmarks), and a LiDAR map fed by LidarFrontend's
// edge/planar feature points (LidarFrontend::LastFeatures) -- not the full
// raw scan, which would be far too much data to hold across a whole
// sequence between keyframes and the final map build; a full-density map
// fed from raw scans is a reasonable future enhancement, not attempted
// here. Both maps are the same PointCloudMap structure at different voxel
// granularity: the "sparse" vs "denser" distinction in the roadmap is
// about the point *source*, not two different implementations.
class Map {
 public:
  struct Params {
    double landmark_voxel_size_m = 0.2;
    double lidar_voxel_size_m = 0.5;
  };

  explicit Map(Params params = {});

  // Both must already be in world-frame coordinates -- callers transform
  // by the relevant keyframe's optimized pose (see slam_mapping_demo)
  // before inserting.
  void InsertLandmarks(const std::vector<Eigen::Vector3d>& points_world);
  void InsertLidarPoints(const std::vector<Eigen::Vector3d>& points_world);

  const PointCloudMap& LandmarkMap() const { return landmark_map_; }
  const PointCloudMap& LidarMap() const { return lidar_map_; }

  // Writes both maps to one ASCII PLY file (vertex-only, no color/normal
  // properties; landmark and LiDAR points are not distinguished in the
  // output). Returns false if the file couldn't be opened for writing.
  bool ExportPly(const std::filesystem::path& path) const;

 private:
  PointCloudMap landmark_map_;
  PointCloudMap lidar_map_;
};

}  // namespace slam::mapping
