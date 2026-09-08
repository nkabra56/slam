#pragma once

#include <filesystem>
#include <vector>

#include <Eigen/Core>

#include "slam/mapping/point_cloud_map.hpp"

namespace slam::mapping {

// Owns a sparse VIO landmark map and a LiDAR feature-point map, both the
// same PointCloudMap at different voxel granularity.
class Map {
 public:
  struct Params {
    double landmark_voxel_size_m = 0.2;
    double lidar_voxel_size_m = 0.5;
  };

  Map() : Map(Params{}) {}
  explicit Map(Params params);

  // Points must already be in world-frame coordinates.
  void InsertLandmarks(const std::vector<Eigen::Vector3d>& points_world);
  void InsertLidarPoints(const std::vector<Eigen::Vector3d>& points_world);

  const PointCloudMap& LandmarkMap() const { return landmark_map_; }
  const PointCloudMap& LidarMap() const { return lidar_map_; }

  // Writes both maps to one ASCII PLY file (vertex-only, undistinguished).
  bool ExportPly(const std::filesystem::path& path) const;

 private:
  PointCloudMap landmark_map_;
  PointCloudMap lidar_map_;
};

}  // namespace slam::mapping
