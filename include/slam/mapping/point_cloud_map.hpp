#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace slam::mapping {

// Incremental voxel-grid point-cloud accumulator: each inserted point
// merges (running centroid) into its voxel of size `voxel_size`, bounding
// memory regardless of how many points or Insert() calls accumulate over a
// full sequence. Same hash-grid technique as
// frontend_lidar::VoxelDownsample, but stateful across many calls rather
// than a one-shot pass over a single point set -- a small enough piece
// that duplicating it here was simpler than adding a cross-module
// dependency between mapping/ and frontend_lidar/ for it.
class PointCloudMap {
 public:
  explicit PointCloudMap(double voxel_size);

  // `points_world` must already be in the map's world frame.
  void Insert(const std::vector<Eigen::Vector3d>& points_world);

  std::vector<Eigen::Vector3d> Points() const;
  std::size_t NumPoints() const;

 private:
  struct VoxelKey {
    int x;
    int y;
    int z;
    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const;
  };
  struct VoxelAccumulator {
    Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
    int count{0};
  };

  double voxel_size_;
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels_;
};

}  // namespace slam::mapping
