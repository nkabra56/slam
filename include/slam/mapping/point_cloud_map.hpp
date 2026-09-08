#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace slam::mapping {

// Incremental voxel-grid accumulator: each point merges (running centroid)
// into its voxel, bounding memory across many Insert() calls.
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
