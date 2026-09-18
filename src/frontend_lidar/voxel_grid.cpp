#include "slam/frontend_lidar/voxel_grid.hpp"

#include <cmath>
#include <unordered_map>

namespace slam::frontend_lidar {

namespace {

struct VoxelKey {
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    std::size_t h = std::hash<int>{}(key.x);
    h ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

struct VoxelAccumulator {
  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
  int count{0};
};

}  // namespace

std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d>& points,
                                              double voxel_size) {
  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
  for (const auto& p : points) {
    const VoxelKey key{static_cast<int>(std::floor(p.x() / voxel_size)),
                        static_cast<int>(std::floor(p.y() / voxel_size)),
                        static_cast<int>(std::floor(p.z() / voxel_size))};
    auto& acc = voxels[key];
    acc.sum += p;
    ++acc.count;
  }

  std::vector<Eigen::Vector3d> result;
  result.reserve(voxels.size());
  for (const auto& [key, acc] : voxels) {
    result.push_back(acc.sum / acc.count);
  }
  return result;
}

}  // namespace slam::frontend_lidar
