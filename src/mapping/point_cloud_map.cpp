#include "slam/mapping/point_cloud_map.hpp"

#include <cmath>

namespace slam::mapping {

std::size_t PointCloudMap::VoxelKeyHash::operator()(const VoxelKey& key) const {
  std::size_t h = std::hash<int>{}(key.x);
  h ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
  h ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
  return h;
}

PointCloudMap::PointCloudMap(double voxel_size) : voxel_size_(voxel_size) {}

void PointCloudMap::Insert(const std::vector<Eigen::Vector3d>& points_world) {
  for (const auto& p : points_world) {
    const VoxelKey key{static_cast<int>(std::floor(p.x() / voxel_size_)),
                        static_cast<int>(std::floor(p.y() / voxel_size_)),
                        static_cast<int>(std::floor(p.z() / voxel_size_))};
    auto& acc = voxels_[key];
    acc.sum += p;
    ++acc.count;
  }
}

std::vector<Eigen::Vector3d> PointCloudMap::Points() const {
  std::vector<Eigen::Vector3d> result;
  result.reserve(voxels_.size());
  for (const auto& [key, acc] : voxels_) {
    result.push_back(acc.sum / acc.count);
  }
  return result;
}

std::size_t PointCloudMap::NumPoints() const { return voxels_.size(); }

}  // namespace slam::mapping
