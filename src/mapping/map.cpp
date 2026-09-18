#include "slam/mapping/map.hpp"

#include <fstream>

namespace slam::mapping {

Map::Map(Params params)
    : landmark_map_(params.landmark_voxel_size_m), lidar_map_(params.lidar_voxel_size_m) {}

void Map::InsertLandmarks(const std::vector<Eigen::Vector3d>& points_world) {
  landmark_map_.Insert(points_world);
}

void Map::InsertLidarPoints(const std::vector<Eigen::Vector3d>& points_world) {
  lidar_map_.Insert(points_world);
}

bool Map::ExportPly(const std::filesystem::path& path) const {
  std::ofstream file(path);
  if (!file.is_open()) {
    return false;
  }

  const std::vector<Eigen::Vector3d> landmark_points = landmark_map_.Points();
  const std::vector<Eigen::Vector3d> lidar_points = lidar_map_.Points();
  const std::size_t total = landmark_points.size() + lidar_points.size();

  file << "ply\n";
  file << "format ascii 1.0\n";
  file << "element vertex " << total << "\n";
  file << "property float x\n";
  file << "property float y\n";
  file << "property float z\n";
  file << "end_header\n";

  for (const auto& p : landmark_points) {
    file << p.x() << " " << p.y() << " " << p.z() << "\n";
  }
  for (const auto& p : lidar_points) {
    file << p.x() << " " << p.y() << " " << p.z() << "\n";
  }

  return true;
}

}  // namespace slam::mapping
