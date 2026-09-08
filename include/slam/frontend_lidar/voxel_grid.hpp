#pragma once

#include <vector>

#include <Eigen/Core>

namespace slam::frontend_lidar {

// Averages points in the same cubic voxel down to a single centroid.
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d>& points,
                                              double voxel_size);

}  // namespace slam::frontend_lidar
