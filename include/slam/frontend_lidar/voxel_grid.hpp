#pragma once

#include <vector>

#include <Eigen/Core>

namespace slam::frontend_lidar {

// Averages points that fall in the same cubic voxel of size `voxel_size`
// down to a single centroid -- a hash map keyed by voxel index, no PCL.
// Used to cap the number of edge/planar feature points fed into scan
// matching each frame.
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d>& points,
                                              double voxel_size);

}  // namespace slam::frontend_lidar
