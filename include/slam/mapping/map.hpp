#pragma once

namespace slam::mapping {

// Sparse landmark map (from slam::frontend_vio) and voxel/point-cloud map
// (from slam::frontend_lidar), rendered by the slam::viz Pangolin viewer.
//
// TODO(phase-4): map storage + Pangolin live viewer.
class Map {
 public:
  void Clear();
};

}  // namespace slam::mapping
