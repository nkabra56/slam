#pragma once

namespace slam::backend {

// Hand-written sliding-window bundle adjustment (Gauss-Newton /
// Levenberg-Marquardt) and pose-graph optimization with loop closure,
// consuming estimates from both slam::frontend_vio and slam::frontend_lidar.
//
// No third-party optimization library (GTSAM/g2o/Ceres) is used here by
// design; correctness issues get fixed with better math, not a swapped-in
// dependency. See ROADMAP.md.
//
// TODO(phase-3): sliding-window BA, shared pose graph, loop closure.
class SlidingWindowOptimizer {
 public:
  void Optimize();
};

}  // namespace slam::backend
