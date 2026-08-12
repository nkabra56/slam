#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace slam::frontend_lidar {

// Simple static 3D k-d tree over a fixed point set, built once and queried
// many times -- exactly the access pattern scan matching needs each
// Gauss-Newton iteration. No PCL/nanoflann; median-split build, standard
// branch-and-bound nearest-neighbor query.
class KdTree3d {
 public:
  explicit KdTree3d(std::vector<Eigen::Vector3d> points);

  // Returns up to k nearest-neighbor indices into the original points
  // vector (as passed to the constructor), sorted nearest-first. May
  // return fewer than k if the tree has fewer points.
  std::vector<std::size_t> KNearest(const Eigen::Vector3d& query, int k) const;

  const std::vector<Eigen::Vector3d>& points() const { return points_; }

 private:
  struct Node {
    std::size_t point_index{};
    int axis{};
    int left{-1};
    int right{-1};
  };

  int Build(std::vector<std::size_t>& indices, std::size_t begin, std::size_t end, int depth);
  void Query(int node_index, const Eigen::Vector3d& query, int k,
             std::vector<std::pair<double, std::size_t>>& best) const;

  std::vector<Eigen::Vector3d> points_;
  std::vector<Node> nodes_;
  int root_{-1};
};

}  // namespace slam::frontend_lidar
