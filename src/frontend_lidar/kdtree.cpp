#include "slam/frontend_lidar/kdtree.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace slam::frontend_lidar {

namespace {

bool ByDistance(const std::pair<double, std::size_t>& a, const std::pair<double, std::size_t>& b) {
  return a.first < b.first;
}

}  // namespace

KdTree3d::KdTree3d(std::vector<Eigen::Vector3d> points) : points_(std::move(points)) {
  if (points_.empty()) return;
  std::vector<std::size_t> indices(points_.size());
  std::iota(indices.begin(), indices.end(), 0);
  nodes_.reserve(points_.size());
  root_ = Build(indices, 0, indices.size(), 0);
}

int KdTree3d::Build(std::vector<std::size_t>& indices, std::size_t begin, std::size_t end,
                     int depth) {
  if (begin >= end) return -1;

  const int axis = depth % 3;
  const std::size_t mid = begin + (end - begin) / 2;
  std::nth_element(indices.begin() + static_cast<long>(begin), indices.begin() + static_cast<long>(mid),
                    indices.begin() + static_cast<long>(end), [this, axis](std::size_t a, std::size_t b) {
                      return points_[a](axis) < points_[b](axis);
                    });

  Node node;
  node.point_index = indices[mid];
  node.axis = axis;
  const int node_index = static_cast<int>(nodes_.size());
  nodes_.push_back(node);

  const int left = Build(indices, begin, mid, depth + 1);
  const int right = Build(indices, mid + 1, end, depth + 1);
  nodes_[static_cast<std::size_t>(node_index)].left = left;
  nodes_[static_cast<std::size_t>(node_index)].right = right;
  return node_index;
}

std::vector<std::size_t> KdTree3d::KNearest(const Eigen::Vector3d& query, int k) const {
  std::vector<std::pair<double, std::size_t>> best;
  best.reserve(static_cast<std::size_t>(k));
  if (root_ >= 0) {
    Query(root_, query, k, best);
  }
  std::sort(best.begin(), best.end(), ByDistance);

  std::vector<std::size_t> result;
  result.reserve(best.size());
  for (const auto& entry : best) result.push_back(entry.second);
  return result;
}

void KdTree3d::Query(int node_index, const Eigen::Vector3d& query, int k,
                      std::vector<std::pair<double, std::size_t>>& best) const {
  if (node_index < 0) return;
  const Node& node = nodes_[static_cast<std::size_t>(node_index)];
  const Eigen::Vector3d& point = points_[node.point_index];
  const double dist2 = (point - query).squaredNorm();

  if (static_cast<int>(best.size()) < k) {
    best.emplace_back(dist2, node.point_index);
    if (static_cast<int>(best.size()) == k) {
      std::make_heap(best.begin(), best.end(), ByDistance);
    }
  } else if (dist2 < best.front().first) {
    std::pop_heap(best.begin(), best.end(), ByDistance);
    best.back() = {dist2, node.point_index};
    std::push_heap(best.begin(), best.end(), ByDistance);
  }

  const double diff = query(node.axis) - point(node.axis);
  const int near_child = diff < 0 ? node.left : node.right;
  const int far_child = diff < 0 ? node.right : node.left;

  Query(near_child, query, k, best);

  const double worst = static_cast<int>(best.size()) < k ? std::numeric_limits<double>::infinity()
                                                           : best.front().first;
  if (diff * diff < worst) {
    Query(far_child, query, k, best);
  }
}

}  // namespace slam::frontend_lidar
