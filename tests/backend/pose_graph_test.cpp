#include "slam/backend/pose_graph.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::backend {
namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(PoseGraph, TwoNodeEdgeConvergesToMeasurement) {
  PoseGraph graph;
  const NodeId n0 = graph.AddNode(Sophus::SE3d());
  // Deliberately wrong initial guess for n1 -- Solve() has to fix it.
  const NodeId n1 =
      graph.AddNode(Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(10.0, 10.0, 10.0)));

  const Sophus::SE3d z(Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())),
                        Eigen::Vector3d(1.0, 0.5, 0.0));

  PoseGraphEdge edge;
  edge.from = n0;
  edge.to = n1;
  edge.measurement = z;
  graph.AddEdge(edge);

  graph.FixNode(n0);
  graph.Solve();

  // n1 should converge to identity * z^-1, per PoseGraphEdge's convention.
  const Sophus::SE3d expected = Sophus::SE3d() * z.inverse();
  const Sophus::SE3d error = expected.inverse() * graph.Pose(n1);
  EXPECT_LT(error.log().norm(), 1e-6);
}

// Hexagon with vertex 0 pinned to the origin, matching the solver's gauge-fix.
std::vector<Sophus::SE3d> MakeHexagonGroundTruth(double radius) {
  std::vector<Eigen::Vector3d> positions;
  for (int k = 0; k < 6; ++k) {
    const double angle = kPi / 3.0 * k;
    positions.emplace_back(radius * std::cos(angle), radius * std::sin(angle), 0.0);
  }
  const Eigen::Vector3d origin_shift = positions[0];

  std::vector<Sophus::SE3d> poses;
  for (const auto& p : positions) {
    poses.push_back(Sophus::SE3d(Eigen::Quaterniond::Identity(), p - origin_shift));
  }
  return poses;
}

TEST(PoseGraph, LoopClosureCorrectsAccumulatedDrift) {
  const auto ground_truth = MakeHexagonGroundTruth(2.0);

  // Consecutive "odometry" edges biased by a constant translation offset,
  // simulating systematic dead-reckoning drift.
  const Eigen::Vector3d bias(0.05, 0.0, 0.0);
  std::vector<Sophus::SE3d> measured_edges;  // measured_edges[k]: node k -> node k+1
  for (int k = 0; k < 5; ++k) {
    const Sophus::SE3d z_true = ground_truth[k + 1].inverse() * ground_truth[k];
    measured_edges.push_back(
        Sophus::SE3d(z_true.unit_quaternion(), z_true.translation() + bias));
  }
  // Accurate loop-closure edge, node 5 -> node 0 (as if geometric
  // verification found the true relative pose).
  const Sophus::SE3d loop_measurement = ground_truth[0].inverse() * ground_truth[5];

  PoseGraph graph;
  std::vector<NodeId> nodes;
  Sophus::SE3d seed;
  nodes.push_back(graph.AddNode(seed));
  for (int k = 0; k < 5; ++k) {
    seed = seed * measured_edges[static_cast<std::size_t>(k)].inverse();
    nodes.push_back(graph.AddNode(seed));
  }

  const double error_before =
      (graph.Pose(nodes[3]).translation() - ground_truth[3].translation()).norm();
  ASSERT_GT(error_before, 0.05);  // sanity check the drift is actually present

  const Eigen::Matrix<double, 6, 6> odometry_information = Eigen::Matrix<double, 6, 6>::Identity();
  const Eigen::Matrix<double, 6, 6> loop_information =
      50.0 * Eigen::Matrix<double, 6, 6>::Identity();

  for (int k = 0; k < 5; ++k) {
    PoseGraphEdge edge;
    edge.from = nodes[static_cast<std::size_t>(k)];
    edge.to = nodes[static_cast<std::size_t>(k + 1)];
    edge.measurement = measured_edges[static_cast<std::size_t>(k)];
    edge.information = odometry_information;
    graph.AddEdge(edge);
  }

  PoseGraphEdge loop_edge;
  loop_edge.from = nodes[5];
  loop_edge.to = nodes[0];
  loop_edge.measurement = loop_measurement;
  loop_edge.information = loop_information;
  graph.AddEdge(loop_edge);

  graph.FixNode(nodes[0]);
  graph.Solve();

  const double error_after =
      (graph.Pose(nodes[3]).translation() - ground_truth[3].translation()).norm();

  EXPECT_LT(error_after, 0.5 * error_before);
}

TEST(PoseGraph, SolveWithNoEdgesIsANoOp) {
  PoseGraph graph;
  const NodeId n0 = graph.AddNode(Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(3, 4, 5)));
  EXPECT_EQ(graph.Solve(), 0);
  EXPECT_TRUE(graph.Pose(n0).translation().isApprox(Eigen::Vector3d(3, 4, 5)));
}

}  // namespace
}  // namespace slam::backend
