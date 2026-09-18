#include "slam/backend/nav_state_graph.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace slam::backend {
namespace {

TEST(NavStateGraph, PoseEdgeOnlyConstrainsPoseSubBlock) {
  NavStateGraph graph;
  NavState state0;
  state0.velocity = Eigen::Vector3d(0.5, 0.1, -0.2);
  const NavNodeId n0 = graph.AddNode(state0);

  NavState state1;  // deliberately wrong initial guess
  state1.pose = Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(10.0, 10.0, 10.0));
  state1.velocity = Eigen::Vector3d(1.0, 2.0, 3.0);
  state1.bias_gyro = Eigen::Vector3d(0.01, 0.02, 0.03);
  const NavNodeId n1 = graph.AddNode(state1);

  const Sophus::SE3d z(Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitY())),
                        Eigen::Vector3d(1.0, 0.0, 0.5));
  NavPoseEdge edge;
  edge.from = n0;
  edge.to = n1;
  edge.measurement = z;
  graph.AddPoseEdge(edge);

  graph.FixNode(n0);
  graph.Solve();

  const Sophus::SE3d expected_pose = state0.pose * z.inverse();
  const Sophus::SE3d pose_error = expected_pose.inverse() * graph.State(n1).pose;
  EXPECT_LT(pose_error.log().norm(), 1e-6);

  // A pose-only edge shouldn't touch velocity/bias -- with no other
  // constraint on them, they should stay at their seeded values.
  EXPECT_TRUE(graph.State(n1).velocity.isApprox(state1.velocity, 1e-6));
  EXPECT_TRUE(graph.State(n1).bias_gyro.isApprox(state1.bias_gyro, 1e-6));
}

struct SyntheticImuSegment {
  ImuPreintegration preintegration;
  NavState state_i;
  NavState state_j;
  Eigen::Vector3d gravity;
};

SyntheticImuSegment MakeSyntheticImuSegment() {
  SyntheticImuSegment segment;
  segment.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  ImuMeasurement m1;
  m1.timestamp = 0.0;
  m1.linear_acceleration = Eigen::Vector3d(0.8, -0.3, 0.1);
  m1.angular_velocity = Eigen::Vector3d(0.0, 0.0, 0.2);
  segment.preintegration.Integrate(m1);
  ImuMeasurement m2;
  m2.timestamp = 0.1;
  segment.preintegration.Integrate(m2);

  segment.state_i.pose =
      Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0));
  segment.state_i.velocity = Eigen::Vector3d(0.3, 0.0, 0.0);

  const double dt = segment.preintegration.DeltaTime();
  const Eigen::Matrix3d R_i = segment.state_i.pose.rotationMatrix();

  segment.state_j.pose = Sophus::SE3d(
      segment.state_i.pose.so3() * segment.preintegration.DeltaRotation(),
      segment.state_i.pose.translation() + segment.state_i.velocity * dt +
          0.5 * segment.gravity * dt * dt + R_i * segment.preintegration.DeltaPosition());
  segment.state_j.velocity =
      segment.state_i.velocity + segment.gravity * dt + R_i * segment.preintegration.DeltaVelocity();

  return segment;
}

TEST(NavStateGraph, ImuEdgeAloneConvergesToConsistentState) {
  const SyntheticImuSegment segment = MakeSyntheticImuSegment();

  NavStateGraph graph;
  graph.SetGravity(segment.gravity);
  const NavNodeId n0 = graph.AddNode(segment.state_i);

  NavState wrong_guess;  // deliberately wrong
  wrong_guess.pose = Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 5.0, 5.0));
  wrong_guess.velocity = Eigen::Vector3d(-2.0, 3.0, 1.0);
  const NavNodeId n1 = graph.AddNode(wrong_guess);

  NavImuEdge edge;
  edge.from = n0;
  edge.to = n1;
  edge.preintegration = segment.preintegration;
  graph.AddImuEdge(edge);

  graph.FixNode(n0);
  graph.Solve();

  const Sophus::SE3d pose_error = segment.state_j.pose.inverse() * graph.State(n1).pose;
  EXPECT_LT(pose_error.log().norm(), 1e-4);
  EXPECT_LT((graph.State(n1).velocity - segment.state_j.velocity).norm(), 1e-4);
}

TEST(NavStateGraph, HighWeightPoseEdgeDominatesOverImuEdge) {
  const SyntheticImuSegment segment = MakeSyntheticImuSegment();

  NavStateGraph graph;
  graph.SetGravity(segment.gravity);
  const NavNodeId n0 = graph.AddNode(segment.state_i);
  const NavNodeId n1 = graph.AddNode(segment.state_i);  // seed doesn't matter much here

  NavImuEdge imu_edge;
  imu_edge.from = n0;
  imu_edge.to = n1;
  imu_edge.preintegration = segment.preintegration;
  imu_edge.motion_information = Eigen::Matrix<double, 9, 9>::Identity();  // low weight
  graph.AddImuEdge(imu_edge);

  // A deliberately different, much more confident pose measurement -- as
  // if LiDAR/VIO strongly disagreed with the IMU-implied motion.
  const Sophus::SE3d accurate_measurement(
      segment.state_j.pose.so3(),
      segment.state_j.pose.translation() + Eigen::Vector3d(0.5, 0.0, 0.0));
  NavPoseEdge pose_edge;
  pose_edge.from = n0;
  pose_edge.to = n1;
  pose_edge.measurement = accurate_measurement;
  pose_edge.information = 1e6 * Eigen::Matrix<double, 6, 6>::Identity();  // very high weight
  graph.AddPoseEdge(pose_edge);

  graph.FixNode(n0);
  graph.Solve();

  const Sophus::SE3d expected_pose = segment.state_i.pose * accurate_measurement.inverse();
  const double dist_to_pose_edge_target =
      (graph.State(n1).pose.translation() - expected_pose.translation()).norm();
  const double dist_to_imu_only_target =
      (graph.State(n1).pose.translation() - segment.state_j.pose.translation()).norm();

  EXPECT_LT(dist_to_pose_edge_target, dist_to_imu_only_target);
  EXPECT_LT(dist_to_pose_edge_target, 0.05);
}

TEST(NavStateGraph, SolveWithNoEdgesIsANoOp) {
  NavStateGraph graph;
  NavState state;
  state.velocity = Eigen::Vector3d(1, 2, 3);
  const NavNodeId n0 = graph.AddNode(state);
  EXPECT_EQ(graph.Solve(), 0);
  EXPECT_TRUE(graph.State(n0).velocity.isApprox(state.velocity));
}

}  // namespace
}  // namespace slam::backend
