# Graph Report - .  (2026-08-13)

## Corpus Check
- Corpus is ~42,438 words - fits in a single context window. You may not need a graph.

## Summary
- 934 nodes · 1302 edges · 74 communities (57 shown, 17 thin omitted)
- Extraction: 91% EXTRACTED · 9% INFERRED · 0% AMBIGUOUS · INFERRED: 120 edges (avg confidence: 0.8)
- Token cost: 157,173 input · 0 output

## Community Hubs (Navigation)
- IMU Factor & VIO Initializer
- ROS2 SlamNode
- KITTI Dataset Reader
- ROS2 Message Adapters
- Pose Graph Optimizer
- Voxel Grid & Point Cloud Map
- Project Docs & Build Rationale
- VIO Feature Tracking Frontend
- KD-Tree Nearest Neighbor
- Trajectory Evaluation Metrics
- KITTI Raw IMU Reader
- Demo Apps & Common Types
- Ground Removal (LiDAR)
- NavStateGraph Fusion Tests
- Sliding Window Optimizer
- IMU Preintegration (VIO)
- NavState & Graph Implementation
- KITTI Raw IMU Implementation
- NavStateGraph Solve & Retract
- Point Cloud Map Export
- NavStateGraph Edge Types
- Pangolin Viewer Implementation
- Stereo Geometry
- LiDAR Frontend & Demo Apps
- LOAM Feature Extraction Params
- vcpkg Dependency Manifest
- ROS2 Node Threading Primitives
- graphify Skill Reference Docs
- Scan Features & Loop Closure Candidates
- Camera Calibration Types
- Frame Types
- LiDAR Point Types
- Optimizer Implementation Helpers
- LiDAR Scan Matcher
- CMake Build Targets
- IMU Measurement & Preintegration Core
- KITTI Raw Drive Mapping
- Scan Matcher Tests
- Map Implementation
- NavStateGraph Synthetic IMU Tests
- Loop Closure Detection Tests
- Pangolin Viewer Interface
- Optimizer Edge Weighting
- Scan Feature Extraction Tests
- Loop Closure Params
- Loop Closure Result
- Scan Match Result
- Scan Matcher Params
- VIO Track Triangulation
- Loop Closure Detection Implementation
- VIO Frontend Implementation Includes
- Feature Tracker Implementation
- Python Bindings (pyslam)
- Relative Pose Estimate
- LiDAR Frontend Implementation
- graphify Install Hook
- graphify MCP & God Nodes
- graphify Subagent Extraction
- graphify AST/Semantic Extraction
- LIO-SAM & Tier-2 Fusion
- VINS-Mono & VIO Initializer Design
- Phase 4 Mapping Notes
- Phase 2 LiDAR Design Notes
- graphify Watch Mode
- graphify Benchmark Export
- graphify FalkorDB Export
- graphify Node ID Format
- graphify Path & Explain
- graphify Update Modes
- graphify Community Detection
- Roadmap Phase 0
- Roadmap Phase 5 Evaluation

## God Nodes (most connected - your core abstractions)
1. `SlamNode` - 70 edges
2. `ImuPreintegration` - 25 edges
3. `NavStateGraph` - 22 edges
4. `SlidingWindowOptimizer` - 22 edges
5. `PoseGraph` - 16 edges
6. `VioFrontend` - 16 edges
7. `KittiSequenceReader` - 16 edges
8. `Map` - 15 edges
9. `TEST()` - 15 edges
10. `TEST()` - 15 edges

## Surprising Connections (you probably didn't know these)
- `main()` --calls--> `ShouldClose`  [INFERRED]
  apps/main_viz_demo.cpp → include/slam/viz/pangolin_viewer.hpp
- `main()` --calls--> `Update`  [INFERRED]
  apps/main_viz_demo.cpp → include/slam/viz/pangolin_viewer.hpp
- `KdTree3d::Query()` --calls--> `point`  [INFERRED]
  src/frontend_lidar/kdtree.cpp → include/slam/frontend_vio/feature_tracker.hpp
- `Honesty Rules` --semantically_similar_to--> `Never Build-Verified Caveat`  [INFERRED] [semantically similar]
  .claude/skills/graphify/SKILL.md → ROADMAP.md
- `Numeric-with-Reintegration Jacobian Recommendation` --semantically_similar_to--> `Confidence Score Rubric`  [INFERRED] [semantically similar]
  PHASE6_PLAN.md → .claude/skills/graphify/references/extraction-spec.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Phase 6 Tier-1 Tightly-Coupled Fusion Components** — phase6_plan_imu_factor_tier1, phase6_plan_nav_state, phase6_plan_vio_initializer, phase6_plan_nav_state_graph_separation [INFERRED 0.85]
- **Build Targets Linking slam_core** — cmakelists_slam_core_target, apps_cmakelists_demo_targets, tests_cmakelists_slam_tests, bindings_cmakelists_pyslam, src_viz_cmakelists_slam_viz [EXTRACTED 1.00]
- **graphify Extraction Pipeline Stages** — claude_skills_graphify_skill_ast_extraction, claude_skills_graphify_skill_semantic_extraction, claude_skills_graphify_skill_subagent_dispatch, claude_skills_graphify_references_extraction_spec_prompt [EXTRACTED 1.00]

## Communities (74 total, 17 thin omitted)

### Community 0 - "IMU Factor & VIO Initializer"
Cohesion: 0.05
Nodes (50): BiasCorrectedMatchesDirectIntegrationWithSameBias, BiasRandomWalkReflectsBiasDifference, SO3d, Vector3d, ImuBias, accel, gyro, ImuPreintegration (+42 more)

### Community 1 - "ROS2 SlamNode"
Cohesion: 0.04
Nodes (51): PendingLidarScan, CameraInfo, ConstSharedPtr, Node, Path, size_t, string, unique_ptr (+43 more)

### Community 2 - "KITTI Dataset Reader"
Cohesion: 0.07
Nodes (43): Isometry3d, TimestampSec, vector, KittiSequenceReader, ground_truth_poses_, GroundTruthPoseAt, HasGroundTruth, LoadCalibration (+35 more)

### Community 3 - "ROS2 Message Adapters"
Cohesion: 0.08
Nodes (42): ComputesBaselineFromRightProjectionMatrix, CopiesFieldsCorrectly, ExtractsFocalLengthAndPrincipalPoint, ForwardOpticalTranslationBecomesForwardBaseTranslation, HandlesMissingIntensityField, Imu, MapsForwardAndUpAxesCorrectly, ParsesXyzIntensityFields (+34 more)

### Community 4 - "Pose Graph Optimizer"
Cohesion: 0.06
Nodes (38): Matrix, NodeId, SE3d, vector, PoseGraph, AddEdge, AddNode, edges_ (+30 more)

### Community 5 - "Voxel Grid & Point Cloud Map"
Cohesion: 0.06
Nodes (35): AccumulatesAcrossMultipleInsertCalls, CentroidIsAverageOfMergedPoints, DistinctVoxelsStayDistinct, EmptyMapHasNoPoints, HandlesEmptyInput, unordered_map, PointCloudMap, Insert (+27 more)

### Community 6 - "Project Docs & Build Rationale"
Cohesion: 0.07
Nodes (36): Confidence Score Rubric, Honesty Rules, install()/EXPORT slamTargets, Docker Compose Services (slam, slam-ros2), Dockerfile (multi-stage runtime/ros2 build), Library-First Build Integration, Forster et al. 2017: On-Manifold Preintegration, Optical-to-REP-103 Frame Conversion (+28 more)

### Community 7 - "VIO Feature Tracking Frontend"
Cohesion: 0.06
Nodes (31): DetectsCornersOnFirstFrame, FeatureTracker, DetectNewCorners, next_id_, params_, prev_image_, tracks_, Mat (+23 more)

### Community 8 - "KD-Tree Nearest Neighbor"
Cohesion: 0.11
Nodes (24): FindsNearestNeighborExactMatch, HandlesEmptyTree, Node, vector, Vector3d, KdTree3d, Build, KNearest (+16 more)

### Community 9 - "Trajectory Evaluation Metrics"
Cohesion: 0.10
Nodes (23): DetectsScaleError, AbsoluteTrajectoryError, alignment, rmse_m, SE3d, size_t, KittiOdometryError, avg_rotation_error_deg_per_100m (+15 more)

### Community 10 - "KITTI Raw IMU Reader"
Cohesion: 0.09
Nodes (23): FindsKnownSequences, size_t, TimestampSec, vector, KittiOxtsReader, MeasurementAt, NumMeasurements, oxts_dir_ (+15 more)

### Community 11 - "Demo Apps & Common Types"
Cohesion: 0.11
Nodes (10): ImuMeasurementDefaultsToZero, Matrix, vector, ImuFactorResidual, bias_random_walk, motion, LidarScanHoldsPoints, optional (+2 more)

### Community 12 - "Ground Removal (LiDAR)"
Cohesion: 0.09
Nodes (22): GroundRemovalParams, distance_threshold_m, max_iterations, max_normal_tilt_deg, min_inlier_ratio, GroundRemovalResult, ground_points, non_ground_points (+14 more)

### Community 13 - "NavStateGraph Fusion Tests"
Cohesion: 0.12
Nodes (20): HighWeightPoseEdgeDominatesOverImuEdge, ImuEdgeAloneConvergesToConsistentState, vector, Vector3d, NavStateGraph, AddImuEdge, AddNode, AddPoseEdge (+12 more)

### Community 14 - "Sliding Window Optimizer"
Cohesion: 0.13
Nodes (17): main(), ChainsConsistentKeyframesToExpectedTrajectory, FreezesKeyframesOutsideWindowWithoutBreakingTheChain, ImuEdgeConstrainsRotationOnlyNotTranslation, Params, vector, SlidingWindowOptimizer, AddKeyframe (+9 more)

### Community 15 - "IMU Preintegration (VIO)"
Cohesion: 0.12
Nodes (17): SO3d, Vector3d, ImuPreintegrationResult, delta_position, delta_rotation, delta_time, delta_velocity, ImuPreintegrator (+9 more)

### Community 16 - "NavState & Graph Implementation"
Cohesion: 0.13
Nodes (15): SE3d, Vector3d, NavState, bias_accel, bias_gyro, pose, velocity, NavNodeId (+7 more)

### Community 17 - "KITTI Raw IMU Implementation"
Cohesion: 0.20
Nodes (16): optional, size_t, string, TimestampSec, DaysFromCivil(), KittiOxtsReader::KittiOxtsReader(), KittiOxtsReader::MeasurementAt(), KittiOxtsReader::NumMeasurements() (+8 more)

### Community 18 - "NavStateGraph Solve & Retract"
Cohesion: 0.13
Nodes (13): AppliesZeroDeltaAsIdentity, AccumulateFactor, MatrixXd, ResidualFn, Matrix, SolveParams, vector, NavStateGraph::AccumulateFactor() (+5 more)

### Community 19 - "Point Cloud Map Export"
Cohesion: 0.27
Nodes (12): main(), main(), ExportPlyFailsGracefullyForUnwritablePath, ExportPlyWritesValidHeaderAndVertexCount, Map, ExportPly, InsertLandmarks, InsertLidarPoints (+4 more)

### Community 20 - "NavStateGraph Edge Types"
Cohesion: 0.15
Nodes (14): Matrix, NavNodeId, SE3d, NavImuEdge, bias_information, from, motion_information, preintegration (+6 more)

### Community 21 - "Pangolin Viewer Implementation"
Cohesion: 0.16
Nodes (12): OpenGlRenderState, SE3d, string, vector, Vector3d, PangolinViewer::Impl, display, render_state (+4 more)

### Community 22 - "Stereo Geometry"
Cohesion: 0.18
Nodes (12): Point3d, RecoversKnownDepth, RecoversKnownRigidTransform, RejectsNonPositiveDisparity, optional, Point2f, vector, Vector3d (+4 more)

### Community 23 - "LiDAR Frontend & Demo Apps"
Cohesion: 0.19
Nodes (10): string, main(), PrintRow(), main(), LidarFrontend, feature_params_, has_previous_scan_, matcher_params_ (+2 more)

### Community 24 - "LOAM Feature Extraction Params"
Cohesion: 0.18
Nodes (12): FeatureExtractionParams, curvature_window, edge_points_per_subregion, max_vertical_deg, min_range_m, min_vertical_deg, num_rings, num_subregions (+4 more)

### Community 25 - "vcpkg Dependency Manifest"
Cohesion: 0.15
Nodes (12): eigen3, gtest, pangolin, pybind11, sophus, dependencies, features, viz (+4 more)

### Community 26 - "ROS2 Node Threading Primitives"
Cohesion: 0.20
Nodes (9): atomic, condition_variable, deque, mutex, Odometry, Image, Subscriber, Synchronizer (+1 more)

### Community 27 - "graphify Skill Reference Docs"
Cohesion: 0.18
Nodes (11): graphify Skill Pointer, /graphify add <url>, Neo4j Export, GitHub Clone & Cross-Repo Merge, Post-Commit Auto-Rebuild Hook, save-result Feedback Loop, Query Vocabulary Expansion + BFS/DFS Traversal, Video/Audio Transcription (Whisper) (+3 more)

### Community 28 - "Scan Features & Loop Closure Candidates"
Cohesion: 0.20
Nodes (10): Vector3d, LoopClosureCandidateSource, features, node_id, position, vector, Vector3d, ScanFeatures (+2 more)

### Community 29 - "Camera Calibration Types"
Cohesion: 0.25
Nodes (8): CameraIntrinsics, cx, cy, fx, fy, StereoCalibration, baseline_m, left

### Community 30 - "Frame Types"
Cohesion: 0.25
Nodes (9): Mat, TimestampSec, ImageFrame, image, timestamp, StereoFrame, left, right (+1 more)

### Community 31 - "LiDAR Point Types"
Cohesion: 0.22
Nodes (9): vector, LidarPoint, intensity, x, y, z, LidarScan, points (+1 more)

### Community 32 - "Optimizer Implementation Helpers"
Cohesion: 0.25
Nodes (7): Matrix, Params, size_t, SlidingWindowOptimizer::NumKeyframes(), SlidingWindowOptimizer::RotationOnlyInformation(), SlidingWindowOptimizer::SlidingWindowOptimizer(), SlidingWindowOptimizer::WeightedInformation()

### Community 33 - "LiDAR Scan Matcher"
Cohesion: 0.33
Nodes (8): Matrix, Matrix3d, optional, SE3d, Vector3d, MatchScans(), PointJacobian(), Skew()

### Community 34 - "CMake Build Targets"
Cohesion: 0.43
Nodes (8): Demo App Executables, pyslam pybind11 Module, SLAM_BUILD_* Options, slam_core Library Target, CI Build-and-Test Pipeline, slam_core Source List, slam_viz Library Target, slam_tests Executable

### Community 35 - "IMU Measurement & Preintegration Core"
Cohesion: 0.25
Nodes (6): Vector3d, ImuMeasurement, angular_velocity, linear_acceleration, timestamp, ImuPreintegrator::Integrate()

### Community 36 - "KITTI Raw Drive Mapping"
Cohesion: 0.29
Nodes (5): string, KittiRawDriveMapping, date, drive, start_frame

### Community 37 - "Scan Matcher Tests"
Cohesion: 0.39
Nodes (7): RecoversKnownSmallRigidTransform, ReturnsNulloptWithInsufficientFeatures, vector, Vector3d, MakeJitteredPlane(), MakeTwoOrthogonalLines(), TEST()

### Community 38 - "Map Implementation"
Cohesion: 0.32
Nodes (7): Params, vector, Vector3d, Map::ExportPly(), Map::InsertLandmarks(), Map::InsertLidarPoints(), Map::Map()

### Community 39 - "NavStateGraph Synthetic IMU Tests"
Cohesion: 0.29
Nodes (7): Vector3d, MakeSyntheticImuSegment(), SyntheticImuSegment, gravity, preintegration, state_i, state_j

### Community 40 - "Loop Closure Detection Tests"
Cohesion: 0.33
Nodes (6): FindsAndVerifiesNearbyCandidate, RejectsCandidateOutsideProximityRadius, RejectsCandidateTooRecent, ReturnsNulloptWithNoCandidates, MakeCommonPlanarFeatures(), TEST()

### Community 41 - "Pangolin Viewer Interface"
Cohesion: 0.29
Nodes (6): Impl, unique_ptr, PangolinViewer, impl_, ShouldClose, Update

### Community 42 - "Optimizer Edge Weighting"
Cohesion: 0.33
Nodes (6): EdgeMeasurement, RotationOnlyInformation, WeightedInformation, NodeId, optional, SlidingWindowOptimizer::AddKeyframe()

### Community 43 - "Scan Feature Extraction Tests"
Cohesion: 0.40
Nodes (5): FlagsSharpCornerAsEdgePoint, IgnoresRingsWithTooFewPoints, ProducesBoundedFeatureCounts, MakeSingleRingWithCorner(), TEST()

### Community 44 - "Loop Closure Params"
Cohesion: 0.33
Nodes (6): NodeId, LoopClosureParams, min_edge_correspondences, min_node_gap, min_planar_correspondences, proximity_radius_m

### Community 45 - "Loop Closure Result"
Cohesion: 0.33
Nodes (6): SE3d, LoopClosureResult, matched_node_id, num_edge_correspondences, num_planar_correspondences, relative_pose

### Community 46 - "Scan Match Result"
Cohesion: 0.33
Nodes (5): SE3d, ScanMatchResult, num_edge_correspondences, num_planar_correspondences, pose

### Community 47 - "Scan Matcher Params"
Cohesion: 0.33
Nodes (6): ScanMatcherParams, convergence_threshold, max_edge_correspondence_dist, max_iterations, max_planar_correspondence_dist, LidarFrontend::LidarFrontend()

### Community 48 - "VIO Track Triangulation"
Cohesion: 0.33
Nodes (5): TriangulateTracks, FrameResult, VioFrontend::ProcessImu(), VioFrontend::ProcessStereoFrame(), VioFrontend::VioFrontend()

### Community 49 - "Loop Closure Detection Implementation"
Cohesion: 0.33
Nodes (5): NodeId, optional, vector, Vector3d, DetectLoopClosure()

### Community 50 - "VIO Frontend Implementation Includes"
Cohesion: 0.33
Nodes (6): Mat, TrackId, unordered_map, vector, Vector3d, VioFrontend::TriangulateTracks()

### Community 51 - "Feature Tracker Implementation"
Cohesion: 0.40
Nodes (4): Mat, Params, FeatureTracker::DetectNewCorners(), FeatureTracker::FeatureTracker()

### Community 52 - "Python Bindings (pyslam)"
Cohesion: 0.50
Nodes (3): PYBIND11_MODULE(), m, pyslam

### Community 53 - "Relative Pose Estimate"
Cohesion: 0.50
Nodes (4): SE3d, RelativePoseEstimate, num_inliers, pose

### Community 54 - "LiDAR Frontend Implementation"
Cohesion: 0.67
Nodes (3): FrameResult, Downsample(), LidarFrontend::ProcessScan()

## Knowledge Gaps
- **256 isolated node(s):** `gyro`, `accel`, `linearization_bias_`, `raw_measurements_`, `delta_time_` (+251 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **17 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `SlamNode` connect `ROS2 SlamNode` to `KITTI Dataset Reader`, `ROS2 Message Adapters`, `VIO Feature Tracking Frontend`, `Demo Apps & Common Types`, `Sliding Window Optimizer`, `Point Cloud Map Export`, `LiDAR Frontend & Demo Apps`, `ROS2 Node Threading Primitives`, `Camera Calibration Types`?**
  _High betweenness centrality (0.185) - this node is a cross-community bridge._
- **Why does `ImuPreintegration` connect `IMU Factor & VIO Initializer` to `IMU Measurement & Preintegration Core`, `Demo Apps & Common Types`, `NavStateGraph Edge Types`, `NavStateGraph Synthetic IMU Tests`?**
  _High betweenness centrality (0.126) - this node is a cross-community bridge._
- **Why does `ScanFeatures` connect `Scan Features & Loop Closure Candidates` to `LiDAR Scan Matcher`, `ROS2 Message Adapters`, `Loop Closure Detection Tests`, `Optimizer Edge Weighting`, `Demo Apps & Common Types`, `Loop Closure Detection Implementation`, `LiDAR Frontend Implementation`, `LiDAR Frontend & Demo Apps`, `LOAM Feature Extraction Params`?**
  _High betweenness centrality (0.079) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `SlidingWindowOptimizer` (e.g. with `main()` and `main()`) actually correct?**
  _`SlidingWindowOptimizer` has 4 INFERRED edges - model-reasoned connections that need verification._
- **What connects `gyro`, `accel`, `linearization_bias_` to the rest of the system?**
  _256 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `IMU Factor & VIO Initializer` be split into smaller, more focused modules?**
  _Cohesion score 0.0531986531986532 - nodes in this community are weakly interconnected._
- **Should `ROS2 SlamNode` be split into smaller, more focused modules?**
  _Cohesion score 0.038461538461538464 - nodes in this community are weakly interconnected._