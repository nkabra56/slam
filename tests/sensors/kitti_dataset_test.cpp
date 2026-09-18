#include "slam/sensors/kitti_dataset.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

namespace slam::sensors {
namespace {

namespace fs = std::filesystem;

class KittiSequenceReaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    sequence_dir_ = fs::temp_directory_path() / "slam_kitti_test_sequence";
    fs::remove_all(sequence_dir_);
    fs::create_directories(sequence_dir_ / "image_0");
    fs::create_directories(sequence_dir_ / "velodyne");

    {
      std::ofstream times(sequence_dir_ / "times.txt");
      times << "0.0\n0.1\n0.2\n";
    }

    for (int i = 0; i < 3; ++i) {
      cv::Mat image(2, 2, CV_8UC1, cv::Scalar(i));
      char name[16];
      std::snprintf(name, sizeof(name), "%06d.png", i);
      cv::imwrite((sequence_dir_ / "image_0" / name).string(), image);
    }

    for (int i = 0; i < 3; ++i) {
      char name[16];
      std::snprintf(name, sizeof(name), "%06d.bin", i);
      std::ofstream bin(sequence_dir_ / "velodyne" / name, std::ios::binary);
      const float points[8] = {1.0f, 2.0f, 3.0f, 0.5f, -1.0f, -2.0f, -3.0f, 0.9f};
      bin.write(reinterpret_cast<const char*>(points), sizeof(points));
    }

    poses_file_ = sequence_dir_ / "poses.txt";
    std::ofstream poses(poses_file_);
    for (int i = 0; i < 3; ++i) {
      poses << "1 0 0 " << i << " 0 1 0 0 0 0 1 0\n";
    }

    std::ofstream calib(sequence_dir_ / "calib.txt");
    calib << "P0: 700 0 300 0 0 700 200 0 0 0 1 0\n";
    calib << "P1: 700 0 300 -350 0 700 200 0 0 0 1 0\n";
    calib << "P2: 700 0 300 0 0 700 200 0 0 0 1 0\n";
    calib << "P3: 700 0 300 -350 0 700 200 0 0 0 1 0\n";
  }

  void TearDown() override { fs::remove_all(sequence_dir_); }

  fs::path sequence_dir_;
  fs::path poses_file_;
};

TEST_F(KittiSequenceReaderTest, ReadsTimestamps) {
  KittiSequenceReader reader(sequence_dir_);
  EXPECT_EQ(reader.NumFrames(), 3u);
  EXPECT_DOUBLE_EQ(reader.TimestampAt(1), 0.1);
}

TEST_F(KittiSequenceReaderTest, LoadsImages) {
  KittiSequenceReader reader(sequence_dir_);
  const ImageFrame frame = reader.LoadImage(0);
  EXPECT_EQ(frame.image.rows, 2);
  EXPECT_EQ(frame.image.cols, 2);
}

TEST_F(KittiSequenceReaderTest, LoadsLidarScans) {
  KittiSequenceReader reader(sequence_dir_);
  const LidarScan scan = reader.LoadLidarScan(0);
  ASSERT_EQ(scan.points.size(), 2u);
  EXPECT_FLOAT_EQ(scan.points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(scan.points[1].intensity, 0.9f);
}

TEST_F(KittiSequenceReaderTest, ReadsGroundTruthPoses) {
  KittiSequenceReader reader(sequence_dir_, poses_file_);
  ASSERT_TRUE(reader.HasGroundTruth());
  const Eigen::Isometry3d pose = reader.GroundTruthPoseAt(2);
  EXPECT_DOUBLE_EQ(pose.translation().x(), 2.0);
}

TEST_F(KittiSequenceReaderTest, LoadsCalibration) {
  KittiSequenceReader reader(sequence_dir_);
  const StereoCalibration calib = reader.LoadCalibration();
  EXPECT_DOUBLE_EQ(calib.left.fx, 700.0);
  EXPECT_DOUBLE_EQ(calib.left.fy, 700.0);
  EXPECT_DOUBLE_EQ(calib.left.cx, 300.0);
  EXPECT_DOUBLE_EQ(calib.left.cy, 200.0);
  EXPECT_DOUBLE_EQ(calib.baseline_m, 0.5);
}

TEST_F(KittiSequenceReaderTest, LoadsLidarToCameraFromTrLine) {
  {
    std::ofstream calib(sequence_dir_ / "calib.txt", std::ios::app);
    // cam = (-vel_y, -vel_z, vel_x) + (0.1, 0.2, 0.3)
    calib << "Tr: 0 -1 0 0.1 0 0 -1 0.2 1 0 0 0.3\n";
  }

  KittiSequenceReader reader(sequence_dir_);
  const Eigen::Vector3d p_camera = reader.LoadLidarToCamera() * Eigen::Vector3d(1.0, 2.0, 3.0);
  EXPECT_NEAR(p_camera.x(), -1.9, 1e-12);
  EXPECT_NEAR(p_camera.y(), -2.8, 1e-12);
  EXPECT_NEAR(p_camera.z(), 1.3, 1e-12);
}

TEST_F(KittiSequenceReaderTest, ThrowsWhenTrLineIsMissing) {
  KittiSequenceReader reader(sequence_dir_);
  EXPECT_THROW(reader.LoadLidarToCamera(), std::runtime_error);
}

TEST_F(KittiSequenceReaderTest, ThrowsOnMissingDirectory) {
  EXPECT_THROW(KittiSequenceReader(sequence_dir_ / "does_not_exist"), std::runtime_error);
}

}  // namespace
}  // namespace slam::sensors
