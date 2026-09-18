#include "slam/sensors/kitti_raw_imu.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <gtest/gtest.h>

namespace slam::sensors {
namespace {

namespace fs = std::filesystem;

std::string ZeroPadded(std::size_t index) {
  std::ostringstream oss;
  oss << std::setw(10) << std::setfill('0') << index;
  return oss.str();
}

class KittiOxtsReaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    oxts_dir_ = fs::temp_directory_path() / "slam_oxts_test";
    fs::remove_all(oxts_dir_);
    fs::create_directories(oxts_dir_ / "data");

    {
      std::ofstream timestamps(oxts_dir_ / "timestamps.txt");
      timestamps << "2011-10-03 12:00:00.000000000\n";
      timestamps << "2011-10-03 12:00:00.100000000\n";
      timestamps << "2011-10-03 12:00:00.200000000\n";
      timestamps << "2011-10-03 12:00:00.300000000\n";
    }

    // 23 fields: lat lon alt roll pitch yaw vn ve vf vl vu | ax ay az | af al au | wx wy wz | wf wl wu
    for (int i = 0; i < 4; ++i) {
      std::ofstream data(oxts_dir_ / "data" / (ZeroPadded(static_cast<std::size_t>(i)) + ".txt"));
      for (int f = 0; f < 11; ++f) data << "0 ";
      data << (1.0 + i) << " " << (2.0 + i) << " " << (3.0 + i) << " ";  // ax ay az
      data << "0 0 0 ";                                                 // af al au (unused)
      data << (0.1 + i) << " " << (0.2 + i) << " " << (0.3 + i) << " ";  // wx wy wz
      data << "0 0 0\n";                                                 // wf wl wu (unused)
    }
  }

  void TearDown() override { fs::remove_all(oxts_dir_); }

  fs::path oxts_dir_;
};

TEST_F(KittiOxtsReaderTest, ReadsAccelAndGyroFromBodyFrameFields) {
  KittiOxtsReader reader(oxts_dir_);
  ASSERT_EQ(reader.NumMeasurements(), 4u);

  const ImuMeasurement m = reader.MeasurementAt(2);
  EXPECT_DOUBLE_EQ(m.linear_acceleration.x(), 3.0);
  EXPECT_DOUBLE_EQ(m.linear_acceleration.y(), 4.0);
  EXPECT_DOUBLE_EQ(m.linear_acceleration.z(), 5.0);
  EXPECT_NEAR(m.angular_velocity.x(), 2.1, 1e-9);
  EXPECT_NEAR(m.angular_velocity.y(), 2.2, 1e-9);
  EXPECT_NEAR(m.angular_velocity.z(), 2.3, 1e-9);
}

TEST_F(KittiOxtsReaderTest, TimestampsAreRelativeToStartFrame) {
  KittiOxtsReader reader(oxts_dir_);
  EXPECT_NEAR(reader.MeasurementAt(0).timestamp, 0.0, 1e-6);
  EXPECT_NEAR(reader.MeasurementAt(2).timestamp, 0.2, 1e-6);
}

TEST_F(KittiOxtsReaderTest, StartFrameOffsetsIndexingAndZeroesTimestamp) {
  KittiOxtsReader reader(oxts_dir_, /*start_frame=*/2);
  ASSERT_EQ(reader.NumMeasurements(), 2u);
  EXPECT_NEAR(reader.MeasurementAt(0).timestamp, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(reader.MeasurementAt(0).linear_acceleration.x(), 3.0);  // raw frame 2's ax
}

TEST_F(KittiOxtsReaderTest, ThrowsWhenStartFrameBeyondEnd) {
  EXPECT_THROW(KittiOxtsReader(oxts_dir_, /*start_frame=*/10), std::runtime_error);
}

TEST(LookupRawDriveMapping, FindsKnownSequences) {
  const auto m00 = LookupRawDriveMapping("00");
  ASSERT_TRUE(m00.has_value());
  EXPECT_EQ(m00->date, "2011_10_03");
  EXPECT_EQ(m00->drive, "2011_10_03_drive_0027");
  EXPECT_EQ(m00->start_frame, 0u);

  const auto m08 = LookupRawDriveMapping("08");
  ASSERT_TRUE(m08.has_value());
  EXPECT_EQ(m08->drive, "2011_09_30_drive_0028");
  EXPECT_EQ(m08->start_frame, 1100u);
}

TEST(LookupRawDriveMapping, ReturnsNulloptForUnknownSequence) {
  EXPECT_FALSE(LookupRawDriveMapping("21").has_value());
  EXPECT_FALSE(LookupRawDriveMapping("not-a-sequence").has_value());
}

}  // namespace
}  // namespace slam::sensors
