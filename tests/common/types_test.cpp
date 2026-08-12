#include "slam/common/types.hpp"

#include <gtest/gtest.h>

namespace slam {
namespace {

TEST(Types, ImuMeasurementDefaultsToZero) {
  ImuMeasurement measurement;
  EXPECT_TRUE(measurement.angular_velocity.isZero());
  EXPECT_TRUE(measurement.linear_acceleration.isZero());
}

TEST(Types, LidarScanHoldsPoints) {
  LidarScan scan;
  scan.points.push_back(LidarPoint{1.0f, 2.0f, 3.0f, 0.5f});
  ASSERT_EQ(scan.points.size(), 1u);
  EXPECT_FLOAT_EQ(scan.points[0].z, 3.0f);
}

}  // namespace
}  // namespace slam
