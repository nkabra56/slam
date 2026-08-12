#pragma once

#include "slam/common/types.hpp"

namespace slam::frontend_vio {

// Tracks visual features across frames and preintegrates IMU measurements
// between them to produce frame-to-frame pose estimates.
//
// TODO(phase-1): feature detection/tracking (Shi-Tomasi + KLT), IMU
// preintegration, and initial relative pose estimation (5-point / PnP).
class VioFrontend {
 public:
  void ProcessImage(const ImageFrame& frame);
  void ProcessImu(const ImuMeasurement& measurement);
};

}  // namespace slam::frontend_vio
