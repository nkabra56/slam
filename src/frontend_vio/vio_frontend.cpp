#include "slam/frontend_vio/vio_frontend.hpp"

namespace slam::frontend_vio {

void VioFrontend::ProcessImage(const ImageFrame& /*frame*/) {
  // TODO(phase-1): feature tracking + relative pose estimation.
}

void VioFrontend::ProcessImu(const ImuMeasurement& /*measurement*/) {
  // TODO(phase-1): IMU preintegration between keyframes.
}

}  // namespace slam::frontend_vio
