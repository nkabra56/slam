#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "slam/sensors/kitti_dataset.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pyslam, m) {
  m.doc() = "Python bindings for the slam project (grows alongside each ROADMAP phase).";

  py::class_<slam::sensors::KittiSequenceReader>(m, "KittiSequenceReader")
      .def(py::init<const std::filesystem::path&, std::optional<std::filesystem::path>>(),
           py::arg("sequence_dir"), py::arg("poses_file") = std::nullopt)
      .def("num_frames", &slam::sensors::KittiSequenceReader::NumFrames)
      .def("timestamp_at", &slam::sensors::KittiSequenceReader::TimestampAt)
      .def("has_ground_truth", &slam::sensors::KittiSequenceReader::HasGroundTruth);
}
