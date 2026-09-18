#include "slam/viz/pangolin_viewer.hpp"

#include <pangolin/pangolin.h>

namespace slam::viz {

struct PangolinViewer::Impl {
  std::string window_name;
  pangolin::OpenGlRenderState render_state;
  pangolin::View* display = nullptr;
};

PangolinViewer::PangolinViewer(std::string window_name) : impl_(std::make_unique<Impl>()) {
  impl_->window_name = window_name;

  pangolin::CreateWindowAndBind(window_name, 1024, 768);
  glEnable(GL_DEPTH_TEST);

  impl_->render_state = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -20, -20, 0, 0, 0, pangolin::AxisNegY));

  impl_->display = &pangolin::CreateDisplay()
                        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                        .SetHandler(new pangolin::Handler3D(impl_->render_state));
}

PangolinViewer::~PangolinViewer() {
  if (impl_) {
    pangolin::DestroyWindow(impl_->window_name);
  }
}

void PangolinViewer::Update(const std::vector<Sophus::SE3d>& trajectory,
                             const std::vector<Eigen::Vector3d>& landmark_points,
                             const std::vector<Eigen::Vector3d>& lidar_points) {
  if (pangolin::ShouldQuit()) return;

  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  impl_->display->Activate(impl_->render_state);

  glColor3f(0.0f, 0.0f, 0.0f);
  glLineWidth(2.0f);
  glBegin(GL_LINE_STRIP);
  for (const auto& pose : trajectory) {
    const Eigen::Vector3d t = pose.translation();
    glVertex3d(t.x(), t.y(), t.z());
  }
  glEnd();

  glPointSize(2.0f);
  glColor3f(0.0f, 0.6f, 0.0f);
  glBegin(GL_POINTS);
  for (const auto& p : landmark_points) glVertex3d(p.x(), p.y(), p.z());
  glEnd();

  glPointSize(1.0f);
  glColor3f(0.4f, 0.4f, 0.4f);
  glBegin(GL_POINTS);
  for (const auto& p : lidar_points) glVertex3d(p.x(), p.y(), p.z());
  glEnd();

  pangolin::FinishFrame();
}

bool PangolinViewer::ShouldClose() const { return pangolin::ShouldQuit(); }

}  // namespace slam::viz
