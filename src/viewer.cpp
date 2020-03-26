//
// Created by root on 3/23/20.
//

#include "viewer.h"
#include <cv.hpp>
#include <thread>
namespace clean_slam {
void DrawCartisianCoordinate() {
  glLineWidth(3);
  glBegin(GL_LINES);
  glColor3f(0.8f, 0.f, 0.f);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0);
  glColor3f(0.f, 0.8f, 0.f);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 1, 0);
  glColor3f(0.2f, 0.2f, 1.f);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 1);
  glEnd();
}

void DrawCamera() {
  const float w = 1;
  const float h = w * 0.75;
  const float z = w * 0.6;
  glLineWidth(2);
  glBegin(GL_LINES);
  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);
  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);
  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);
  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);

  glEnd();
}

void DrawMapPoint(cv::Mat point_3d) {

  glPointSize(2);
  glBegin(GL_POINTS);
  glColor3f(1.0, 0.0, 0.0);
  glVertex3f(point_3d.at<float>(0), point_3d.at<float>(1),
             point_3d.at<float>(2));
  glEnd();
}

void DrawMapPoints(cv::Mat points_3d) {

  for (int i = 0; i < points_3d.rows; ++i) {
    DrawMapPoint(points_3d.row(i));
  }
}

void DrawCameraWithCoordinate() {
  DrawCamera();
  DrawCartisianCoordinate();
}

Viewer::Viewer(const ViewerSettings &viewer_settings)
    : _viewer_settings_(viewer_settings) {
  //  _contents.push_back(Content{});
}

void Viewer::Run() {
  pangolin::CreateWindowAndBind("Main", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(150));
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, _viewer_settings_.view_point_f,
                                 _viewer_settings_.view_point_f, 512, 389, 0.1,
                                 1000),
      pangolin::ModelViewLookAt(
          _viewer_settings_.view_point_x, _viewer_settings_.view_point_y,
          _viewer_settings_.view_point_z, 0, 0, 0, 0.0, -1.0, pangolin::AxisY));
  pangolin::Handler3D handler(s_cam);
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                              .SetHandler(&handler);

  while (!pangolin::ShouldQuit()) {
    //    Content content;
    //    {
    //      std::lock_guard<std::mutex> lock(_mutex);
    //      content = _contents.front();
    //      if (_contents.size() > 1)
    //        _contents.pop();
    //    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    d_cam.Activate(s_cam);
    for (const auto &content : _contents) {

      const auto camera_pose =
          content.homogeneous_matrix.to_homogeneous_matrix();
      glPushMatrix();
      glMultMatrixd((double *)camera_pose.data());
      DrawCameraWithCoordinate();
      glPopMatrix();

      DrawMapPoints(content.triangulated_points);
    }
    pangolin::FinishFrame();
  }
}
} // namespace clean_slam
