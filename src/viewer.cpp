//
// Created by root on 3/23/20.
//

#include "viewer.h"
#include <cv.hpp>
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
      pangolin::ProjectionMatrix(1024, 768, viewer_settings_.view_point_f,
                                 viewer_settings_.view_point_f, 512, 389, 0.1,
                                 1000),
      pangolin::ModelViewLookAt(
          viewer_settings_.view_point_x, viewer_settings_.view_point_y,
          viewer_settings_.view_point_z, 0, 0, 0, 0.0, -1.0, pangolin::AxisY));
  pangolin::Handler3D handler(s_cam);
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                              .SetHandler(&handler);

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    DrawCartisianCoordinate();

    //    DrawCamera();
    glPushMatrix();
    Eigen::Matrix4f m;
    m << 1, 0, 0, -1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // eigen by default use column major, which matches opengl
    glMultMatrixf((float *)m.data());
    DrawCamera();
    DrawCartisianCoordinate();

    glPopMatrix();

    cv::Mat p = (cv::Mat_<float>(1, 3) << 1, 2, 0);
    DrawMapPoint(p);
    pangolin::FinishFrame();
  }
}
Viewer::Viewer(const ViewerSettings &viewer_settings)
    : viewer_settings_(viewer_settings) {}
} // namespace clean_slam
