//
// Created by root on 4/18/20.
//
#include "cv_algorithms.h"

namespace clean_slam {

g2o::SE3Quat GetVelocity(const g2o::SE3Quat &Tcw_current,
                         const g2o::SE3Quat &Tcw_prev) {
  return Tcw_current * Tcw_prev.inverse();
}

std::vector<Eigen::Vector2d>
ReprojectPoints3d(const std::vector<Eigen::Vector3d> &points_3d,
                  const g2o::SE3Quat &current_pose,
                  const cv::Mat &camera_intrinsic) {
  std::vector<Eigen::Vector2d> points_reprojected;
  points_reprojected.resize(points_3d.size());
  ReprojectPoints3d(points_3d, points_reprojected.begin(), current_pose,
                    camera_intrinsic);
  return points_reprojected;
}

} // namespace clean_slam