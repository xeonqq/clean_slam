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
  const double fx = camera_intrinsic.at<double>(0, 0);
  const double fy = camera_intrinsic.at<double>(1, 1);
  const double x0 = camera_intrinsic.at<double>(0, 2);
  const double y0 = camera_intrinsic.at<double>(1, 2);
  std::transform(
      points_3d.begin(), points_3d.end(), points_reprojected.begin(),
      [&](const Eigen::Vector3d &point_3d) {
        const Eigen::Vector3d point_3d_cam_frame = current_pose.map(point_3d);
        Eigen::Vector2d point_image;
        point_image << point_3d_cam_frame[0] * fx / point_3d_cam_frame[2] + x0,
            point_3d_cam_frame[1] * fy / point_3d_cam_frame[2] + y0;
        return point_image;
      });
  return points_reprojected;
}

} // namespace clean_slam