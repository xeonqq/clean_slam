//
// Created by root on 4/18/20.
//

#ifndef CLEAN_SLAM_SRC_CV_ALGORITHMS_H_
#define CLEAN_SLAM_SRC_CV_ALGORITHMS_H_

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/types/se3quat.h>
#include <vector>

namespace clean_slam {

g2o::SE3Quat GetVelocity(const g2o::SE3Quat &Tcw_current,
                         const g2o::SE3Quat &Tcw_prev);

std::vector<Eigen::Vector2d>
ReprojectPoints3d(const std::vector<Eigen::Vector3d> &points_3d,
                  const g2o::SE3Quat &current_pose,
                  const cv::Mat &camera_intrinsic);

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_CV_ALGORITHMS_H_
