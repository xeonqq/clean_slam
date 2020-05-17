//
// Created by root on 4/18/20.
//

#ifndef CLEAN_SLAM_SRC_CV_ALGORITHMS_H_
#define CLEAN_SLAM_SRC_CV_ALGORITHMS_H_

#include "frame.h"
#include <Eigen/Dense>
#include <boost/range/algorithm.hpp>
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

template <class SinglePassRange1, class OutputIterator>
void ReprojectPoints3d(const SinglePassRange1 &points_3d, OutputIterator out,
                       const g2o::SE3Quat &current_pose,
                       const cv::Mat &camera_intrinsic) {
  const double fx = camera_intrinsic.at<double>(0, 0);
  const double fy = camera_intrinsic.at<double>(1, 1);
  const double x0 = camera_intrinsic.at<double>(0, 2);
  const double y0 = camera_intrinsic.at<double>(1, 2);
  boost::range::transform(points_3d, out, [&](const Eigen::Vector3d &point_3d) {
    const Eigen::Vector3d point_3d_cam_frame = current_pose.map(point_3d);
    Eigen::Vector2d point_image;
    point_image << point_3d_cam_frame[0] * fx / point_3d_cam_frame[2] + x0,
        point_3d_cam_frame[1] * fy / point_3d_cam_frame[2] + y0;
    return point_image;
  });
}

std::vector<int> GetPointsInArea(const Eigen::Vector2d &center, float radius,
                                 const std::vector<cv::KeyPoint> &key_points);

std::vector<std::pair<size_t, size_t>>
SearchByProjection(const OrbFeatures &features,
                   const std::vector<Eigen::Vector2d> &projected_map_points,
                   const DescriptorsView &map_points_descriptors,
                   const std::vector<bool> &mask, int search_radius);
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_CV_ALGORITHMS_H_
