//
// Created by root on 4/11/20.
//
#include <third_party/g2o/g2o/types/se3quat.h>

#include "cv_utils.h"
#include "key_frame.h"
#include <iostream>
namespace clean_slam {

KeyFrame KeyFrame::Create(const g2o::SE3Quat &Tcw,
                          std::vector<Eigen::Vector3d> &&points_3d,
                          const cv::Mat &descriptors,
                          const std::vector<cv::KeyPoint> &key_points,
                          const OctaveScales &octave_scales) {
  std::vector<Bound> distance_bounds;
  distance_bounds.reserve(points_3d.size());
  const auto pose_in_world = Tcw.inverse();
  for (size_t i = 0; i < points_3d.size(); ++i) {
    const auto distance = (points_3d[i] - pose_in_world.translation()).norm();
    float octave_scale = octave_scales[key_points[i].octave];
    const auto max_distance = octave_scale * distance;
    const auto min_distance =
        max_distance / octave_scales[octave_scales.size() - 1];
    distance_bounds.emplace_back(min_distance, max_distance);
  }
  return KeyFrame(Tcw, std::move(points_3d), descriptors,
                  std::move(distance_bounds));
}

KeyFrame::KeyFrame(const g2o::SE3Quat &Tcw,
                   std::vector<Eigen::Vector3d> &&points_3d,
                   const cv::Mat &descriptors,
                   std::vector<Bound> &&distance_bounds)
    : _Tcw(Tcw), _points_3d(std::move(points_3d)),
      _descriptors(descriptors), _distance_bounds{std::move(distance_bounds)} {}

const g2o::SE3Quat &KeyFrame::GetTcw() const { return _Tcw; }

const std::vector<Eigen::Vector3d> &KeyFrame::GetPoints3D() const {
  return _points_3d;
}

const cv::Mat &KeyFrame::GetDescriptors() const { return _descriptors; }

const std::vector<Bound> &KeyFrame::GetDistanceBounds() const {
  return _distance_bounds;
}

} // namespace clean_slam