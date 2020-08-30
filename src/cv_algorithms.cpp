//
// Created by root on 4/18/20.
//
#include "cv_algorithms.h"
#include <opencv2/features2d.hpp>
namespace clean_slam {

g2o::SE3Quat GetVelocity(const g2o::SE3Quat &Tcw_current,
                         const g2o::SE3Quat &Tcw_prev) {
  return Tcw_current * Tcw_prev.inverse();
}

bool KeyPointWithinRadius(const cv::KeyPoint &key_point,
                          const Eigen::Vector2d &point, float radius) {
  const auto distance_sqr = std::pow((point.x() - key_point.pt.x), 2) +
                            std::pow((point.y() - key_point.pt.y), 2);
  return distance_sqr <= std::pow(radius, 2);
}

BoundF Calculate3DPointDistanceBound(const g2o::SE3Quat &Tcw,
                                     const cv::KeyPoint &key_point,
                                     const Eigen::Vector3d &point_3d,
                                     const OctaveScales &octave_scales) {
  const auto pose_in_world = Tcw.inverse();
  const auto distance = (point_3d - pose_in_world.translation()).norm();
  float octave_scale = octave_scales[key_point.octave];
  const float max_distance = octave_scale * distance;
  const float min_distance =
      max_distance / octave_scales[octave_scales.size() - 1];
  return {min_distance, max_distance};
}

Eigen::Vector3d ViewingDirection(const g2o::SE3Quat &Tcw,
                                 const Eigen::Vector3d &point_3d) {
  const auto pose_in_world = Tcw.inverse();
  Eigen::Vector3d dir = point_3d - pose_in_world.translation();
  dir.normalize();
  return dir;
}
} // namespace clean_slam