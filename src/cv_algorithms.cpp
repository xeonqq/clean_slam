//
// Created by root on 4/18/20.
//
#include "cv_algorithms.h"
#include "match_map.h"
#include <boost/range/adaptor/indexed.hpp>
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

std::vector<cv::DMatch> SearchByProjection(
    const OrbFeatureMatcher &matcher,
    const std::vector<Eigen::Vector2d> &projected_map_points,
    const cv::Mat &projected_map_points_descriptors,
    const std::vector<uint8_t> &map_points_octaves, const OrbFeatures &features,
    const cv::Mat &mask, int search_radius, const OctaveScales &octave_scales,
    int num_nearest_neighbor, int descriptor_distance_threshold) {

  /*
   * We have orb features (key points + feature descriptors) from current frame,
   * member and 3d map points observed from the last frame projected onto the
   * current frame, (some are not valid, indicated by mask) we also know the
   * descriptors of each projected points
   *
   * Then for each projected map point, we need to find out the same point
   * observed in current frame, based on descriptor distance. return the matched
   * map points index and current key point index pairs
   *
   * Once we have the matched map points indexes for the newly observed points,
   * we can perform again a bundle adjustment.
   * */
  const auto &current_descriptors = features.GetDescriptors();
  std::vector<std::vector<cv::DMatch>> matches_for_map_points =
      matcher.KnnMatch(projected_map_points_descriptors, current_descriptors,
                       num_nearest_neighbor);

  const auto &current_key_points = features.GetUndistortedKeyPoints();

  MatchMap map;

  for (const auto &matches_per_map_point :
       matches_for_map_points | boost::adaptors::indexed()) {

    if (!mask.empty() && !mask.at<uint8_t>(matches_per_map_point.index()))
      continue;

    for (const cv::DMatch &m : matches_per_map_point.value()) {
      const auto &projected_map_point = projected_map_points[m.queryIdx];
      const auto &current_key_point = current_key_points[m.trainIdx];

      // find the first one which satisfies the condition, then stop
      BoundI octave_bound{map_points_octaves[m.queryIdx] - 1,
                          map_points_octaves[m.queryIdx] + 1};
      if (octave_bound.IsWithIn(current_key_point.octave) &&
          KeyPointWithinRadius(current_key_point, projected_map_point,
                               search_radius *
                                   octave_scales[current_key_point.octave]) &&
          m.distance <= descriptor_distance_threshold) {

        map.Emplace(m);
        break;
      }
    }
  }
  return map.ToVector();
}

} // namespace clean_slam