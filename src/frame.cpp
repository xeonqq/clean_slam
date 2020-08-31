//
// Created by root on 5/12/20.
//
#include "frame.h"
#include "cv_utils.h"
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/transform.hpp>

namespace clean_slam {

const std::vector<cv::KeyPoint> &Frame::GetKeyPoints() const {
  return _matched_key_points;
}

cv::Mat Frame::GetDescriptors() const {
  cv::Mat descriptors = cv::Mat(_matched_map_points.size(), 32, CV_8UC1);
  for (size_t i = 0; i < _matched_map_points.size(); ++i) {
    _matched_map_points[i]->GetRepresentativeDescriptor().copyTo(
        descriptors.row(i));
  }
  return descriptors;
}
std::vector<uint8_t> Frame::GetOctaves() const {
  std::vector<uint8_t> octaves;
  octaves.reserve(_matched_key_points.size());
  boost::range::transform(
      _matched_key_points, std::back_inserter(octaves),
      [](const cv::KeyPoint &key_point) { return key_point.octave; });
  return octaves;
}

std::vector<Eigen::Vector2d>
Frame::ReprojectPoints3d(const g2o::SE3Quat &current_pose,
                         const cv::Mat &camera_intrinsic) const {
  std::vector<Eigen::Vector2d> points_reprojected;
  points_reprojected.resize(_matched_map_points.size());
  clean_slam::ReprojectPoints3d(GetMapPointsPositions(_matched_map_points),
                                std::begin(points_reprojected), current_pose,
                                camera_intrinsic);
  return points_reprojected;
}

std::vector<cv::DMatch> Frame::SearchByProjection(
    const OrbFeatureMatcher &matcher, const OrbFeatures &features,
    const std::vector<Eigen::Vector2d> &projected_map_points,
    const cv::Mat &mask, int search_radius,
    const OctaveScales &octave_scales) const {

  /*
   * We have orb features (key points + feature descriptors) from current frame
   * and 3d map points observed from the last frame projected onto the current
   * frame, (some are not valid, indicated by mask) we also know the descriptors
   * of each projected points
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
      matcher.KnnMatch(GetDescriptors(), current_descriptors,
                       kNumNearestNeighbor);

  const auto &current_key_points = features.GetUndistortedKeyPoints();

  std::vector<cv::DMatch> matched_pairs;
  const auto map_points_octaves = GetOctaves();
  matched_pairs.reserve(map_points_octaves.size());

  for (const auto &matches_per_map_point :
       matches_for_map_points | boost::adaptors::indexed()) {

    if (!mask.at<uint8_t>(matches_per_map_point.index()))
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
          m.distance <= kDescriptorDistanceThreshold) {
        matched_pairs.push_back(m);
        break;
      }
    }
  }
  return matched_pairs;
}

std::vector<MapPoint *>
Frame::GetMatchedMapPoints(const std::vector<cv::DMatch> &matches) const {
  const auto matched_indexes = boost::adaptors::transform(
      matches, [](const auto &match) { return match.queryIdx; });

  return FilterByIndex(_matched_map_points, matched_indexes);
}

const KeyFrame &Frame::GetRefKeyFrame() const {
  return _map->GetKeyFrame(_ref_kf);
}

size_t Frame::GetRefKeyFrameNumKeyPoints() const {
  return GetRefKeyFrame().NumKeyPoints();
}
vertex_t Frame::GetRefKfVertex() const { return _ref_kf; }
} // namespace clean_slam
