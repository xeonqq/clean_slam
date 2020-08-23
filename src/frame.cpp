//
// Created by root on 5/12/20.
//
#include "frame.h"
#include "cv_utils.h"
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/transform.hpp>

namespace clean_slam {

Points3DView Frame::GetPoints3DView() const {
  return Points3DView(_map->GetPoints3D(), _map_point_indexes);
}

DescriptorsView Frame::GetDescriptorsView() const {
  return DescriptorsView(_map->GetDescriptors(), _map_point_indexes);
}

const std::vector<cv::KeyPoint> &Frame::GetKeyPoints() const {
  return _key_points;
}

cv::Mat Frame::GetDescriptors() const {
  const auto view = GetDescriptorsView();
  cv::Mat descriptors = cv::Mat(view.size(), 32, CV_8UC1);
  for (size_t i = 0; i < view.size(); ++i) {
    descriptors.at<DescriptorT>(i) = view[i];
  }
  return descriptors;
}
std::vector<uint8_t> Frame::GetOctaves() const {
  std::vector<uint8_t> octaves;
  octaves.reserve(_key_points.size());
  boost::range::transform(
      _key_points, std::back_inserter(octaves),
      [](const cv::KeyPoint &key_point) { return key_point.octave; });
  return octaves;
}

std::vector<Eigen::Vector2d>
Frame::ReprojectPoints3d(const g2o::SE3Quat &current_pose,
                         const cv::Mat &camera_intrinsic) const {
  const auto points_3d = GetPoints3DView();
  std::vector<Eigen::Vector2d> points_reprojected;
  points_reprojected.resize(points_3d.size());
  clean_slam::ReprojectPoints3d(points_3d, std::begin(points_reprojected),
                                current_pose, camera_intrinsic);
  return points_reprojected;
}

std::vector<cv::DMatch> Frame::SearchByProjection(
    const OrbFeatureMatcher &matcher, const OrbFeatures &features,
    const std::vector<Eigen::Vector2d> &projected_map_points,
    const cv::Mat &mask, int search_radius,
    const OctaveScales &octave_scales) const {

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

std::vector<Eigen::Vector3d>
Frame::GetMatchedMapPoints(const std::vector<cv::DMatch> &matches) const {
  const auto matched_indexes = boost::adaptors::transform(
      matches, [](const auto &match) { return match.queryIdx; });

  return FilterByIndex(GetPoints3DView(), matched_indexes);
}

std::vector<size_t>
Frame::GetMatchedMapPointsIds(const std::vector<cv::DMatch> &matches) const {
  const auto points_3d_view = GetPoints3DView();
  std::vector<size_t> matched_map_ids;
  matched_map_ids.reserve(matches.size());
  boost::range::transform(
      matches, std::back_inserter(matched_map_ids), [&points_3d_view](const auto &match) { return points_3d_view.MapIndex(match.queryIdx); });
  return matched_map_ids;
}
} // namespace clean_slam
