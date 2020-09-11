//
// Created by root on 5/12/20.
//
#include "frame.h"
#include "cv_utils.h"
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/transform.hpp>

namespace clean_slam {

cv::Mat Frame::GetMatchedMapPointsDescriptors() const {
  return GetMapPointsDescriptors(GetMatchedMapPointsRng());
}

std::vector<uint8_t> Frame::GetMatchedMapPointsOctaves() const {
  std::vector<uint8_t> octaves;
  octaves.reserve(_matched_map_point_to_idx.size());
  const auto &key_points = _orb_features.GetUndistortedKeyPoints();
  for (auto kp_idx : _matched_map_point_to_idx | boost::adaptors::map_values) {
    octaves.push_back(key_points[kp_idx].octave);
  }
  return octaves;
}

std::vector<cv::KeyPoint> Frame::GetUnmatchedKeyPoints() const {
  return RemoveByIndex(GetUndistortedKeyPoints(),
                       _matched_map_point_to_idx | boost::adaptors::map_values);
}
cv::Mat Frame::GetUnmatchedKeyPointsDescriptors() const {
  return RemoveByIndex(GetOrbFeatures().GetDescriptors(),
                       _matched_map_point_to_idx | boost::adaptors::map_values);
}

std::vector<Eigen::Vector2d>
Frame::ReprojectPoints3d(const std::vector<Eigen::Vector3d> &points_3d,
                         const cv::Mat &camera_intrinsic) const {
  std::vector<Eigen::Vector2d> points_reprojected;
  points_reprojected.reserve(points_3d.size());
  clean_slam::ReprojectPoints3d(points_3d,
                                std::back_inserter(points_reprojected), _Tcw,
                                camera_intrinsic);
  return points_reprojected;
}

std::vector<cv::DMatch> Frame::SearchByProjection(
    const OrbFeatureMatcher &matcher,
    const std::vector<Eigen::Vector2d> &projected_map_points,
    const Frame &prev_frame, const cv::Mat &mask, int search_radius,
    const OctaveScales &octave_scales) {

  auto matches_map_point_to_key_point = clean_slam::SearchByProjection(
      matcher, projected_map_points,
      prev_frame.GetMatchedMapPointsDescriptors(),
      prev_frame.GetMatchedMapPointsOctaves(), _orb_features, mask,
      search_radius, octave_scales, kNumNearestNeighbor,
      kDescriptorDistanceThreshold);
  const auto prev_frame_matches_map_points = prev_frame.GetMatchedMapPoints();
  for (const auto &match : matches_map_point_to_key_point) {
    _matched_map_point_to_idx.emplace(
        prev_frame_matches_map_points[match.queryIdx], match.trainIdx);
  }

  return matches_map_point_to_key_point;
}

size_t Frame::SearchUnmatchedKeyPointsByProjection(
    const OrbFeatureMatcher &matcher, const std::vector<MapPoint *> &map_points,
    const std::vector<Eigen::Vector2d> &projected_map_points,
    const std::vector<uint8_t> &map_points_octaves, int search_radius,
    const OctaveScales &octave_scales) {
  auto unmatched_orb_features =
      OrbFeatures(GetUnmatchedKeyPoints(), GetUnmatchedKeyPointsDescriptors());
  const auto matches_map_point_to_key_point = clean_slam::SearchByProjection(
      matcher, projected_map_points, GetMapPointsDescriptors(map_points),
      map_points_octaves, unmatched_orb_features, cv::Mat{}, search_radius,
      octave_scales, kNumNearestNeighbor, kDescriptorDistanceThreshold);

  const auto unmatched_key_points_idxes = GetUnmatchedIndexes(
      GetUndistortedKeyPoints().size(),
      _matched_map_point_to_idx | boost::adaptors::map_values);
  for (const auto &match : matches_map_point_to_key_point) {
    _matched_map_point_to_idx.emplace(
        map_points[match.queryIdx], unmatched_key_points_idxes[match.trainIdx]);
  }

  return matches_map_point_to_key_point.size();
}
void Frame::OptimizePose(OptimizerOnlyPose *optimizer_only_pose) {

  optimizer_only_pose->Clear();
  _Tcw = optimizer_only_pose->Optimize(
      _Tcw, GetMatchedKeyPoints(),
      GetMapPointsPositions(GetMatchedMapPointsRng()));
}

std::vector<MapPoint *>
Frame::GetMatchedMapPoints(const std::vector<cv::DMatch> &matches) const {
  const auto matched_indexes = boost::adaptors::transform(
      matches, [](const auto &match) { return match.queryIdx; });

  return FilterByIndex(GetMatchedMapPoints(), matched_indexes);
}

std::vector<cv::KeyPoint> Frame::GetMatchedKeyPoints() const {
  std::vector<cv::KeyPoint> matched_key_points;
  matched_key_points.reserve(_matched_map_point_to_idx.size());
  const auto &key_points = _orb_features.GetUndistortedKeyPoints();
  for (auto idx : _matched_map_point_to_idx | boost::adaptors::map_values) {
    matched_key_points.push_back(key_points[idx]);
  }
  return matched_key_points;
}

const KeyFrame &Frame::GetRefKeyFrame() const {
  return _map->GetKeyFrame(_ref_kf);
}

size_t Frame::GetRefKeyFrameNumKeyPoints() const {
  return GetRefKeyFrame().NumKeyPoints();
}
vertex_t Frame::GetRefKfVertex() const { return _ref_kf; }

std::set<vertex_t> Frame::GetKeyFramesShareSameMapPoints() const {
  std::set<vertex_t> key_frames_share_map_points; // K1 from orb_slam paper
  for (const auto map_point : GetMatchedMapPointsRng()) {
    const auto key_frames_observing_map_point = map_point->Observers();
    for (const auto key_frame : key_frames_observing_map_point) {
      key_frames_share_map_points.insert(
          key_frame->GetVertex()); // remove duplication, since many map_points
                                   // are observed from the same key frame
    }
  }
  return key_frames_share_map_points;
}

std::set<vertex_t> Frame::GetKeyFramesToTrackLocalMap() const {
  const auto key_frames_share_same_map_points =
      GetKeyFramesShareSameMapPoints();
  auto key_frames = key_frames_share_same_map_points;
  for (auto key_frame_share_same_map_points :
       key_frames_share_same_map_points) {
    const auto neighbors = _map->GetNeighbors(key_frame_share_same_map_points);
    boost::range::transform(neighbors,
                            std::inserter(key_frames, key_frames.end()),
                            [](vertex_t v) { return v; });
  }
  return key_frames;
}
boost::select_first_range<std::map<MapPoint *, size_t>>
Frame::GetMatchedMapPointsRng() const {
  return _matched_map_point_to_idx | boost::adaptors::map_keys;
}

std::vector<MapPoint *> Frame::GetMatchedMapPoints() const {
  auto rng = GetMatchedMapPointsRng();
  std::vector<MapPoint *> matched_map_points;
  matched_map_points.reserve(boost::size(rng));
  boost::copy(rng, std::back_inserter(matched_map_points));
  return matched_map_points;
}
const std::vector<cv::KeyPoint> &Frame::GetUndistortedKeyPoints() const {
  return _orb_features.GetUndistortedKeyPoints();
}
const OrbFeatures &Frame::GetOrbFeatures() const { return _orb_features; }

size_t Frame::GetNumMatchedMapPoints() const {
  return _matched_map_point_to_idx.size();
}
} // namespace clean_slam
