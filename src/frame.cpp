//
// Created by root on 5/12/20.
//
#include "frame.h"
#include "cv_utils.h"
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/transform.hpp>

namespace clean_slam {

cv::Mat Frame::GetMatchedMapPointsDescriptors() const {
  return GetMapPointsDescriptors(_matched_map_points);
}

std::vector<uint8_t> Frame::GetMatchedMapPointsOctaves() const {
  std::vector<uint8_t> octaves;
  octaves.reserve(_matched_key_points.size());
  boost::range::transform(
      _matched_key_points, std::back_inserter(octaves),
      [](const cv::KeyPoint &key_point) { return key_point.octave; });
  return octaves;
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

FrameArtifact Frame::SearchByProjection(
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

  auto matched_key_points_indexes = TrainIdxs{}(matches_map_point_to_key_point);
  _matched_key_points = FilterByIndex(_orb_features.GetUndistortedKeyPoints(),
                                      matched_key_points_indexes);
  _matched_map_points =
      prev_frame.GetMatchedMapPoints(matches_map_point_to_key_point);

  return FrameArtifact(this, std::move(matched_key_points_indexes),
                       std::move(matches_map_point_to_key_point));
}

void Frame::OptimizePose(OptimizerOnlyPose *optimizer_only_pose) {

  optimizer_only_pose->Clear();
  _Tcw = optimizer_only_pose->Optimize(
      _Tcw, _matched_key_points, GetMapPointsPositions(_matched_map_points));
}

std::vector<MapPoint *>
Frame::GetMatchedMapPoints(const std::vector<cv::DMatch> &matches) const {
  const auto matched_indexes = boost::adaptors::transform(
      matches, [](const auto &match) { return match.queryIdx; });

  return FilterByIndex(_matched_map_points, matched_indexes);
}

const std::vector<cv::KeyPoint> &Frame::GetMatchedKeyPoints() const {
  return _matched_key_points;
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
  for (const auto map_point : _matched_map_points) {
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
const std::vector<MapPoint *> &Frame::GetMatchedMapPoints() const {
  return _matched_map_points;
}
const std::vector<cv::KeyPoint> &Frame::GetUndistortedKeyPoints() const {
  return _orb_features.GetUndistortedKeyPoints();
}
const OrbFeatures &Frame::GetOrbFeatures() const { return _orb_features; }

size_t Frame::GetNumMatchedMapPoints() const {
  return _matched_map_points.size();
}
} // namespace clean_slam
