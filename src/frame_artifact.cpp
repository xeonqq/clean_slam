//
// Created by root on 9/7/20.
//

#include "frame_artifact.h"
#include "cv_algorithms.h"
#include "cv_utils.h"
#include "frame.h"
#include <boost/range/algorithm_ext/insert.hpp>

namespace clean_slam {

std::vector<cv::KeyPoint> FrameArtifact::GetUnmatchedKeyPoints() const {
  return RemoveByIndex(_frame->GetOrbFeatures().GetUndistortedKeyPoints(),
                       _matched_key_points_idxs);
}
cv::Mat FrameArtifact::GetUnmatchedKeyPointsDescriptors() const {
  return RemoveByIndex(_frame->GetOrbFeatures().GetDescriptors(),
                       _matched_key_points_idxs);
}
Frame &FrameArtifact::GetFrame() { return *_frame; }
const Frame &FrameArtifact::GetFrame() const { return *_frame; }

size_t FrameArtifact::SearchUnmatchedKeyPointsByProjection(
    const OrbFeatureMatcher &matcher, const std::vector<MapPoint *> &map_points,
    const std::vector<Eigen::Vector2d> &projected_map_points,
    const std::vector<uint8_t> &map_points_octaves, int search_radius,
    const OctaveScales &octave_scales) {
  auto unmatched_orb_features =
      OrbFeatures(GetUnmatchedKeyPoints(), GetUnmatchedKeyPointsDescriptors());
  const auto matches_map_point_to_key_point = clean_slam::SearchByProjection(
      matcher, projected_map_points, GetMapPointsDescriptors(map_points),
      map_points_octaves, unmatched_orb_features, cv::Mat{}, search_radius,
      octave_scales, _frame->kNumNearestNeighbor,
      _frame->kDescriptorDistanceThreshold);

  const auto newly_matched_key_points =
      FilterByIndex(unmatched_orb_features.GetKeyPoints(),
                    TrainIdxs{}(matches_map_point_to_key_point));

  boost::range::insert(_frame->_matched_key_points,
                       _frame->_matched_key_points.end(),
                       newly_matched_key_points);
  boost::range::insert(
      _frame->_matched_map_points, _frame->_matched_map_points.end(),
      FilterByIndex(map_points, QueryIdxs{}(matches_map_point_to_key_point)));
  return matches_map_point_to_key_point.size();
}
} // namespace clean_slam
