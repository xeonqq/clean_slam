//
// Created by root on 5/12/20.
//
#include "frame.h"
#include "cv_utils.h"
#include "epipolar_constraint_motion_estimator.h"
#include "homography_motion_estimator.h"
#include "map.h"
#include "match_map.h"
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <opencv2/core/eigen.hpp>

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
  _matched_map_point_to_idx.clear();
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
  auto unmatched_orb_features = GetUnmatchedFeatures();
  const auto matches_map_point_to_key_point = clean_slam::SearchByProjection(
      matcher, projected_map_points, GetMapPointsDescriptors(map_points),
      map_points_octaves, unmatched_orb_features, cv::Mat{}, search_radius,
      octave_scales, kNumNearestNeighbor, kDescriptorDistanceThreshold);

  const auto unmatched_key_points_idxes = GetUnmatchedIndexes();
  for (const auto &match : matches_map_point_to_key_point) {
    _matched_map_point_to_idx.emplace(
        map_points[match.queryIdx], unmatched_key_points_idxes[match.trainIdx]);
  }

  return matches_map_point_to_key_point.size();
}

std::vector<size_t> Frame::GetUnmatchedIndexes() const {
  return clean_slam::GetUnmatchedIndexes(GetUndistortedKeyPoints().size(),
                                         _matched_map_point_to_idx |
                                             boost::adaptors::map_values);
}

OrbFeatures Frame::GetUnmatchedFeatures() const {
  return OrbFeatures(GetUnmatchedKeyPoints(),
                     GetUnmatchedKeyPointsDescriptors());
}

TriangulationResult
Frame::MatchUnmatchedKeyPoints(const OrbFeatureMatcher &matcher, Frame &frame,
                               const cv::Mat &camera_intrinsics) {
  const auto unmatched_features_query = GetUnmatchedFeatures();
  const auto unmatched_features_train = frame.GetUnmatchedFeatures();
  const auto matches_for_key_points =
      matcher.KnnMatch(unmatched_features_query.GetDescriptors(),
                       unmatched_features_train.GetDescriptors(), 1);
  const auto unmatched_query_key_points_idxes = GetUnmatchedIndexes();
  const auto unmatched_train_key_points_idxes = frame.GetUnmatchedIndexes();
  MatchMap map;
  // for each query feature, so for each unmatched feature in current frame
  for (const auto &matches_for_key_point : matches_for_key_points) {
    if (!matches_for_key_point.empty()) {
      const auto &match = matches_for_key_point[0];
      if (match.distance <= kDescriptorDistanceThreshold) {
        map.Emplace(match);
      }
    }
  }
  const auto &key_points_query = unmatched_features_query.GetKeyPoints();
  const auto &key_points_train = unmatched_features_train.GetKeyPoints();
  std::vector<cv::Point2f> good_points_query;
  std::vector<cv::Point2f> good_points_train;
  std::vector<int> good_points_query_idxes;
  std::vector<int> good_points_train_idxes;
  Eigen::Matrix3d K;
  cv::cv2eigen(camera_intrinsics, K);
  const auto F = GetFundamentalMatrix(_Tcw, frame.GetTcw(), K);
  for (const auto &match :
       map.GetTrainIdxToMatch() | boost::adaptors::map_values) {
    const auto &key_point_query = key_points_query[match.queryIdx];
    const auto &key_point_train = key_points_train[match.trainIdx];
    const auto distance_sqr =
        DistanceSqrToEpipolarLine(key_point_query.pt, F, key_point_train.pt);
    // todo: the threshold for distance to epipolar line is too high
    if (distance_sqr <
        EpipolarConstraintMotionEstimator::chi_square_threshold * 30) {
      good_points_query.push_back(key_point_query.pt);
      good_points_train.push_back(key_point_train.pt);
      good_points_query_idxes.push_back(
          unmatched_query_key_points_idxes[match.queryIdx]);
      good_points_train_idxes.push_back(
          unmatched_train_key_points_idxes[match.trainIdx]);
    }
  }
  if (good_points_query.empty()) {
    return {};
  }
  cv::Mat points_3d_cartisian = TriangulatePoints(
      _Tcw, good_points_query, frame.GetTcw(), good_points_train, K);
  cv::Mat mask = cv::Mat(points_3d_cartisian.rows, 1, CV_8U, 1U);
  ValidateMapPointsPositiveDepth(points_3d_cartisian, _Tcw, mask);
  ValidateMapPointsPositiveDepth(points_3d_cartisian, frame.GetTcw(), mask);
  const auto reproj_err_query = Calculate3DPointsReprojectionError(
      points_3d_cartisian, camera_intrinsics * ToTransformationMatrix(_Tcw),
      good_points_query);
  const auto reproj_err_train = Calculate3DPointsReprojectionError(
      points_3d_cartisian,
      camera_intrinsics * ToTransformationMatrix(frame.GetTcw()),
      good_points_train);
  mask = (reproj_err_query < HomographyMotionEstimator::chi_square_threshold) &
         mask;
  mask = (reproj_err_train < HomographyMotionEstimator::chi_square_threshold) &
         mask;

  return {ToStdVectorByMask(points_3d_cartisian.reshape(1), mask),
          FilterByMask(good_points_query_idxes, mask),
          FilterByMask(good_points_train_idxes, mask)};
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
  return GetRefKeyFrame().GetNumKeyPoints();
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

size_t Frame::GetNumKeyPoints() const { return _orb_features.NumKeyPoints(); }
const cv::Mat &Frame::GetDescriptors() const {
  return _orb_features.GetDescriptors();
}

const OrbFeatures &Frame::GetOrbFeatures() const { return _orb_features; }

size_t Frame::GetNumMatchedMapPoints() const {
  return _matched_map_point_to_idx.size();
}

cv::Mat Frame::GetMatchedKeyPointDescriptor(MapPoint *map_point) const {
  return GetDescriptors().row(_matched_map_point_to_idx.at(map_point));
}
} // namespace clean_slam
