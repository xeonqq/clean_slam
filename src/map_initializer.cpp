//
// Created by root on 4/25/20.
//

#include "map_initializer.h"
#include "cv_algorithms.h"
#include "cv_utils.h"
#include "key_frame.h"
#include "viewer.h"

#include <third_party/spdlog/spdlog.h>

namespace clean_slam {

MapInitializer::MapInitializer(OrbExtractor *orb_extractor,
                               const OrbFeatureMatcher *orb_feature_matcher,
                               Optimizer *optimizer,
                               const cv::Mat &camera_intrinsic, Map *map,
                               std::vector<Frame> *frames, IViewer *viewer)
    : _orb_extractor(orb_extractor), _orb_feature_matcher{orb_feature_matcher},
      _optimizer(optimizer), _camera_motion_estimator{camera_intrinsic},
      _map{map}, _frames{frames}, _viewer{viewer} {}

void MapInitializer::ProcessFirstImage(const cv::Mat &image, double timestamp) {
  _previous_orb_features = _orb_extractor->DetectAndUndistortKeyPoints(image);
  _previous_timestamp = timestamp;
  _viewer->OnNotify(image, _previous_orb_features);
}

bool MapInitializer::InitializeCameraPose(const cv::Mat &image,
                                          double timestamp) {
  bool initialized{false};
  auto current_orb_features =
      _orb_extractor->DetectAndUndistortKeyPoints(image);
  const std::vector<cv::DMatch> good_matches =
      _orb_feature_matcher->Match(current_orb_features, _previous_orb_features);

  // 1st time filter by matched orb features
  const PointsPair matched_points_pair_undistorted =
      OrbFeatureMatcher::GetMatchedPointsPairUndistorted(
          current_orb_features, _previous_orb_features, good_matches);

  const auto &points_current_frame =
      matched_points_pair_undistorted.GetPointsCurrFrame();
  const auto &points_previous_frame =
      matched_points_pair_undistorted.GetPointsPrevFrame();
  const auto plausible_transformation = _camera_motion_estimator.Estimate(
      points_previous_frame, points_current_frame);
  if (plausible_transformation.IsGood()) {
    spdlog::info("Initialized");
    initialized = true;
    // same filter as 1st time, but getting key_points out
    auto key_points_pairs =
        OrbFeatureMatcher::GetMatchedKeyPointsPairUndistorted(
            current_orb_features, _previous_orb_features, good_matches);
    // 2nd time filter by good points in plausible transform
    auto good_key_points_prev_frame = FilterByMask(
        key_points_pairs.first, plausible_transformation.GetGoodPointsMask());
    auto good_key_points_curr_frame = FilterByMask(
        key_points_pairs.second, plausible_transformation.GetGoodPointsMask());
    KeyPointsPair good_key_points_pair = {
        std::move(good_key_points_prev_frame),
        std::move(good_key_points_curr_frame)};

    _optimizer->Clear();
    OptimizedResult optimized_result = _optimizer->Optimize(
        plausible_transformation.GetHomogeneousMatrix(), good_key_points_pair,
        plausible_transformation.GetGoodTriangulatedPoints());
    optimized_result.NormalizeBaseLine();

    const auto kf0 = _map->AddKeyFrame(
        g2o::SE3Quat(), _previous_orb_features.GetUndistortedKeyPoints(),
        _previous_orb_features.GetDescriptors());
    const auto kf1 =
        _map->AddKeyFrame(optimized_result.optimized_Tcw,
                          current_orb_features.GetUndistortedKeyPoints(),
                          current_orb_features.GetDescriptors());
    _map->AddKeyFramesWeight(kf0, kf1,
                             optimized_result.optimized_points.size());
    const auto kf0_matched_key_points_indexes =
        FilterByMask(TrainIdxs{}(good_matches),
                     plausible_transformation.GetGoodPointsMask());
    const auto kf1_matched_key_points_indexes =
        FilterByMask(QueryIdxs{}(good_matches),
                     plausible_transformation.GetGoodPointsMask());

    auto map_points = _map->AddMapPoints(optimized_result.optimized_points,
                                         kf0_matched_key_points_indexes, kf0,
                                         kf1_matched_key_points_indexes, kf1);

    _frames->emplace_back(_previous_orb_features, map_points,
                          kf0_matched_key_points_indexes, _map, g2o::SE3Quat(),
                          _previous_timestamp, kf0);
    _frames->emplace_back(current_orb_features, map_points,
                          kf1_matched_key_points_indexes, _map,
                          optimized_result.optimized_Tcw, timestamp, kf1);
#ifdef DEBUG
    cv::Mat out;
    std::vector<cv::DMatch> debug_good_matches;
    for (size_t i{0}; i < good_key_points_pair.first.size(); ++i) {
      debug_good_matches.emplace_back(i, i, 0);
    }
    cv::drawMatches(image, good_key_points_pair.first, _viewer->GetImage(),
                    good_key_points_pair.second, debug_good_matches, out);
    cv::imwrite("good_matches_after_finding_plausible_transform.png", out);
#endif

    _viewer->OnNotify({g2o::SE3Quat(), {}});
    _viewer->OnNotify(
        {optimized_result.optimized_Tcw, GetMapPointsPositions(map_points)});
  }

#ifdef DEBUG
  if (initialized) {
    cv::Mat out;
    cv::drawMatches(image, current_orb_features.GetUndistortedKeyPoints(),
                    _viewer->GetImage(),
                    _previous_orb_features.GetUndistortedKeyPoints(),
                    good_matches, out);
    cv::imwrite("good_matches_in_map_intialization.png", out);
  }
#endif
  _viewer->OnNotify(image, current_orb_features);
  _previous_orb_features = std::move(current_orb_features);
  _previous_timestamp = timestamp;
  return initialized;
}

} // namespace clean_slam
