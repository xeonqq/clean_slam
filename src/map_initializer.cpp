//
// Created by root on 4/25/20.
//

#include "map_initializer.h"
#include "cv_algorithms.h"
#include "cv_utils.h"
#include "key_frame.h"
#include "viewer.h"

#include <boost/range/algorithm_ext/iota.hpp>
#include <third_party/spdlog/spdlog.h>

namespace clean_slam {

MapInitializer::MapInitializer(OrbExtractor *orb_extractor,
                               const OrbFeatureMatcher *orb_feature_matcher,
                               Optimizer *optimizer,
                               const cv::Mat &camera_intrinsic, Map *map,
                               const OctaveScales &octave_scales,
                               Viewer *viewer)
    : _orb_extractor(orb_extractor), _orb_feature_matcher{orb_feature_matcher},
      _optimizer(optimizer), _camera_motion_estimator{camera_intrinsic},
      _map{map}, _octave_scales{octave_scales}, _viewer{viewer} {}

void MapInitializer::ProcessFirstImage(const cv::Mat &image, double timestamp) {
  _previous_orb_features = _orb_extractor->DetectAndUndistortKeyPoints(image);
  _previous_timestamp = timestamp;
  _viewer->OnNotify(image, _previous_orb_features);
}

boost::optional<std::pair<Frame, Frame>>
MapInitializer::InitializeCameraPose(const cv::Mat &image, double timestamp) {
  boost::optional<std::pair<Frame, Frame>> frame;
  auto current_orb_features =
      _orb_extractor->DetectAndUndistortKeyPoints(image);
  std::vector<Eigen::Vector3d> good_triangulated_points;
  const std::vector<cv::DMatch> good_matches =
      _orb_feature_matcher->Match(current_orb_features, _previous_orb_features);
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
    auto key_points_pairs =
        OrbFeatureMatcher::GetMatchedKeyPointsPairUndistorted(
            current_orb_features, _previous_orb_features, good_matches);
    const auto good_key_points_prev_frame = FilterByMask(
        key_points_pairs.first, plausible_transformation.GetGoodPointsMask());
    const auto good_key_points_curr_frame = FilterByMask(
        key_points_pairs.second, plausible_transformation.GetGoodPointsMask());
    KeyPointsPair good_key_points_pair = {
        std::move(good_key_points_prev_frame),
        std::move(good_key_points_curr_frame)};

    _optimizer->Clear();
    OptimizedResult optimized_result = _optimizer->Optimize(
        plausible_transformation.GetHomogeneousMatrix(), good_key_points_pair,
        plausible_transformation.GetGoodTriangulatedPoints());
    optimized_result.NormalizeBaseLine();

    auto matched_descriptors = OrbFeatureMatcher::GetMatchedDescriptors(
        current_orb_features, _previous_orb_features, good_matches);

    const auto good_descriptors_current_frame =
        FilterByMask(matched_descriptors.second,
                     plausible_transformation.GetGoodPointsMask());

    std::vector<size_t> map_point_indexes;
    map_point_indexes.resize(optimized_result.optimized_points.size());
    boost::range::iota(map_point_indexes, 0);

    _map->Construct(optimized_result.optimized_Tcw,
                    std::move(optimized_result.optimized_points),
                    good_descriptors_current_frame, good_key_points_pair.second,
                    _octave_scales);
    _map->AddMapPoints(optimized_result.optimized_Tcw,
                       optimized_result.optimized_points,
                       good_descriptors_current_frame,
                       good_key_points_pair.second, _octave_scales);
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
    frame = std::make_pair(Frame{std::move(good_key_points_pair.first),
                                 map_point_indexes, _map, g2o::SE3Quat(),
                                 _previous_timestamp},
                           Frame{std::move(good_key_points_pair.second),
                                 std::move(map_point_indexes), _map,
                                 optimized_result.optimized_Tcw, timestamp});

    auto first_key_frame = KeyFrame::Create(
        g2o::SE3Quat(), _previous_orb_features.GetUndistortedKeyPoints(),
        _previous_orb_features.GetDescriptors(),
        optimized_result.optimized_points);

    _viewer->OnNotify({g2o::SE3Quat(), {}});
    _viewer->OnNotify({optimized_result.optimized_Tcw, _map->GetPoints3D()});
  }

#ifdef DEBUG
  if (frame) {
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
  return frame;
}

} // namespace clean_slam
