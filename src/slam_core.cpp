//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include "cv_algorithms.h"
#include "cv_utils.h"
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <third_party/spdlog/spdlog.h>

namespace clean_slam {

SlamCore::SlamCore(const cv::Mat &camera_intrinsics,
                   const cv::Mat &camera_distortion_coeffs,
                   OrbExtractor *orb_extractor, Optimizer *optimizer,
                   Viewer *viewer, const OctaveScales &octave_scale)
    : _camera_intrinsic(camera_intrinsics),
      _orb_extractor(orb_extractor), _optimizer{optimizer},
      _viewer(viewer), _camera_motion_estimator{camera_intrinsics},
      _undistorted_image_boundary{camera_intrinsics, camera_distortion_coeffs},
      _octave_scales{octave_scale} {}

void SlamCore::ProcessFirstImage(const cv::Mat &image, double timestamp) {
  _undistorted_image_boundary.ComputeUndistortedCorners(image);
  _previous_frame = {image, _orb_extractor->DetectAndUndistortKeyPoints(image)};

  HomogeneousMatrix homogeneous_matrix;
  _trajectory.emplace_back(homogeneous_matrix, timestamp);

  _viewer->OnNotify(Content{homogeneous_matrix, {}, _previous_frame});
}

bool SlamCore::InitializeCameraPose(const cv::Mat &image, double timestamp) {
  Frame current_frame{image,
                      _orb_extractor->DetectAndUndistortKeyPoints(image)};
  HomogeneousMatrix homogeneous_matrix;
  std::vector<Eigen::Vector3d> good_triangulated_points;
  bool initialized = false;
  const std::vector<cv::DMatch> good_matches =
      _orb_feature_matcher.Match(current_frame, _previous_frame);
  const PointsPair matched_points_pair_undistorted =
      OrbFeatureMatcher::GetMatchedPointsPairUndistorted(
          current_frame, _previous_frame, good_matches);

  const auto &points_current_frame =
      matched_points_pair_undistorted.GetPointsCurrFrame();
  const auto &points_previous_frame =
      matched_points_pair_undistorted.GetPointsPrevFrame();
  const auto plausible_transformation = _camera_motion_estimator.Estimate(
      points_previous_frame, points_current_frame);
  if (plausible_transformation.IsGood()) {
    spdlog::info("Initialized");
    initialized = true;
    const auto key_points_pairs =
        OrbFeatureMatcher::GetMatchedKeyPointsPairUndistorted(
            current_frame, _previous_frame, good_matches);
    const auto good_key_points_prev_frame = FilterByMask(
        key_points_pairs.first, plausible_transformation.GetGoodPointsMask());
    const auto good_key_points_curr_frame = FilterByMask(
        key_points_pairs.second, plausible_transformation.GetGoodPointsMask());
    KeyPointsPair good_key_points_pair = {
        std::move(good_key_points_prev_frame),
        std::move(good_key_points_curr_frame)};

    OptimizedResult optimized_result = _optimizer->Optimize(
        plausible_transformation.GetHomogeneousMatrix(), good_key_points_pair,
        plausible_transformation.GetGoodTriangulatedPoints());
    optimized_result.NormalizeBaseLine();
    _velocity = GetVelocity(optimized_result.optimized_Tcw, g2o::SE3Quat{});
    //      DrawGoodMatches(current_frame, good_matches);
    homogeneous_matrix = optimized_result.optimized_Tcw;

    auto matched_descriptors = OrbFeatureMatcher::GetMatchedDescriptors(
        current_frame, _previous_frame, good_matches);
    const auto good_descriptors_current_frame =
        FilterByMask(matched_descriptors.second,
                     plausible_transformation.GetGoodPointsMask());
    _key_frames.push_back(
        KeyFrame::Create(optimized_result.optimized_Tcw,
                         std::move(optimized_result.optimized_points),
                         good_descriptors_current_frame,
                         key_points_pairs.second, _octave_scales));
    _reference_key_frame = &_key_frames.back();
  }

  _trajectory.emplace_back(homogeneous_matrix, timestamp);

  if (_viewer)
    _viewer->OnNotify(
        Content{homogeneous_matrix, good_triangulated_points, current_frame});
  //    cv::Mat out_im;
  //    cv::drawKeypoints(image, current_frame.GetKeyPoints(), out_im);
  //    cv::imshow("image", out_im);
  //    cv::waitKey(0);

  _previous_frame = std::move(current_frame);
  return initialized;
}

void SlamCore::TrackByMotionModel(const cv::Mat &image, double timestamp) {
  Frame current_frame{image,
                      _orb_extractor->DetectAndUndistortKeyPoints(image)};
  // const velocity model
  const auto &homogeneous_matrix = _trajectory.back();
  const auto Tcw = _velocity * homogeneous_matrix.GetTransformation();
  const auto camera_pose_in_world = Tcw.inverse();
  // todo: project 3d points (along with its feature descriptor) to current
  const auto &points_3d = _reference_key_frame->GetPoints3D();
  std::vector<Eigen::Vector2d> points_reprojected =
      ReprojectPoints3d(points_3d, Tcw, _camera_intrinsic);
  std::vector<bool> mask(points_reprojected.size(), false);

  const auto &x_bounds = _undistorted_image_boundary.GetXBounds();
  const auto &y_bounds = _undistorted_image_boundary.GetYBounds();

  const auto &distance_bounds = _reference_key_frame->GetDistanceBounds();

  for (size_t i = 0; i < points_reprojected.size(); ++i) {
    const auto &point = points_reprojected[i];

    //  1. check points in undistorted range
    if (!IsPointWithInBounds(point, x_bounds, y_bounds))
      continue;

    //  2. check new depth of points (wrt to new camera pose) is within
    //  scale pyramid range
    const auto depth =
        (points_3d[i] - camera_pose_in_world.translation()).norm();
    if (!distance_bounds[i].IsWithIn(depth))
      continue;

    int predicted_octave_level =
        _octave_scales.MapDistanceToOctaveLevel(depth, distance_bounds[i]);

    mask[i] = true;
  }
  SearchByProjection(current_frame.GetKeyPoints(), points_reprojected,
                     _reference_key_frame->GetDescriptors(), mask);

  if (_viewer)
    _viewer->OnNotify(Content{Tcw, {}, current_frame});
}

void SlamCore::DrawGoodMatches(
    const Frame &current_frame,
    const std::vector<cv::DMatch> &good_matches) const {
  cv::Mat img_matches;
  cv::drawMatches(
      current_frame.GetImage(), current_frame.GetKeyPoints(),
      this->_previous_frame.GetImage(), this->_previous_frame.GetKeyPoints(),
      good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
      std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  cv::imshow("Good Matches & Object detection", img_matches);
  cv::waitKey(0);
}

const CameraTrajectory &SlamCore::GetTrajectory() const { return _trajectory; }

} // namespace clean_slam
