//
// Created by root on 12/29/19.
//

#include "slam_core.h"
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
                   OrbExtractor *orb_extractor, Viewer *viewer)
    : _camera_intrinsic(camera_intrinsics), _orb_extractor(orb_extractor),
      _viewer(viewer), _initializer{camera_intrinsics},
      _optimizer{camera_intrinsics} {}

bool SlamCore::InitializeCameraPose(const cv::Mat &image, double timestamp) {
  Frame current_frame{image,
                      _orb_extractor->DetectAndUndistortKeyPoints(image)};
  HomogeneousMatrix homogeneous_matrix;
  std::vector<Eigen::Vector3d> good_triangulated_points;
  bool initialized = false;
  if (!_previous_frame.GetImage().empty()) {
    const std::vector<cv::DMatch> good_matches =
        _orb_feature_matcher.Match(current_frame, _previous_frame);
    const PointsPair matched_points_pair_undistorted =
        OrbFeatureMatcher::GetMatchedPointsPairUndistorted(
            current_frame, _previous_frame, good_matches);

    const auto &points_current_frame =
        matched_points_pair_undistorted.GetPointsCurrFrame();
    const auto &points_previous_frame =
        matched_points_pair_undistorted.GetPointsPrevFrame();

    if (_initializer.Initialize(points_previous_frame, points_current_frame)) {
      spdlog::info("Initialized");
      initialized = true;
      const auto plausible_transformation =
          _initializer.GetPlausibleTransformation();

      const auto key_points_pairs =
          OrbFeatureMatcher::GetMatchedKeyPointsPairUndistorted(
              current_frame, _previous_frame, good_matches);
      const auto good_key_points_prev_frame = FilterByMask(
          key_points_pairs.first, plausible_transformation.GetGoodPointsMask());
      const auto good_key_points_curr_frame =
          FilterByMask(key_points_pairs.second,
                       plausible_transformation.GetGoodPointsMask());
      KeyPointsPair good_key_points_pair = {
          std::move(good_key_points_prev_frame),
          std::move(good_key_points_curr_frame)};

      OptimizedResult optimized_result = _optimizer.Optimize(
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
      _key_frames.emplace_back(optimized_result.optimized_Tcw,
                               std::move(optimized_result.optimized_points),
                               good_descriptors_current_frame);
      _reference_key_frame = &_key_frames.back();
    }

  } else {
    homogeneous_matrix = HomogeneousMatrix{};
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
  const auto current_pose = _velocity * _trajectory.back().GetTransformation();
  // todo: project 3d points (along with its feature descriptor) to current
  const auto points_3d = _reference_key_frame->GetPoints3D();
  std::vector<Eigen::Vector2d> points_reprojected =
      ReprojectPoints3d(points_3d, current_pose, _camera_intrinsic);
  const int h = image.rows;
  const int w = image.cols;
  std::vector<bool> mask(points_reprojected.size(), false);

  for (size_t i = 0; i < points_reprojected.size(); ++i) {
    const auto &point = points_reprojected[i];
    // todo: 1. check points in undistorted range
    //      2. check new depth of points (wrt to new camera pose) is within
    //      scale pyramid range

    mask[i] =
        (point[0] < w) && (point[1] < h) && (point[0] > 0) && (point[1] > 0);
  }

  //  projection_matrix = current_pose;
  //  cv::Mat reprojected_image_points;
  //  cv::transform(points_3d, reprojected_image_points, projection_matrix);
  //  cv::convertPointsFromHomogeneous(reprojected_image_points.t(),
  //                                   reprojected_image_points);

  if (_viewer)
    _viewer->OnNotify(Content{current_pose, {}, current_frame});
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
