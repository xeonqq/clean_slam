//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include "cv_utils.h"
#include "transfer_error.h"
#include <chrono>
#include <iterator>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <third_party/spdlog/spdlog.h>

namespace clean_slam {

SlamCore::SlamCore(Viewer *viewer) : _viewer(viewer) {}

void SlamCore::Track(const cv::Mat &image, double timestamp) {

  Frame current_frame{image, _orb_extractor.DetectAndUndistortKeyPoints(image)};
  HomogeneousMatrix homogeneous_matrix;
  cv::Mat good_triangulated_points;
  if (!_previous_frame.GetImage().empty()) {
    const auto good_matches =
        _orb_feature_matcher.Match(current_frame, _previous_frame);
    const auto matched_points_pair_undistorted =
        OrbFeatureMatcher::GetMatchedPointsPairUndistorted(
            current_frame, _previous_frame, good_matches);

    const auto &points_current_frame =
        matched_points_pair_undistorted.GetPointsCurrFrame();
    const auto &points_previous_frame =
        matched_points_pair_undistorted.GetPointsPrevFrame();
    if (_initializer.Initialize(points_previous_frame, points_current_frame)) {
      spdlog::info("Initialized");
      homogeneous_matrix = _initializer.GetHomogeneousMatrix();
      good_triangulated_points = _initializer.GetGoodTriangulatedPoints();
      //      DrawGoodMatches(image, current_frame, good_matches);
    }

    //    _trajectory.emplace_back(homogeneous_mat, timestamp);
  } else {
    homogeneous_matrix = HomogeneousMatrix{};
  }
  _trajectory.emplace_back(homogeneous_matrix, timestamp);

  if (_viewer)
    _viewer->OnNotify(Content{homogeneous_matrix, good_triangulated_points});
  //  cv::Mat out_im;
  //  cv::drawKeypoints(image, current_frame.GetKeyPoints(), out_im);
  //  cv::imshow("image", out_im);
  //  cv::waitKey(0);

  _previous_frame = std::move(current_frame);
}

void SlamCore::DrawGoodMatches(
    const cv::Mat &image, const Frame &current_frame,
    const std::vector<cv::DMatch> &good_matches) const {
  cv::Mat img_matches;
  cv::drawMatches(
      image, current_frame.GetKeyPoints(), this->_previous_frame.GetImage(),
      this->_previous_frame.GetKeyPoints(), good_matches, img_matches,
      cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
      cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  cv::imshow("Good Matches & Object detection", img_matches);
  cv::waitKey(0);
}

void SlamCore::Initialize(const cv::Mat &camera_intrinsics,
                          const cv::Mat &camera_distortion_coeffs) {
  _orb_extractor.SetCameraIntrinsicsAndDistortionCoeffs(
      camera_intrinsics, camera_distortion_coeffs);
  _camera_intrinsic = camera_intrinsics;
  //  _camera_distortion_coeffs = camera_distortion_coeffs;
}
const CameraTrajectory &SlamCore::GetTrajectory() const { return _trajectory; }

} // namespace clean_slam
