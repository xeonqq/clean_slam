//
// Created by root on 1/20/20.
//
#include <chrono>
#include <future>
#include <iostream>

#include "camera_motion_estimator.h"
#include "cv_utils.h"
#include "eigen_utils.h"
#include <cv.hpp>
#include <opencv2/core/eigen.hpp>
#include <third_party/spdlog/spdlog.h>

namespace clean_slam {

using namespace std::chrono;

CameraMotionEstimator::CameraMotionEstimator(const cv::Mat &camera_intrinsic)
    : _camera_intrinsic(camera_intrinsic) {}

PlausibleTransformation CameraMotionEstimator::Estimate(
    const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame) {

  auto start = high_resolution_clock::now();
  auto future = std::async(std::launch::async, [&]() {
    return _epipolar_constraint_motion_estimator
        .EstimateProjectiveTransformation(points_previous_frame,
                                          points_current_frame);
  });
  auto homography_transformation =
      _homography_motion_estimator.EstimateProjectiveTransformation(
          points_previous_frame, points_current_frame);
  auto epipolar_transformation = future.get();
  auto stop = high_resolution_clock::now();
  spdlog::info("H + F total runtime: {}",
               duration_cast<microseconds>(stop - start).count());
  if (IsHomography(homography_transformation, epipolar_transformation)) {
    //  if (false) {
    spdlog::info("Choose H");
    return homography_transformation.EstimateMotion(_camera_intrinsic);
  } else {
    spdlog::info("Choose F");
    return epipolar_transformation.EstimateMotion(_camera_intrinsic);
  }
}

// https://en.wikipedia.org/wiki/Chi-squared_distribution
float ScoreFromChiSquareDistribution(const cv::Mat &transfer_errors,
                                     const cv::Mat &inlies, float basic_score,
                                     float chi_square_threshold) {
  cv::Mat scores = basic_score - transfer_errors;

  cv::Mat mask = (transfer_errors < chi_square_threshold) & inlies;
  cv::Mat new_scores;
  scores.copyTo(new_scores, mask);
  return cv::sum(new_scores)[0];
}

bool IsHomography(const HomographyTransformation &homography_transformation,
                  const EpipolarTransformation &epipolar_transformation) {
  const float homograhy_threshold = 0.45f;
  return homography_transformation.GetReprojectionError() /
             (homography_transformation.GetReprojectionError() +
              epipolar_transformation.GetReprojectionError()) >
         homograhy_threshold;
}
} // namespace clean_slam
