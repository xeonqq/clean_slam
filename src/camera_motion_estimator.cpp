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

namespace clean_slam {

using namespace std::chrono;

CameraMotionEstimator::CameraMotionEstimator(const cv::Mat &camera_intrinsic)
    : _camera_intrinsic(camera_intrinsic) {}

HomogeneousMatrix CameraMotionEstimator::Estimate(
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
  std::cerr << " total runtime: "
            << duration_cast<microseconds>(stop - start).count() << '\n';
  if (IsHomography(homography_transformation, epipolar_transformation)) {
    //      if (false) {
    std::cerr << "Choose H\n";
    return homography_transformation.EstimateMotion(_camera_intrinsic);
  } else {
    std::cerr << "Choose F\n";
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
