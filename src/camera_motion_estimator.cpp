//
// Created by root on 1/20/20.
//

#include "camera_motion_estimator.h"
#include <chrono>
#include <future>
#include <iostream>
namespace clean_slam {

using namespace std::chrono;
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
  std::cerr << " run time: "
            << duration_cast<microseconds>(stop - start).count() << '\n';
  HomogeneousMatrix homogeneous_matrix;
  if (homography_transformation.GetReprojectionError() <
      epipolar_transformation.GetReprojectionError()) {
    homogeneous_matrix =
        homography_transformation.EstimateMotion(_camera_intrinsic);
  } else {
    homogeneous_matrix =
        epipolar_transformation.EstimateMotion(_camera_intrinsic);
  }
  return homogeneous_matrix;
}

CameraMotionEstimator::CameraMotionEstimator(const cv::Mat &camera_intrinsic)
    : _camera_intrinsic(camera_intrinsic) {}

const float IProjectiveTransformation::GetReprojectionError() const {
  return _reprojection_error;
}

IProjectiveTransformation::IProjectiveTransformation(
    const cv::Mat &m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : _m(m), _points_previous_frame(points_previous_frame),
      _points_current_frame(points_current_frame), _inlier(inlier),
      _reprojection_error(reprojection_error) {}

HomogeneousMatrix
HomographyTransformation::EstimateMotion(const cv::Mat &camera_intrinsics) {
  return HomogeneousMatrix(cv::Mat(), cv::Mat());
}

HomographyTransformation::HomographyTransformation(
    const cv::Mat &m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : IProjectiveTransformation(m, points_previous_frame, points_current_frame,
                                inlier, reprojection_error) {}
HomogeneousMatrix
EpipolarTransformation::EstimateMotion(const cv::Mat &camera_intrinsics) {
  return HomogeneousMatrix(cv::Mat(), cv::Mat());
}

EpipolarTransformation::EpipolarTransformation(
    const cv::Mat &m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : IProjectiveTransformation(m, points_previous_frame, points_current_frame,
                                inlier, reprojection_error) {}
} // namespace clean_slam
