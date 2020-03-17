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
  std::cout << "homography reproj err: "
            << homography_transformation.GetReprojectionError() << std::endl;
  std::cout << "epipolar reproj err: "
            << epipolar_transformation.GetReprojectionError() << std::endl;
  //  if (homography_transformation.GetReprojectionError() <
  //      epipolar_transformation.GetReprojectionError()) {
  if (true) {
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
    const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : _m(m), _points_previous_frame(points_previous_frame),
      _points_current_frame(points_current_frame), _inlier(inlier),
      _reprojection_error(reprojection_error) {}

HomogeneousMatrix
HomographyTransformation::EstimateMotion(const cv::Mat &camera_intrinsics) {
  std::vector<cv::Mat> Rs, Ts, Normals;
  cv::decomposeHomographyMat(_m, camera_intrinsics, Rs, Ts, Normals);

  cv::Mat projection_matrix0 =
      camera_intrinsics * cv::Mat::eye(3, 4, camera_intrinsics.type());

  HomogeneousMatrix homogeneous_matrix;
  for (size_t i = 0; i < Rs.size(); ++i) {
    cv::Mat projection_matrix1(3, 4, projection_matrix0.type());
    projection_matrix1(cv::Range::all(), cv::Range(0, 3)) = Rs[i];
    projection_matrix1.col(3) = Ts[i];
    projection_matrix1 = camera_intrinsics * projection_matrix1;

    cv::Mat points_3d_homo;
    cv::triangulatePoints(projection_matrix0, projection_matrix1,
                          _points_previous_frame, _points_current_frame,
                          points_3d_homo);
    cv::Mat points_3d_cartisian;
    cv::convertPointsFromHomogeneous(points_3d_homo.t(), points_3d_cartisian);

    points_3d_cartisian = points_3d_cartisian.reshape(1);
    cv::Mat mask_depth_not_positive =
        (points_3d_cartisian.col(2) <= 0) & _inlier;
    int number_of_non_positive_depth =
        cv::countNonZero(mask_depth_not_positive);
    if (number_of_non_positive_depth == 0) {
      homogeneous_matrix = CreateHomogeneousMatrix(Rs[i], Ts[i]);
      break;
    }
  }
  return homogeneous_matrix;
}

HomographyTransformation::HomographyTransformation(
    const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : IProjectiveTransformation(m, points_previous_frame, points_current_frame,
                                inlier, reprojection_error) {}

HomogeneousMatrix
EpipolarTransformation::EstimateMotion(const cv::Mat &camera_intrinsics) {
  cv::Mat r, t;
  cv::Mat essential_mat = camera_intrinsics.t() * _m * camera_intrinsics;
  cv::recoverPose(essential_mat, _points_previous_frame, _points_current_frame,
                  camera_intrinsics, r, t, _inlier);
  return CreateHomogeneousMatrix(r, t);
}

EpipolarTransformation::EpipolarTransformation(
    const cv::Mat m, const std::vector<cv::Point2f> &points_previous_frame,
    const std::vector<cv::Point2f> &points_current_frame, const cv::Mat &inlier,
    const float reprojection_error)
    : IProjectiveTransformation(m, points_previous_frame, points_current_frame,
                                inlier, reprojection_error) {}
} // namespace clean_slam
