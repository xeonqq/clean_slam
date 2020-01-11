//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include "cv_utils.h"
#include <iterator>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace clean_slam {

void SlamCore::Track(const cv::Mat image) {

  Frame current_frame{image, _orb_extractor.DetectAndUndistortKeyPoints(image)};

  if (!_previous_frame.GetImage().empty()) {
    const auto good_matches =
        _orb_feature_matcher.Match(current_frame, _previous_frame);
    const auto matched_points_pair_undistorted =
        OrbFeatureMatcher::GetMatchedPointsPairUndistorted(
            current_frame, _previous_frame, good_matches);
    cv::Mat homography_inlies;
    const auto &points_current_frame =
        matched_points_pair_undistorted.GetPointsCurrFrame();
    const auto &points_previous_frame =
        matched_points_pair_undistorted.GetPointsPrevFrame();
    cv::Mat H = cv::findHomography(points_previous_frame, points_current_frame,
                                   homography_inlies, CV_RANSAC);
    const auto homography_average_symmetric_transfer_error =
        CalculateSymmetricTransferError(
            points_previous_frame, points_current_frame, H, homography_inlies) /
        cv::countNonZero(homography_inlies);

    std::cout << homography_average_symmetric_transfer_error << std::endl;
    cv::Mat fundamental_inlies;
    cv::Mat F = cv::findFundamentalMat(
        points_previous_frame, points_current_frame, fundamental_inlies);
    //    std::cout << "Homography Mat:\n" << H << std::endl;
    //    std::cout << "Fundemental Mat:\n" << F << std::endl;
    cv::Mat img_matches;
    cv::drawMatches(image, current_frame.GetKeyPoints(),
                    _previous_frame.GetImage(), _previous_frame.GetKeyPoints(),
                    good_matches, img_matches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    imshow("Good Matches & Object detection", img_matches);
  }

  cv::Mat out_im;
  cv::drawKeypoints(image, current_frame.GetKeyPoints(), out_im);
  cv::imshow("image", out_im);
  cv::waitKey(0);

  _previous_frame = std::move(current_frame);
}

float CalculateSymmetricTransferError(
    const std::vector<cv::Point2f> &src_points,
    const std::vector<cv::Point2f> &dst_points, const cv::Mat &m,
    const cv::Mat &inlies_mask) {
  const auto forward_transfer_error =
      CalculateTransferError(src_points, dst_points, m, inlies_mask);
  const auto backward_transfer_error =
      CalculateTransferError(dst_points, src_points, m.inv(), inlies_mask);
  return forward_transfer_error + backward_transfer_error;
}

float CalculateTransferError(const std::vector<cv::Point2f> &src_points,
                             const std::vector<cv::Point2f> &dst_points,
                             const cv::Mat &m, const cv::Mat &inlies_mask) {
  cv::Mat expected_dst_points;
  cv::perspectiveTransform(src_points, expected_dst_points, m);
  cv::Mat diff = ZerosLike(expected_dst_points);
  cv::subtract(expected_dst_points, cv::Mat(dst_points).reshape(2, 1), diff,
               inlies_mask.reshape(1, 1));
  cv::pow(diff, 2, diff);
  auto transfer_error = cv::sum(cv::sum(diff))[0];

  std::cout << transfer_error << std::endl;
  return transfer_error;
}

void SlamCore::Initialize(const cv::Mat &camera_intrinsics,
                          const cv::Mat &camera_distortion_coeffs) {
  _orb_extractor.SetCameraIntrinsicsAndDistortionCoeffs(
      camera_intrinsics, camera_distortion_coeffs);
  //  _camera_intrinsic = camera_intrinsics;
  //  _camera_distortion_coeffs = camera_distortion_coeffs;
}
} // namespace clean_slam
