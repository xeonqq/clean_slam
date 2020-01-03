//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include "cv_utils.h"
#include <opencv2/imgcodecs.hpp>

namespace clean_slam {

void SlamCore::Track(const cv::Mat image) {

  const auto orb_features = _orb_extractor.Detect(image);
  if (!_previous_frame.GetImage().empty()) {
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(
        cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    std::vector<cv::DMatch> matches;
    matcher.match(orb_features.GetDescriptors(),
                  _previous_frame.GetDescriptors(), matches);

    std::vector<cv::DMatch> good_matches = RetrieveGoodMatches(matches, 20);
    cv::Mat img_matches;
    cv::drawMatches(image, orb_features.GetKeyPoints(),
                    _previous_frame.GetImage(), _previous_frame.GetKeyPoints(),
                    good_matches, img_matches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    imshow("Good Matches & Object detection", img_matches);
  }
  _previous_frame = Frame{image, orb_features};

  cv::Mat out_im;
  cv::drawKeypoints(image, orb_features.GetKeyPoints(), out_im);
  cv::imshow("image", out_im);
  cv::waitKey(0);
}

void SlamCore::Initialize(
    const Eigen::Matrix3d &camera_intrinsic,
    const Eigen::Matrix<double, 5, 1> &camera_distortion_coeffs) {
  _camera_intrinsic = camera_intrinsic;
  _camera_distortion_coeffs = camera_distortion_coeffs;
}
} // namespace clean_slam
