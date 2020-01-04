//
// Created by root on 12/29/19.
//

#include "slam_core.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace clean_slam {

void SlamCore::Track(const cv::Mat image) {

  const auto orb_features = _orb_extractor.Detect(image);
  if (!_previous_frame.GetImage().empty()) {
    const auto key_pixels_pair =
        _orb_feature_matcher.Match(orb_features, _previous_frame);

    cv::Mat H =
        cv::findHomography(key_pixels_pair.GetKeyPixelsCurrFrame(),
                           key_pixels_pair.GetKeyPixelsPrevFrame(), CV_RANSAC);
    std::cout << H << std::endl;
    cv::Mat img_matches;
    cv::drawMatches(
        image, orb_features.GetKeyPoints(), _previous_frame.GetImage(),
        _previous_frame.GetKeyPoints(), _orb_feature_matcher.GetGoodMatches(),
        img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    imshow("Good Matches & Object detection", img_matches);
  }
  _previous_frame = Frame{image, orb_features};

  cv::Mat out_im;
  cv::drawKeypoints(image, orb_features.GetKeyPoints(), out_im);
  cv::imshow("image", out_im);
  cv::waitKey(0);
}

void SlamCore::Initialize(const cv::Mat &camera_intrinsic,
                          const cv::Mat &camera_distortion_coeffs) {
  _camera_intrinsic = camera_intrinsic;
  _camera_distortion_coeffs = camera_distortion_coeffs;
}
} // namespace clean_slam
