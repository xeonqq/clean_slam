//
// Created by root on 4/13/20.
//

#include "gtest/gtest.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

namespace clean_slam {

TEST(Norm, HammingDistanceSingleValue) {
  cv::Mat descriptor1 = cv::Mat(1, 1, CV_8UC1);
  cv::Mat descriptor2 = cv::Mat(1, 1, CV_8UC1);
  descriptor1.at<uint8_t>(0, 0) = 27; //  00011011
  descriptor2.at<uint8_t>(0, 0) = 18; //  00010010

  int dist = cv::norm(descriptor1, descriptor2, cv::NORM_HAMMING);
  EXPECT_EQ(dist, 2);
}

TEST(Norm, HammingDistanceMultipleValues) {
  cv::Mat descriptor1 = cv::Mat(1, 2, CV_8UC1);
  cv::Mat descriptor2 = cv::Mat(1, 2, CV_8UC1);
  descriptor1.at<uint8_t>(0, 0) = 27; //  00011011
  descriptor1.at<uint8_t>(0, 1) = 18; //  00010010
  descriptor2.at<uint8_t>(0, 0) = 18; //  00010010
  descriptor2.at<uint8_t>(0, 1) = 27; //  00011011

  int dist = cv::norm(descriptor1, descriptor2, cv::NORM_HAMMING);
  EXPECT_EQ(dist, 4);
}
} // namespace clean_slam