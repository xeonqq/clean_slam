//
// Created by root on 9/6/20.
//

#include "cv_utils.h"
#include "gtest/gtest.h"
namespace clean_slam {

TEST(RemoveByIndex, Vec) {
  std::vector<int> vec{2, 3, 27, 4};

  auto result = RemoveByIndex(vec, std::vector{3, 1});
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result[0], static_cast<uint8_t>(2));
  EXPECT_EQ(result[1], static_cast<uint8_t>(27));
}

TEST(RemoveByIndex, Mat) {
  cv::Mat mat = cv::Mat::zeros(4, 1, CV_8U);
  mat.row(0) = 2;
  mat.row(1) = 3;
  mat.row(2) = 27;
  mat.row(3) = 4;

  auto result = RemoveByIndex(mat, std::vector{3, 1});
  EXPECT_EQ(result.rows, 2);
  EXPECT_EQ(result.cols, 1);
  EXPECT_EQ(result.at<uint8_t>(0), static_cast<uint8_t>(2));
  EXPECT_EQ(result.at<uint8_t>(1), static_cast<uint8_t>(27));
}
} // namespace clean_slam
