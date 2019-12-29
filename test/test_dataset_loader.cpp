//
// Created by root on 12/25/19.
//

#include "dataset_loader.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <opencv2/opencv.hpp>

namespace clean_slam {

TEST(DatasetLoaderTest, load_dataset) {
  // arrange
  DatasetLoader loader;
  // act
  loader.LoadFreiburgDataset(
      DATASET_DIR + std::string("/rgbd_dataset_freiburg1_xyz"), CONFIG_DIR);
  const auto& image_files = loader.GetImageFiles();
  const auto& ground_truths = loader.GetGroundTruths();

  // assert
  EXPECT_EQ(image_files.size(), 798);
  EXPECT_DOUBLE_EQ(image_files.front().timestamp, 1305031102.175304);
  EXPECT_EQ(image_files.front().image_filename, "rgb/1305031102.175304.png");

  EXPECT_EQ(ground_truths.size(), 3000);

  EXPECT_EQ(ground_truths.front().GetTimestamp(), 1305031098.6659);
  const auto& q = ground_truths.front().GetQuaternion() ;
  EXPECT_NEAR(-q.x(), 0.6132,1.1e-4);
  EXPECT_NEAR(-q.y(), 0.5962, 1.1e-4);
  EXPECT_NEAR(-q.z(), -0.331, 1.1e-4);
  EXPECT_NEAR(-q.w(), -0.3986,1.1e-4);
  EXPECT_TRUE(ground_truths.front().GetTranslation().isApprox(
      Eigen::Vector3d(1.3563, 0.6305, 1.6380)));
}


TEST(DatasetLoaderTest, GetGroundTruthAtTime_ThenGroudTruthWillBeInterpolated) {
  // arrange
  DatasetLoader loader;
  // act
  loader.LoadFreiburgDataset(
      DATASET_DIR + std::string("/rgbd_dataset_freiburg1_xyz"), CONFIG_DIR);

  // assert
  const auto ground_truth = loader.GetGroundTruthAt(1305031102.175304);

  EXPECT_EQ(ground_truth.GetTimestamp(), 1305031102.175304);

  const auto translation = ground_truth.GetTranslation();
  EXPECT_FLOAT_EQ(translation.x(),1.34064388 );
  EXPECT_FLOAT_EQ(translation.y(),0.626624807);
  EXPECT_FLOAT_EQ(translation.z(),1.65765381);

  const auto& q = ground_truth.GetQuaternion() ;
  EXPECT_NEAR(-q.x(), 6.57444653e-01, 1.1e-4);
  EXPECT_NEAR(-q.y(), 6.12530539e-01, 1.1e-4);
  EXPECT_NEAR(-q.z(), -2.94845424e-01,1.1e-4);
  EXPECT_NEAR(-q.w(), -3.24889307e-01,1.1e-4);
}

template<typename T>
void ExpectTwoMatsEqual(const cv::Mat& a, const cv::Mat& b)
{
  for (size_t row=0; row<a.rows; ++row)
  {
    for (size_t col=0; col<a.cols; ++col)
      EXPECT_FLOAT_EQ(a.at<T>(row, col), b.at<T>(row, col));
  }
}

TEST(DatasetLoaderTest, LoadCameraIntrinsics) {
  // arrange
  DatasetLoader loader;
  // act
  loader.LoadFreiburgDataset(
      DATASET_DIR + std::string("/rgbd_dataset_freiburg1_xyz"), CONFIG_DIR);

  const auto &camera_intrinsics = loader.GetCameraIntrinsics();
  const auto &distortion_coeffs = loader.GetDistortionCoeffs();

  Eigen::Matrix3d expect_camera_intrinsics;
  expect_camera_intrinsics << 517.306408, 0, 318.643040, 0, 516.469215,
      255.313989, 0, 0, 1;

  Eigen::Matrix<double, 5, 1> expect_distortion_coeffs;
  expect_distortion_coeffs << 0.262383, -0.953104, -0.005358, 0.002628,
      1.163314;

  // assert
  //  ExpectTwoMatsEqual<double>(expect_camera_intrinsics, camera_intrinsics);
  //  ExpectTwoMatsEqual<double>(expect_distortion_coeffs, distortion_coeffs);
  expect_camera_intrinsics.isApprox(camera_intrinsics);
  expect_distortion_coeffs.isApprox(distortion_coeffs);
}
} // namespace clean_slam
