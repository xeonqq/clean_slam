//
// Created by root on 5/17/20.
//

#include "cv_algorithms.h"
#include "gtest/gtest.h"
#include <gmock/gmock-matchers.h>

namespace clean_slam {
TEST(
    SearchByProjection,
    DISABLED_GivenOrbFeaturesFromCurrentFrameAndMapPoints_ThenFindTheMatchingPairOfMapPointsAndCurrentKeyPoints) {
  // Given
  float key_point_size = 2;
  float key_point_angle = -1;
  float key_point_response = 0;
  std::vector<cv::KeyPoint> current_key_points = {
      // float x, float y, float _size, float _angle=-1, float _response=0, int
      // _octave=0
      {10, 11, key_point_size, key_point_angle, key_point_response, 0},
      {11, 12, key_point_size, key_point_angle, key_point_response, 0},
      {50, 50, key_point_size, key_point_angle, key_point_response, 0},
      {100, 12, key_point_size, key_point_angle, key_point_response, 0},
  };
  cv::Mat current_points_descriptors =
      cv::Mat::zeros(current_key_points.size(), 32, CV_8UC1);
  current_points_descriptors.at<uint8_t>(0, 0) = 2;  //   00000010
  current_points_descriptors.at<uint8_t>(1, 0) = 27; //  00011011
  current_points_descriptors.at<uint8_t>(2, 0) = 27; //  00011011
  current_points_descriptors.at<uint8_t>(3, 0) = 19; //  00010010

  OrbFeatures current_features{std::move(current_key_points),
                               current_points_descriptors};

  std::vector<Eigen::Vector2d> projected_map_points{
      {10, 10}, {10, 12}, {100, 10}};
  cv::Mat map_points_descriptors =
      cv::Mat::zeros(projected_map_points.size(), 32, CV_8UC1);
  map_points_descriptors.at<uint8_t>(0, 0) = 27; //  00011011
  map_points_descriptors.at<uint8_t>(1, 0) = 1;
  map_points_descriptors.at<uint8_t>(2, 0) = 18; // 00010010
  DescriptorsView map_points_descriptors_view{map_points_descriptors,
                                              {0, 1, 2}};

  std::vector<bool> mask(projected_map_points.size(), true);
  int search_radius = 10;

  // When
  const auto matches = clean_slam::SearchByProjection(
      current_features, projected_map_points, map_points_descriptors_view, mask,
      search_radius);
  const std::vector<std::pair<size_t, size_t>> expected_matches = {
      {0, 1}, {1, 0}, {2, 3}};
  EXPECT_THAT(matches, ::testing::UnorderedElementsAreArray(expected_matches));

  // Then
}
} // namespace clean_slam
