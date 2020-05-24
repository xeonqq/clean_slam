//
// Created by root on 5/17/20.
//

#include "cv_algorithms.h"
#include "gtest/gtest.h"
#include <gmock/gmock-matchers.h>

namespace cv {
bool operator==(const ::cv::DMatch &lhs, const ::cv::DMatch &rhs) {
  return lhs.distance == rhs.distance && lhs.queryIdx == rhs.queryIdx &&
         lhs.trainIdx == rhs.trainIdx;
}
} // namespace cv

namespace clean_slam {
class SearchByProjectionFixture : public testing::Test {

public:
  SearchByProjectionFixture() {
    // setup current observed points
    float key_point_size = 2;
    float key_point_angle = -1;
    float key_point_response = 0;
    std::vector<cv::KeyPoint> current_key_points = {
        // float x, float y, float _size, float _angle=-1, float _response=0,
        // int
        // _octave=0
        {10, 11, key_point_size, key_point_angle, key_point_response, 1},
        {10, 11, key_point_size, key_point_angle, key_point_response, 0},
        {11, 12, key_point_size, key_point_angle, key_point_response, 0},
        {50, 50, key_point_size, key_point_angle, key_point_response, 0},
        {100, 12, key_point_size, key_point_angle, key_point_response, 0},
    };
    cv::Mat current_points_descriptors =
        cv::Mat::zeros(current_key_points.size(), 32, CV_8UC1);
    current_points_descriptors.at<uint8_t>(0, 0) = 2;  //   00000010
    current_points_descriptors.at<uint8_t>(1, 0) = 2;  //   00000010
    current_points_descriptors.at<uint8_t>(2, 0) = 27; //  00011011
    current_points_descriptors.at<uint8_t>(3, 0) = 27; //  00011011
    current_points_descriptors.at<uint8_t>(4, 0) = 19; //  00010011

    _current_features =
        OrbFeatures{std::move(current_key_points), current_points_descriptors};

    // setup map points
    _projected_map_points =
        std::vector<Eigen::Vector2d>{{10, 10}, {10, 12}, {100, 10}};
    _map_points_descriptors =
        cv::Mat::zeros(_projected_map_points.size(), 32, CV_8UC1);
    _map_points_descriptors.at<uint8_t>(0, 0) = 27; //  00011011
    _map_points_descriptors.at<uint8_t>(1, 0) = 1;  //  00000001
    _map_points_descriptors.at<uint8_t>(2, 0) = 19; // 00010011
    _indexes = {0, 1, 2};
    _octaves = {0, 1, 0};

    _mask = cv::Mat(_projected_map_points.size(), 1, CV_8U, true);
  }

protected:
  OctavesView GetOctavesView() { return OctavesView{_octaves, _indexes}; }

  OrbFeatures _current_features;
  std::vector<Eigen::Vector2d> _projected_map_points;
  cv::Mat _map_points_descriptors;
  cv::Mat _mask;

private:
  std::vector<size_t> _indexes;
  std::vector<int> _octaves;
};

TEST_F(
    SearchByProjectionFixture,
    GivenOrbFeaturesFromCurrentFrameAndMapPoints_ThenFindTheMatchingPairOfMapPointsAndCurrentKeyPoints) {

  // When
  int search_radius = 10;
  const auto matches = clean_slam::SearchByProjection(
      _current_features, _projected_map_points, GetOctavesView(),
      _map_points_descriptors, _mask, search_radius);

  // Then

  const std::vector<::cv::DMatch> expected_matches = {
      {0, 2, 0}, {1, 0, 2}, {2, 4, 0}};
  EXPECT_THAT(matches, ::testing::UnorderedElementsAreArray(expected_matches));
}

TEST_F(
    SearchByProjectionFixture,
    GivenOrbFeaturesFromCurrentFrameAndMapPointsAlsoMaskWithFalse_ThenFindTheMatchingPairOfMapPointsAndCurrentKeyPoints) {

  // Given
  _mask.at<uint8_t>(2) = false;

  // When
  int search_radius = 10;
  const auto matches = clean_slam::SearchByProjection(
      _current_features, _projected_map_points, GetOctavesView(),
      _map_points_descriptors, _mask, search_radius);

  // Then
  const std::vector<::cv::DMatch> expected_matches = {{0, 2, 0}, {1, 0, 2}};
  EXPECT_THAT(matches, ::testing::UnorderedElementsAreArray(expected_matches));
}

} // namespace clean_slam
