//
// Created by root on 9/18/20.
//
#include "map.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <boost/range/algorithm.hpp>
#include <gmock/gmock-matchers.h>
namespace clean_slam {
/*
 * test data from
https://github.com/xeonqq/multiple_view_geometry/blob/master/tests/test_triangulation.py
 *
frame0 extrinsic cam wrt to world
 [[ 7.07106781e-01  4.32978028e-17  7.07106781e-01  1.70000000e+00]
 [-7.07106781e-01  4.32978028e-17  7.07106781e-01  0.00000000e+00]
 [ 0.00000000e+00 -1.00000000e+00  6.12323400e-17  5.00000000e-01]
 [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
frame 0 intrinsic
 [[350.   0. 512.]
 [  0. 350. 384.]
 [  0.   0.   1.]]
frame1 extrinsic
[[ 1.000000e+00  0.000000e+00  0.000000e+00  2.300000e+00]
 [ 0.000000e+00  6.123234e-17  1.000000e+00  0.000000e+00]
 [ 0.000000e+00 -1.000000e+00  6.123234e-17  5.000000e-01]
 [ 0.000000e+00  0.000000e+00  0.000000e+00  1.000000e+00]]
frame 0 intrinsic
[[350.   0. 512.]
 [  0. 350. 384.]
 [  0.   0.   1.]]
frame0 pixel:
 [[225.63636364 225.63636364 225.63636364]
 [458.99617376 309.00382624 159.01147871]]
frame1 pixel:
 [[477.         477.         477.        ]
 [442.33333333 325.66666667 209.        ]]
3d points:
 [[2 2 2]
 [3 3 3]
 [0 1 2]]
*/
Frame SetupFrame0(Map *map) {
  Eigen::Matrix3d rot;
  // clang-format off
  rot <<
    7.07106781e-01, 0, 7.07106781e-01,
    -7.07106781e-01,0, 7.07106781e-01,
    0.00000000e+00, -1.00000000e+00, 0;
  // clang-format on
  Eigen::Vector3d t;
  t << 1.7, 0., 0.5;

  g2o::SE3Quat Twc{rot, t};
  auto Tcw = Twc.inverse();
  float size = 1;
  std::vector<cv::KeyPoint> key_points{{225.63636364, 458.99617376, size},
                                       {225.63636364, 309.00382624, size},
                                       {225.63636364, 159.01147871, size}};
  cv::Mat key_points_descriptors =
      cv::Mat::zeros(key_points.size(), 32, CV_8UC1);
  key_points_descriptors.at<uint8_t>(0, 0) = 2;  //   00000010
  key_points_descriptors.at<uint8_t>(1, 0) = 27; //  00011011
  key_points_descriptors.at<uint8_t>(2, 0) = 19; //  00010011
  OrbFeatures orb_features{std::move(key_points), key_points_descriptors};
  return Frame{std::move(orb_features), map, Tcw, 0};
}

Frame SetupFrame1(Map *map) {
  Eigen::Matrix3d rot;
  // clang-format off
  rot <<
  1.000000e+00, 0.000000e+00, 0.000000e+00,
  0.000000e+00, 0, 1.000000e+00,
  0.000000e+00, -1.000000e+00, 0;
  // clang-format on
  Eigen::Vector3d t;
  t << 2.3, 0.0, 0.5;
  g2o::SE3Quat Twc{rot, t};
  auto Tcw = Twc.inverse();
  float size = 1;
  std::vector<cv::KeyPoint> key_points{
      {477, 442.33333333, size}, {477, 325.66666667, size}, {477, 209, size}};
  cv::Mat key_points_descriptors =
      cv::Mat::zeros(key_points.size(), 32, CV_8UC1);
  key_points_descriptors.at<uint8_t>(0, 0) = 2;  //   00000010
  key_points_descriptors.at<uint8_t>(1, 0) = 27; //  00011011
  key_points_descriptors.at<uint8_t>(2, 0) = 19; //  00010011
  OrbFeatures orb_features{std::move(key_points), key_points_descriptors};
  return Frame{std::move(orb_features), map, Tcw, 0.3};
}

MATCHER(Vector3dEq, "") {
  return std::get<0>(arg).isApprox(std::get<1>(arg), 0.1);
}

TEST(LocalMapping, LocalMapping) {
  OctaveScales octave_scales{1.2};
  OrbFeatureMatcher feature_matcher;
  cv::Mat intrinsics =
      (cv::Mat_<double>(3, 3) << 350., 0., 512., 0., 350., 384., 0., 0., 1.0);
  Map map{octave_scales, feature_matcher, intrinsics};

  Frame frame0 = SetupFrame0(&map);
  Frame frame1 = SetupFrame1(&map);
  auto kf0 = map.AddKeyFrame(frame0);
  auto kf1 = map.AddKeyFrame(frame1);
  map.AddMapPoints({{2, 3, 0}}, {0}, kf0, {0}, kf1);
  map.LocalMapping(kf0);
  auto &map_points = map.GetMapPoints();
  std::vector<Eigen::Vector3d> map_points_positions;
  boost::range::transform(
      map_points, std::back_inserter(map_points_positions),
      [](const auto &map_point) { return map_point.GetPoint3D(); });
  // two additional map points are triangulated after local mapping
  std::vector<Eigen::Vector3d> expected_map_points_positions{
      {2, 3, 0}, {2, 3, 1}, {2, 3, 2}};
  EXPECT_THAT(map_points_positions,
              ::testing::UnorderedPointwise(Vector3dEq(),
                                            expected_map_points_positions));
}
} // namespace clean_slam
