//
// Created by root on 5/16/20.
#include "frame.h"
#include "gtest/gtest.h"

namespace clean_slam {

TEST(Points3DView, WhenIterate_ThenRetievePointsAccordingToProvidedIndexes) {
  std::vector<Eigen::Vector3d> points = {{0, 0, 0}, {1, 1, 1}, {2, 2, 2}};
  std::vector<std::size_t> indexes = {2, 1};
  Points3DView view{points, indexes};
  size_t i{0};
  std::vector<Eigen::Vector3d> expected_points = {{2, 2, 2}, {1, 1, 1}};
  std::for_each(view.begin(), view.end(),
                [&i, &expected_points](const auto &point) {
                  EXPECT_TRUE(point.isApprox(expected_points[i]));
                  ++i;
                });
  EXPECT_TRUE(points[2].isApprox(expected_points[0]));
  EXPECT_EQ(points.size(), 3);
}

} // namespace clean_slam
