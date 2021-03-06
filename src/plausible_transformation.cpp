//
// Created by root on 3/23/20.
//

#include "plausible_transformation.h"
#include "cv_utils.h"
#include "projective_transformation.h"
#include <third_party/spdlog/spdlog.h>

namespace clean_slam {
PlausibleTransformation::PlausibleTransformation(
    const cv::Mat &r, const cv::Mat &t, int num_of_good_points,
    const cv::Mat &good_points_mask, const cv::Mat &triangulated_points,
    bool has_similar_good)
    : r(r), t(t), num_of_good_points(num_of_good_points),
      good_points_mask(good_points_mask),
      _triangulated_points(triangulated_points),
      _has_similar_good(has_similar_good) {

  _good_triangulated_points =
      ToStdVectorByMask(_triangulated_points, good_points_mask);
}

const cv::Mat &PlausibleTransformation::R() const { return r; }

const cv::Mat &PlausibleTransformation::T() const { return t; }

int PlausibleTransformation::GetNumOfGoodPoints() const {
  return num_of_good_points;
}

const cv::Mat &PlausibleTransformation::GetGoodPointsMask() const {
  return good_points_mask;
}
HomogeneousMatrix PlausibleTransformation::GetHomogeneousMatrix() const {
  return CreateHomogeneousMatrix(r, t);
}
const cv::Mat &PlausibleTransformation::GetTriangulatedPoints() const {
  return _triangulated_points;
}
bool PlausibleTransformation::IsGood() const {
  //  spdlog::info("has similar: {}, num_good_points: {}, all: {}",
  //  _has_similar_good, num_of_good_points, _triangulated_points.rows);
  return (!_has_similar_good) &&
         (num_of_good_points > kNumOfGoodPointsThreshold) &&
         (num_of_good_points >
          ProjectiveTransformation::kPositivePointsRateThreshold *
              _triangulated_points.rows);
}

const std::vector<Eigen::Vector3d> &
PlausibleTransformation::GetGoodTriangulatedPoints() const {
  return _good_triangulated_points;
}

} // namespace clean_slam
