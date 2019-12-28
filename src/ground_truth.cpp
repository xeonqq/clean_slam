//
// Created by root on 12/28/19.
//

#include "ground_truth.h"
#include "math_utils.h"
#include <Eigen/Core>
#include <algorithm>

namespace clean_slam {
GroundTruth GroundTruths::GetGroundTruthAt(double timestamp) const {
  const auto it =
      std::lower_bound(begin(), end(), timestamp,
                       [](const auto &ground_truth, const auto &value) {
                         return ground_truth.GetTimestamp() < value;
                       });
  const auto &a = *(it - 1);
  const auto &b = *it;
  const auto t =
      (timestamp - a.GetTimestamp()) / (b.GetTimestamp() - a.GetTimestamp());
  const auto translation =
      Interpolate(a.GetTranslation(), b.GetTranslation(), t);
  const auto quaternion = a.GetQuaternion().slerp(t, b.GetQuaternion());
  return GroundTruth{translation, quaternion, timestamp};
}
} // namespace clean_slam
