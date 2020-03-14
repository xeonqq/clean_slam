//
// Created by root on 3/14/20.
//
#include "eigen_utils.h"

namespace clean_slam {
Eigen::Vector3d
SkewSymmetricMatToTranslation(const Eigen::Matrix3d &skew_symetric_mat) {
  Eigen::Vector3d translation;
  translation << skew_symetric_mat(2, 1), skew_symetric_mat(0, 2),
      skew_symetric_mat(1, 0);
  return translation;
}
} // namespace clean_slam
