//
// Created by root on 12/28/19.
//

#ifndef CLEAN_SLAM_SRC_MATH_UTILS_H_
#define CLEAN_SLAM_SRC_MATH_UTILS_H_

namespace clean_slam {

template <typename T1, typename T2>
T1 Interpolate(const T1 &start, const T1 &end, const T2 &t) {
  return start + (end - start) * t;
}
}
#endif // CLEAN_SLAM_SRC_MATH_UTILS_H_
