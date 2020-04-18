//
// Created by root on 4/17/20.
//

#ifndef CLEAN_SLAM_SRC_BOUND_H_
#define CLEAN_SLAM_SRC_BOUND_H_

namespace clean_slam {
class Bound {
public:
  Bound() = default;
  Bound(float low, float high);
  bool IsWithIn(float value) const;
  float GetLow() const;
  float GetHigh() const;

private:
  float _low;
  float _high;
};
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_BOUND_H_
