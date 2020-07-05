//
// Created by root on 4/17/20.
//

#ifndef CLEAN_SLAM_SRC_BOUND_H_
#define CLEAN_SLAM_SRC_BOUND_H_

namespace clean_slam {
template <typename T> class Bound {
public:
  Bound() = default;
  Bound(T low, T high) : _low(low), _high(high) {}

  bool IsWithIn(T value) const { return value >= _low && value <= _high; }
  T GetLow() const { return _low; }
  T GetHigh() const { return _high; }

private:
  T _low;
  T _high;
};
using BoundF = Bound<float>;
using BoundI = Bound<int>;
} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_BOUND_H_
