//
// Created by root on 4/17/20.
//

#include "bound.h"
namespace clean_slam {

Bound::Bound(float low, float high) : _low(low), _high(high) {}

bool Bound::IsWithIn(float value) const {
  return value >= _low && value <= _high;
}
} // namespace clean_slam
