//
// Created by root on 3/29/20.
//
#include <algorithm>

#include "octave_scales.h"

namespace clean_slam {

float OctaveScales::operator[](size_t i) const { return _octave_scales[i]; }

const std::array<float, 8> &OctaveScales::GetOctaveInvSigma2Scales() const {
  return _octave_inv_sigma2_scales;
}

int OctaveScales::MapDistanceToOctaveLevel(float distance,
                                           const BoundF &bound) const {
  // max_distance / 1.2^level > distance > max_distance / 1.2^(level+1)
  int level =
      std::ceil(std::log(bound.GetHigh() / distance) / kLogOctaveBasicScale);
  return std::max(0, std::min(level, 7));
}
} // namespace clean_slam
