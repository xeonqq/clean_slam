//
// Created by root on 3/29/20.
//

#ifndef CLEAN_SLAM_SRC_OCTAVE_SCALES_H_
#define CLEAN_SLAM_SRC_OCTAVE_SCALES_H_

#include <array>
#include <cmath>

#include "bound.h"

namespace clean_slam {

class OctaveScales {
public:
  constexpr OctaveScales(float per_octave_scale)
      : kOctaveBasicScale{per_octave_scale}, kLogOctaveBasicScale{std::log(
                                                 kOctaveBasicScale)},
        _octave_inv_sigma2_scales{}, _octave_scales{} {

    _octave_inv_sigma2_scales[0] = _octave_scales[0] = 1;
    for (size_t i = 1; i < _octave_scales.size(); ++i) {
      _octave_scales[i] = _octave_scales[i - 1] * kOctaveBasicScale;
      _octave_inv_sigma2_scales[i] =
          1.0f / (_octave_scales[i] * _octave_scales[i]);
    }
  }
  float operator[](size_t i) const;
  const std::array<float, 8> &GetOctaveInvSigma2Scales() const;
  constexpr size_t size() const { return _octave_scales.size(); }
  int MapDistanceToOctaveLevel(float distance, const Bound &bound) const;

private:
  const float kOctaveBasicScale;
  const float kLogOctaveBasicScale;
  std::array<float, 8> _octave_inv_sigma2_scales;
  std::array<float, 8> _octave_scales;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_OCTAVE_SCALES_H_
