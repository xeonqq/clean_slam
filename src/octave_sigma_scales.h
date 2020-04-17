//
// Created by root on 3/29/20.
//

#ifndef CLEAN_SLAM_SRC_OCTAVE_SIGMA_SCALES_H_
#define CLEAN_SLAM_SRC_OCTAVE_SIGMA_SCALES_H_

#include <array>
namespace clean_slam {

class OctaveScales {
public:
  constexpr OctaveScales(float per_octave_scale)
      : kOctaveBasicScale{per_octave_scale}, _octave_sigma_scales{},
        _octave_scales{} {

    _octave_sigma_scales[0] = _octave_scales[0] = 1;
    for (size_t i = 1; i < _octave_scales.size(); ++i) {
      _octave_scales[i] = _octave_scales[i - 1] * kOctaveBasicScale;
      _octave_sigma_scales[i] = 1.0f / (_octave_scales[i] * _octave_scales[i]);
    }
  }
  float operator[](size_t i) const;
  const std::array<float, 8> &GetOctaveSigmaScales() const;
  size_t size() const { return _octave_scales.size(); }

private:
  const float kOctaveBasicScale;
  std::array<float, 8> _octave_sigma_scales;
  std::array<float, 8> _octave_scales;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_OCTAVE_SIGMA_SCALES_H_
