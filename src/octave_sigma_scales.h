//
// Created by root on 3/29/20.
//

#ifndef CLEAN_SLAM_SRC_OCTAVE_SIGMA_SCALES_H_
#define CLEAN_SLAM_SRC_OCTAVE_SIGMA_SCALES_H_

#include <array>
namespace clean_slam {

class OctaveSigmaScales {
public:
  constexpr OctaveSigmaScales() : _octave_sigma_scale{} {

    _octave_sigma_scale[0] = 1;
    for (size_t i = 1; i < _octave_sigma_scale.size(); ++i) {
      _octave_sigma_scale[i] = _octave_sigma_scale[i - 1] * kOctaveBasicScale;
    }
    std::for_each(_octave_sigma_scale.begin(), _octave_sigma_scale.end(),
                  [](auto &scale) {
                    scale *= scale;
                    scale = 1.0f / scale;
                  });
  }
  float operator[](size_t i) const;

private:
  std::array<float, 8> _octave_sigma_scale;
  const float kOctaveBasicScale = 1.2f;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_OCTAVE_SIGMA_SCALES_H_
