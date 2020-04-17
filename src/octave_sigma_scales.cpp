//
// Created by root on 3/29/20.
//
#include <algorithm>

#include "octave_sigma_scales.h"

namespace clean_slam {

float OctaveScales::operator[](size_t i) const { return _octave_scales[i]; }

const std::array<float, 8> &OctaveScales::GetOctaveSigmaScales() const {
  return _octave_sigma_scales;
}
} // namespace clean_slam
