//
// Created by root on 3/29/20.
//
#include <algorithm>

#include "octave_sigma_scales.h"

namespace clean_slam {

float OctaveSigmaScales::operator[](size_t i) const {
  return _octave_sigma_scale[i];
}
} // namespace clean_slam
