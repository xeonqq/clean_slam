//
// Created by root on 12/25/19.
//

#include "slam_system.h"

namespace clean_slam {

const std::vector<g2o::SE3Quat> &SlamSystem::GetCamTrajectory() const {
  return _core.GetTrajectory();
}

void SlamSystem::Run() {
  if (_dataset_loader) {
    _core.Initialize(_dataset_loader->GetCameraIntrinsics(),
                     _dataset_loader->GetDistortionCoeffs());
    size_t i = 0;
    for (const auto &image_file : _dataset_loader->GetImageFiles()) {
      auto im = cv::imread(_dataset_loader->GetDatasetFolder() + '/' +
                               image_file.image_filename,
                           cv::IMREAD_GRAYSCALE);
      _core.Track(im);
      ++i;
      if (i == 2)
        break;
    }
  }
}
} // namespace clean_slam
