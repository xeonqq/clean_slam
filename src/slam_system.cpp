//
// Created by root on 12/25/19.
//

#include "slam_system.h"
#include <iostream>
namespace clean_slam {

const CameraTrajectory &SlamSystem::GetCamTrajectory() const {
  return _core.GetTrajectory();
}

void SlamSystem::Run() {
  if (_dataset_loader) {
    std::thread _viewer_thread(&Viewer::Run, &_viewer);

    _core.Initialize(_dataset_loader->GetCameraIntrinsics(),
                     _dataset_loader->GetDistortionCoeffs());
    size_t i = 0;
    for (const auto &image_file : _dataset_loader->GetImageFiles()) {
      if (i > 3 && i < 6) {

        auto im = cv::imread(_dataset_loader->GetDatasetFolder() + '/' +
                                 image_file.image_filename,
                             cv::IMREAD_GRAYSCALE);
        _core.Track(im, image_file.timestamp);
      }
      ++i;
      //      if (i == 2)
      //        break;
    }
    _viewer_thread.join();
  }
}
} // namespace clean_slam
