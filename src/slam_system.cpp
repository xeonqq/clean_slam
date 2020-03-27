//
// Created by root on 12/25/19.
//
#include <chrono>
#include <iostream>

#include "slam_system.h"
#include "spdlog/spdlog.h"

namespace clean_slam {

const CameraTrajectory &SlamSystem::GetCamTrajectory() const {
  return _core.GetTrajectory();
}

void SlamSystem::Run() {

  spdlog::set_level(spdlog::level::info); // Set global log level to debug

  if (_dataset_loader) {
    std::thread _viewer_thread(&Viewer::Run, &_viewer);

    _core.Initialize(_dataset_loader->GetCameraIntrinsics(),
                     _dataset_loader->GetDistortionCoeffs());
    size_t i = 0;
    for (const auto &image_file : _dataset_loader->GetImageFiles()) {

      using namespace std::chrono;
      if (i >= 22 && i <= 23) {

        auto start = high_resolution_clock::now();
        auto im = cv::imread(_dataset_loader->GetDatasetFolder() + '/' +
                                 image_file.image_filename,
                             cv::IMREAD_GRAYSCALE);
        _core.Track(im, image_file.timestamp);
        auto stop = high_resolution_clock::now();
        spdlog::info("{} slam runtime per step: {}", i,
                     duration_cast<microseconds>(stop - start).count());
      }
      ++i;
      //      if (i == 2)
      //        break;
    }
    _viewer_thread.join();
  }
}
} // namespace clean_slam
