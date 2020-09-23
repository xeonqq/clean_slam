//
// Created by root on 12/25/19.
//
#include <chrono>
#include <iostream>

#include "slam_core.h"
#include "slam_system.h"
#include "spdlog/spdlog.h"

namespace clean_slam {

CameraTrajectory SlamSystem::GetCamTrajectory() const {
  return _core->GetTrajectory();
}

void SlamSystem::Run() {

  spdlog::set_level(spdlog::level::debug); // Set global log level to debug

  if (_dataset_loader) {
    auto viewer_thread = _ioc_factory.CreateViewerThread();

    size_t i = 0;
    for (const auto &image_file : _dataset_loader->GetImageFiles()) {

      using namespace std::chrono;
      //      int end = 500;
      //      if (i >= 20 && i < end) {

      auto start = high_resolution_clock::now();
      auto im = cv::imread(_dataset_loader->GetDatasetFolder() + '/' +
                               image_file.image_filename,
                           cv::IMREAD_GRAYSCALE);
      _core->process_event(NewImage{im, image_file.timestamp});
      auto stop = high_resolution_clock::now();
      spdlog::info("{} slam runtime per step: {} ms", i,
                   duration_cast<microseconds>(stop - start).count() / 1000.f);
      //      }
      ++i;
      //      if (i > end)
      //        break;
    }
    if (viewer_thread) {
      viewer_thread->join();
      cv::destroyAllWindows();
    }
  }
}
} // namespace clean_slam
