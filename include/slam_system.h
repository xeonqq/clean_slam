//
// Created by root on 12/25/19.
//

#ifndef CLEAN_SLAM_SLAMSYSTEM_H
#define CLEAN_SLAM_SLAMSYSTEM_H

#include "dataset_loader.h"
#include "slam_core.h"
#include "viewer.h"
#include <thread>

namespace clean_slam {
class SlamSystem {
public:
  SlamSystem() = default;
  SlamSystem(const DatasetLoader *dataset_loader)
      : _dataset_loader{dataset_loader},
        _viewer{dataset_loader->GetViewerSettings()} {}
  void Run();
  const CameraTrajectory &GetCamTrajectory() const;

private:
  const DatasetLoader *_dataset_loader;
  Viewer _viewer;
  SlamCore _core;
};
}

#endif // CLEAN_SLAM_SLAMSYSTEM_H
