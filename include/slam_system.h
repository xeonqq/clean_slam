//
// Created by root on 12/25/19.
//

#ifndef CLEAN_SLAM_SLAMSYSTEM_H
#define CLEAN_SLAM_SLAMSYSTEM_H

#include "dataset_loader.h"
#include "slam_core.h"

namespace clean_slam {
class SlamSystem {
public:
  SlamSystem() = default;
  SlamSystem(const DatasetLoader *dataset_loader)
      : _dataset_loader{dataset_loader} {};
  void Run();
  const std::vector<g2o::SE3Quat> &GetCamTrajectory() const;

private:
  const DatasetLoader *_dataset_loader;
  SlamCore _core;
};
}

#endif // CLEAN_SLAM_SLAMSYSTEM_H
