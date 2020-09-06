//
// Created by root on 12/25/19.
//

#ifndef CLEAN_SLAM_SLAMSYSTEM_H
#define CLEAN_SLAM_SLAMSYSTEM_H

#include "dataset_loader.h"
#include "ioc_factory.h"
#include "slam_core.h"
#include "stamped_transformation.h"
#include "viewer.h"

#include <boost/msm/back/state_machine.hpp>
#include <thread>

namespace clean_slam {
class SlamSystem {
public:
  SlamSystem() = default;
  SlamSystem(const DatasetLoader *dataset_loader, bool use_gui = true)
      : _dataset_loader{dataset_loader}, _ioc_factory{dataset_loader, use_gui},
        _core{_ioc_factory.CreateSlamCore()} {}
  void Run();
  CameraTrajectory GetCamTrajectory() const;

private:
  const DatasetLoader *_dataset_loader;
  IocFactory _ioc_factory;
  std::unique_ptr<boost::msm::back::state_machine<SlamCore>> _core;
};
} // namespace clean_slam

#endif // CLEAN_SLAM_SLAMSYSTEM_H
