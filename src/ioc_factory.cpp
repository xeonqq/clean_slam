//
// Created by root on 4/13/20.
//

#include "ioc_factory.h"

namespace clean_slam {

IocFactory::IocFactory(const DatasetLoader *dataset_loader, bool use_gui)
    : _use_gui{use_gui}, _dataset_loader(dataset_loader),
      _octave_scales{dataset_loader->GetOrbExtractorSettings().scale_factor},
      _optimizer{dataset_loader->GetCameraIntrinsics(), _octave_scales},
      _optimizer_only_pose{dataset_loader->GetCameraIntrinsics(),
                           _octave_scales} {
  if (use_gui) {
    _viewer = std::make_unique<Viewer>(dataset_loader->GetViewerSettings());
  } else {
    _viewer = std::make_unique<NullViewer>();
  }

  const auto &orb_extractor_settings =
      dataset_loader->GetOrbExtractorSettings();

  auto orb_detector = cv::ORB::create(orb_extractor_settings.num_features,
                                      orb_extractor_settings.scale_factor,
                                      orb_extractor_settings.num_levels);
  _orb_extractor = OrbExtractor(std::move(orb_detector),
                                dataset_loader->GetCameraIntrinsics(),
                                dataset_loader->GetDistortionCoeffs());
}

std::unique_ptr<boost::msm::back::state_machine<SlamCore>>
IocFactory::CreateSlamCore() {
  return std::make_unique<boost::msm::back::state_machine<SlamCore>>(
      _dataset_loader->GetCameraIntrinsics(),
      _dataset_loader->GetDistortionCoeffs(), &_orb_extractor, &_optimizer,
      &_optimizer_only_pose, _viewer.get(), _octave_scales);
}

boost::optional<std::thread> IocFactory::CreateViewerThread() {
  boost::optional<std::thread> thread;
  if (_use_gui) {
    thread = std::thread(&IViewer::Run, _viewer.get());
  }
  return thread;
}
} // namespace clean_slam
