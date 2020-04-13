//
// Created by root on 4/13/20.
//

#include "ioc_factory.h"
namespace clean_slam {

IocFactory::IocFactory(const DatasetLoader *dataset_loader)
    : dataset_loader(dataset_loader), _viewer{
                                          dataset_loader->GetViewerSettings()} {
  const auto &orb_extractor_settings =
      dataset_loader->GetOrbExtractorSettings();

  auto orb_detector = cv::ORB::create(orb_extractor_settings.num_features,
                                      orb_extractor_settings.scale_factor,
                                      orb_extractor_settings.num_levels);
  _orb_extractor = OrbExtractor(std::move(orb_detector),
                                dataset_loader->GetCameraIntrinsics(),
                                dataset_loader->GetDistortionCoeffs());
}

std::unique_ptr<SlamCore> IocFactory::CreateSlamCore() {
  return std::make_unique<SlamCore>(dataset_loader->GetCameraIntrinsics(),
                                    dataset_loader->GetDistortionCoeffs(),
                                    &_orb_extractor, &_viewer);
}

std::thread IocFactory::CreateViewerThread() {
  return std::thread(&Viewer::Run, &_viewer);
}
} // namespace clean_slam
