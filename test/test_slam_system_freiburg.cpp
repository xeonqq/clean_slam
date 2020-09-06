//
// Created by root on 12/28/19.
//

#include "slam_system.h"
#include "gtest/gtest.h"

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/tuple/tuple.hpp>
#include <iostream>
#include <set>
#include <string>
#include <vector>
namespace clean_slam {

bool operator==(const g2o::SE3Quat &lhs, const g2o::SE3Quat &rhs) {
  const auto v1 = lhs.toVector();
  const auto v2 = rhs.toVector();
  auto result = true;
  for (int i = 0; i < v1.size(); ++i) {
    result &= (v1[i] == v2[i]);
  }
  return result;
}

float ComputeTransformationError(const g2o::SE3Quat &ground_truth,
                                 const StampedTransformation &calculated) {
  const auto calculated_translation =
      calculated.GetTransformation().to_homogeneous_matrix().block<3, 1>(0, 3);

  const auto gt_translation =
      ground_truth.to_homogeneous_matrix().block<3, 1>(0, 3);
  return (gt_translation - calculated_translation).norm();
}

float ComputeRotationError(const g2o::SE3Quat &ground_truth,
                           const StampedTransformation &calculated) {
  const auto calculated_rotation = calculated.GetTransformation().rotation();
  const auto gt_rotation = ground_truth.rotation();
  auto diff = gt_rotation.angularDistance(calculated_rotation);
  return diff;
}

struct Report {
  void CalculateError(const g2o::SE3Quat &gt_initial_pose_wrt_current_pose,
                      const StampedTransformation &stamped_transformation) {
    const auto translation_error = ComputeTransformationError(
        gt_initial_pose_wrt_current_pose, stamped_transformation);
    _translation_errors.push_back(translation_error);
    _translation_error_sum += translation_error;

    const auto rotation_error = ComputeRotationError(
        gt_initial_pose_wrt_current_pose, stamped_transformation);
    _rotation_errors.push_back(rotation_error);
    _rotation_error_sum += rotation_error;
    ++_count;
  }

  float GetMeanTranslationError() { return _translation_error_sum / _count; }
  float GetMeanRotationError() { return _rotation_error_sum / _count; }
  void show() {
    std::cout << "Report: \ntrajectory size: " << _count << "\n"
              << "mean translate error: " << GetMeanTranslationError() << "\n"
              << "mean rotation error: " << GetMeanRotationError() << std::endl;
  }
  int _count{0};
  std::vector<double> _rotation_errors;
  std::vector<double> _translation_errors;
  float _translation_error_sum;
  float _rotation_error_sum;
};

TEST(SlamSystemFreiburgTest, Run) {
  // arrange
  DatasetLoader dataset_loader;
  dataset_loader.LoadFreiburgDataset(
      DATASET_DIR + std::string("/rgbd_dataset_freiburg1_xyz"),
      CONFIG_DIR + std::string("/TUM1.yaml"));
  SlamSystem system{&dataset_loader, false};

  // act
  system.Run();

  // assert
  const auto &trajectory = system.GetCamTrajectory();

  const auto initial_gt_transformation =
      dataset_loader.GetGroundTruthAt(trajectory.front().GetTimestamp())
          .GetTransformation();

  Report report;
  for (const auto &stamped_transformation : trajectory) {
    const g2o::SE3Quat ground_truth_transformation =
        dataset_loader.GetGroundTruthAt(stamped_transformation.GetTimestamp())
            .GetTransformation();
    const auto gt_initial_pose_wrt_current_pose =
        ground_truth_transformation * initial_gt_transformation.inverse();

    report.CalculateError(gt_initial_pose_wrt_current_pose,
                          stamped_transformation);
  }

  report.show();
  EXPECT_LE(report.GetMeanTranslationError(), 0.1655865);
  EXPECT_LE(report.GetMeanRotationError(), 0.1421);
}

} // namespace clean_slam
