//
// Created by root on 3/27/20.
//

#ifndef CLEAN_SLAM_SRC_BUNDLE_ADJUSTMENT_H_
#define CLEAN_SLAM_SRC_BUNDLE_ADJUSTMENT_H_
#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/core/block_solver.h>
#include <third_party/g2o/g2o/core/optimization_algorithm_levenberg.h>
#include <third_party/g2o/g2o/core/sparse_optimizer.h>
#include <third_party/g2o/g2o/types/se3quat.h>

namespace clean_slam {

class BundleAdjustment {
public:
  template <template <typename> class LinearSolver>
  static BundleAdjustment
  CreateFullBundleAdjustment(const cv::Mat &camera_intrinsics) {
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new LinearSolver<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    return BundleAdjustment{camera_intrinsics, solver};
  }

  BundleAdjustment(const cv::Mat &camera_intrinsics,
                   g2o::OptimizationAlgorithmLevenberg *solver);

  void AddPose(int id, const g2o::SE3Quat &pose, bool fixed = false);

  void AddPoint3D(int id, const Eigen::Vector3d &point_3d, bool fixed = false,
                  bool marginalized = true);

  void AddEdge(int point_3d_id, int pose_id, const Eigen::Vector2d &measurement,
               const Eigen::Matrix2d &information = Eigen::Matrix2d::Identity(),
               double delta = sqrt(5.991));

  void AddEdgeOnlyPose(
      const Eigen::Vector3d &point_3d, const Eigen::Vector2d &measurement,
      const Eigen::Matrix2d &information = Eigen::Matrix2d::Identity(),
      double robust_kernel_delta = sqrt(5.991));

  void Clear();
  void Optimize(int iterations = 20, bool verbose = false);

  const g2o::SE3Quat &GetOptimizedPose(int id) const;
  const Eigen::Vector3d &GetOptimizedPoint(int id) const;

private:
  g2o::SparseOptimizer _optimizer;
  cv::Mat _camera_intrinsics;
};

} // namespace clean_slam
#endif // CLEAN_SLAM_SRC_BUNDLE_ADJUSTMENT_H_
