//
// Created by root on 6/27/20.
//
#include "pose_bundle_adjustment.h"

#include <third_party/g2o/g2o/core/block_solver.h>
#include <third_party/g2o/g2o/core/optimization_algorithm_levenberg.h>
#include <third_party/g2o/g2o/core/robust_kernel_impl.h>
#include <third_party/g2o/g2o/solvers/linear_solver_dense.h>
#include <third_party/g2o/g2o/types/types_six_dof_expmap.h>
namespace clean_slam {

PoseBundleAdjustment::PoseBundleAdjustment(const cv::Mat &camera_intrinsics)
    : _camera_intrinsics{camera_intrinsics} {
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

  // why dense from OrbSlam
  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  _optimizer.setAlgorithm(solver);
}

void PoseBundleAdjustment::AddPose(const g2o::SE3Quat &pose) {
  g2o::VertexSE3Expmap *se3quat = new g2o::VertexSE3Expmap();
  se3quat->setId(0);
  se3quat->setEstimate(pose);
  se3quat->setFixed(false);
  _optimizer.addVertex(se3quat);
}

void PoseBundleAdjustment::AddEdge(const Eigen::Vector3d &point_3d,
                                   const Eigen::Vector2d &measurement,
                                   const Eigen::Matrix2d &information,
                                   double robust_kernel_delta) {
  g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();
  e->setVertex(
      0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(_optimizer.vertex(0)));
  e->setMeasurement(measurement);
  e->setInformation(information);

  auto kernel = new g2o::RobustKernelHuber;
  kernel->setDelta(robust_kernel_delta);
  e->setRobustKernel(kernel);

  e->fx = _camera_intrinsics.at<double>(0, 0);
  e->fy = _camera_intrinsics.at<double>(1, 1);
  e->cx = _camera_intrinsics.at<double>(0, 2);
  e->cy = _camera_intrinsics.at<double>(1, 2);
  e->Xw = point_3d;
  _optimizer.addEdge(e);
}

void PoseBundleAdjustment::Optimize(int iterations, bool verbose) {
  _optimizer.setVerbose(verbose);
  _optimizer.initializeOptimization();
  _optimizer.optimize(iterations);
}

const g2o::SE3Quat &PoseBundleAdjustment::GetOptimizedPose() const {
  const g2o::VertexSE3Expmap *vertex_SE3 =
      static_cast<const g2o::VertexSE3Expmap *>(_optimizer.vertex(0));
  return vertex_SE3->estimate();
}

} // namespace clean_slam
