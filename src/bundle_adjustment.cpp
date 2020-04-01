//
// Created by root on 3/27/20.
//

#include "bundle_adjustment.h"
#include "homogeneous_matrix.h"
#include <opencv2/core/mat.hpp>
#include <third_party/g2o/g2o/core/block_solver.h>
#include <third_party/g2o/g2o/core/optimization_algorithm_levenberg.h>
#include <third_party/g2o/g2o/core/robust_kernel_impl.h>
#include <third_party/g2o/g2o/solvers/linear_solver_dense.h>
#include <third_party/g2o/g2o/solvers/linear_solver_eigen.h>
#include <third_party/g2o/g2o/types/types_seven_dof_expmap.h>
#include <third_party/g2o/g2o/types/types_six_dof_expmap.h>

namespace clean_slam {

BundleAdjustment::BundleAdjustment(const cv::Mat &camera_intrinsics)
    : _camera_intrinsics(camera_intrinsics) {
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

  linearSolver =
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  _optimizer.setAlgorithm(solver);
}

void BundleAdjustment::AddPose(int id, const g2o::SE3Quat &pose, bool fixed) {
  g2o::VertexSE3Expmap *se3quat = new g2o::VertexSE3Expmap();
  se3quat->setId(id);
  se3quat->setEstimate(pose);
  se3quat->setFixed(fixed);
  _optimizer.addVertex(se3quat);
}

void BundleAdjustment::AddPoint3D(int id, const Eigen::Vector3d &point_3d,
                                  bool fixed, bool marginalized) {
  g2o::VertexSBAPointXYZ *point_xyz = new g2o::VertexSBAPointXYZ();
  point_xyz->setId(id);
  point_xyz->setEstimate(point_3d);
  point_xyz->setMarginalized(marginalized);
  point_xyz->setFixed(fixed);
  _optimizer.addVertex(point_xyz);
}

void BundleAdjustment::AddEdge(int point_3d_id, int pose_id,
                               const Eigen::Vector2d &measurement,
                               const Eigen::Matrix2d &information,
                               double robust_kernel_delta) {
  g2o::EdgeSE3ProjectXYZ *edge = new g2o::EdgeSE3ProjectXYZ();
  edge->setVertex(0, _optimizer.vertex(point_3d_id));
  edge->setVertex(1, _optimizer.vertex(pose_id));
  edge->setMeasurement(measurement); // pixel point (u,v)
  edge->setInformation(information);
  auto kernel = new g2o::RobustKernelHuber;
  kernel->setDelta(robust_kernel_delta);
  edge->setRobustKernel(kernel);
  edge->fx = _camera_intrinsics.at<double>(0, 0);
  edge->fy = _camera_intrinsics.at<double>(1, 1);
  edge->cx = _camera_intrinsics.at<double>(0, 2);
  edge->cy = _camera_intrinsics.at<double>(1, 2);

  _optimizer.addEdge(edge);
}

void BundleAdjustment::Optimize(int iterations, bool verbose) {
  _optimizer.setVerbose(verbose);
  _optimizer.initializeOptimization();
  _optimizer.optimize(iterations);
}

const g2o::SE3Quat &BundleAdjustment::GetOptimizedPose(int id) const {
  const g2o::VertexSE3Expmap *vertex_SE3 =
      static_cast<const g2o::VertexSE3Expmap *>(_optimizer.vertex(id));
  return vertex_SE3->estimate();
}

const Eigen::Vector3d &BundleAdjustment::GetOptimizedPoint(int id) const {
  const g2o::VertexSBAPointXYZ *point =
      static_cast<const g2o::VertexSBAPointXYZ *>(_optimizer.vertex(id));
  return point->estimate();
}
//    def vertex_estimate(self, vertex_id):
//        vertex = self._optimizer.vertex(vertex_id)
//        return vertex.estimate()
//
//    def vertex(self, vertex_id):
//        return self._optimizer.vertex(vertex_id)

} // namespace clean_slam
