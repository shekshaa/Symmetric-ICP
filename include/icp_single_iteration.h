#ifndef ICP_SINGLE_ITERATION_H
#define ICP_SINGLE_ITERATION_H
#include <Eigen/Core>

enum ICPMethod
{
  ICP_METHOD_POINT_TO_POINT = 0,
  ICP_METHOD_POINT_TO_PLANE = 1,
  ICP_METHOD_SYMMETRIC = 2,
  NUM_ICP_METHODS = 3,
};
// Conduct a single iteration of the iterative closest point method using the symmetric
// objective to align (VP,FP) to (VQ,FQ) by finding the rigid transformation (R,t)
//
// Inputs:
//   VP  #VX by 3 list of mesh vertex positions
//   FP  #FX by 3 list of triangle mesh indices into VX
//   VQ  #VY by 3 list of mesh vertex positions
//   FQ  #FY by 3 list of triangle mesh indices into VY
//   num_samples  number of random samples to use (larger --> more accurate)
//   method  method of linearization to use
// Outputs:
//   R  3 by 3 rotation matrix
//   t  3d translation vector
void icp_single_iteration(
  const Eigen::MatrixXd & VP,
  const Eigen::MatrixXi & FP,
  const Eigen::MatrixXd & VQ,
  const Eigen::MatrixXi & FQ,
  const int num_samples,
  const bool is_robust,
  const ICPMethod method,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t);

#endif
