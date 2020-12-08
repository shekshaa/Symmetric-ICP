#ifndef POINT_TO_PLANE_RIGID_MATCHING_H
#define POINT_TO_PLANE_RIGID_MATCHING_H
#include <Eigen/Core>
// Given a set of source points P and corresponding target points Q and normals NQ
// find the optimal rigid transformation (R,t) that aligns P to Q
// using point to plane rigid matching
//
// Inputs:
//   P  #X by 3 set of source points
//   Q  #X by 3 set of target points
//   NQ  #X by 3 set of target normals
//
// Outputs:
//   R  3 by 3 rotation matrix
//   t  3d translation vector 
//   
void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & Q,
  const Eigen::MatrixXd & NQ,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t);
#endif


