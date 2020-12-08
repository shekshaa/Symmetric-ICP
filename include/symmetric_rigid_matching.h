#ifndef REGISTRATION_SYMMETRIC_RIGID_MATCHING_H
#define REGISTRATION_SYMMETRIC_RIGID_MATCHING_H
#include <Eigen/Core>
// Given a set of source points P and normals NP
// and corresponding target points Q and normals NQ
// find the optimal rigid transformation (R,t) that aligns P to Q
// using symmetric objective matching
//
// Inputs:
//   P  #X by 3 set of source points
//   Q  #X by 3 set of target points
//   NP  #X by 3 set of source normals
//   NQ  #X by 3 set of target normals
//
// Outputs:
//   R  3 by 3 rotation matrix
//   t  3d translation vector
//
void symmetric_rigid_matching(
        const Eigen::MatrixXd & P,
        const Eigen::MatrixXd & Q,
        const Eigen::MatrixXd & NP,
        const Eigen::MatrixXd & NQ,
        Eigen::Matrix3d & R,
        Eigen::RowVector3d & t);
#endif
