#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"

void point_to_point_rigid_matching(
        const Eigen::MatrixXd &P,
        const Eigen::MatrixXd &Q,
        Eigen::Matrix3d &R,
        Eigen::RowVector3d &t) {
    // find optimal rotation
    Eigen::MatrixXd Pbar = P.rowwise() - P.colwise().mean();
    Eigen::MatrixXd Qbar = Q.rowwise() - Q.colwise().mean();
    Eigen::MatrixXd M = Qbar.transpose() * Pbar;
    closest_rotation(M, R);

    // find optimal translation based on optimal rotation
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(Q.rows());
    t = ((Q.transpose() * ones - R * P.transpose() * ones) / (ones.transpose() * ones)).transpose();
}

