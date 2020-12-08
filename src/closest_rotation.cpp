#include "closest_rotation.h"
#include <Eigen/Dense>

void closest_rotation(
        const Eigen::Matrix3d &M,
        Eigen::Matrix3d &R) {
    // SVD decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d omega = Eigen::Matrix3d::Identity();

    // Calculation of optimal R
    double det = (U * V.transpose()).determinant();
    omega(2, 2) = det;
    R = U * omega * V.transpose();
}
