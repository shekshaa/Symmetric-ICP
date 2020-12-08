#include "point_to_plane_rigid_matching.h"
#include <Eigen/Dense>

void point_to_plane_rigid_matching(
        const Eigen::MatrixXd &P,
        const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &NQ,
        Eigen::Matrix3d &R,
        Eigen::RowVector3d &t) {
    int num_steps = 5;
    R = Eigen::Matrix3d::Identity();
    t = Eigen::RowVector3d::Zero();
    Eigen::MatrixXd changing_P = P; // store x as it changes in few steps

    for (int step = 0; step < num_steps; ++step) {
        // compute A and b of linear system
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(6, 1);
        for (int i = 0; i < changing_P.rows(); ++i) {
            Eigen::MatrixXd element = Eigen::MatrixXd(6, 1);
            Eigen::Vector3d p_i = changing_P.row(i).transpose();
            Eigen::Vector3d n_i = NQ.row(i).transpose();
            Eigen::Vector3d q_i = Q.row(i).transpose();
            element << p_i.cross(n_i), n_i;
            A += element * element.transpose();
            b += element * n_i.transpose() * (q_i - p_i);
        }

        // solve linear equation
        Eigen::VectorXd u = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

        // optimal rotation R
        Eigen::VectorXd a = u.head(3);
        double theta = a.norm();
        Eigen::VectorXd w = a.normalized();
        Eigen::Matrix3d W;
        W << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
        Eigen::Matrix3d optimal_R = Eigen::Matrix3d::Identity() + sin(theta) * W + (1 - cos(theta)) * (W * W);

        // optimal translation t
        Eigen::Vector3d optimal_t = u.tail(3);

        // update points, rotation and translation
        t = (optimal_R * t.transpose() + optimal_t).transpose();
        R = optimal_R * R;
        changing_P = (changing_P * optimal_R.transpose()).rowwise() + optimal_t.transpose();
    }
}
