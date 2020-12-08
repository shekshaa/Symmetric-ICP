#include "symmetric_rigid_matching.h"
#include "Eigen/Dense"

void symmetric_rigid_matching(
        const Eigen::MatrixXd & P,
        const Eigen::MatrixXd & Q,
        const Eigen::MatrixXd & NP,
        const Eigen::MatrixXd & NQ,
        Eigen::Matrix3d & R,
        Eigen::RowVector3d & t) {

    // normalize point sets
    Eigen::RowVector3d Pmean = P.colwise().mean();
    Eigen::RowVector3d Qmean = Q.colwise().mean();
    Eigen::MatrixXd Pbar = P.rowwise() - Pmean;
    Eigen::MatrixXd Qbar = Q.rowwise() - Qmean;

    // sum of normals
    Eigen::MatrixXd N = NP + NQ;

    // compute A and b of linear system
    int num_points = P.rows();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(6, 1);
    for (int i = 0; i < num_points; ++i) {
        Eigen::MatrixXd x_i = Eigen::MatrixXd(6, 1);
        Eigen::Vector3d n_i = N.row(i);
        Eigen::Vector3d p_i = Pbar.row(i);
        Eigen::Vector3d q_i = Qbar.row(i);
        double b_i = (p_i - q_i).dot(n_i);
        x_i << (p_i + q_i).cross(n_i), n_i;
        A += x_i * x_i.transpose();
        b += b_i * x_i;
    }

    // solve linear equation
    Eigen::VectorXd u = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-b);

    Eigen::Vector3d a_tilda = u.head(3); // scaled rotation axis
    Eigen::Vector3d t_tilda = u.tail(3); // scaled translation

    // compute intermediate rotation
    double theta = atan(a_tilda.norm()); // rotation angle
    Eigen::Vector3d a = a_tilda.normalized(); // normalized rotation axis
    Eigen::Matrix3d W;
    W << 0, -a(2), a(1), a(2), 0, -a(0), -a(1), a(0), 0;
    Eigen::Matrix3d intermediate_R = Eigen::Matrix3d::Identity() + sin(theta) * W + (1 - cos(theta)) * (W * W);

    // compose translations and rotations
    Eigen::Vector3d t1 = -Pmean.transpose();
    Eigen::Vector3d t2 = cos(theta) * t_tilda;
    Eigen::Vector3d t3 = Qmean.transpose();
    R = intermediate_R * intermediate_R;
    t = (intermediate_R * intermediate_R * t1) + (intermediate_R * t2) + t3;
}
