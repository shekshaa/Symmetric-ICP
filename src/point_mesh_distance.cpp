#include "point_mesh_distance.h"
#include "igl/per_face_normals.h"
#include "igl/AABB.h"

void point_mesh_distance(
        const Eigen::MatrixXd &X,
        const Eigen::MatrixXd &VY,
        const Eigen::MatrixXi &FY,
        Eigen::VectorXd &D,
        Eigen::MatrixXd &P,
        Eigen::MatrixXd &N) {

    // compute distance faster using AABB tree data structure
    igl::AABB<Eigen::MatrixXd,3> tree;
    Eigen::VectorXi I;
    Eigen::MatrixXd C;
    tree.init(VY, FY);
    tree.squared_distance(VY, FY, X, D, I, P);
    D = D.cwiseSqrt();

    // find all triangle normals
    Eigen::MatrixXd all_N;
    igl::per_face_normals(VY, FY, Eigen::Vector3d(1, 1, 1).normalized(), all_N);

    // select normals
    N.resizeLike(X);
    for (int i = 0; i < I.rows(); ++i) {
        N.row(i) = all_N.row(I(i));
    }
}
