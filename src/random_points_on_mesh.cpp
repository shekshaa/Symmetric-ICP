#include "random_points_on_mesh.h"
#include "igl/random_points_on_mesh.h"
#include "igl/per_face_normals.h"

void random_points_on_mesh(
        const int n_samples,
        const Eigen::MatrixXd &V,
        const Eigen::MatrixXi &F,
        Eigen::MatrixXd &X,
        Eigen::MatrixXd &N) {

    Eigen::MatrixXd B;
    Eigen::VectorXi F_idx;
    F_idx.resize(n_samples);
    B.resize(n_samples, 3);
    X.resize(n_samples, 3);
    igl::random_points_on_mesh(n_samples, V, F, B, F_idx, X);

    int n_vertex = F.maxCoeff() + 1;
    Eigen::MatrixXd FN;
    FN.resize(n_vertex, 3);
    igl::per_face_normals(V, F, Eigen::Vector3d(1,1,1).normalized(), FN);
    N.resize(n_samples, 3);
    for (int i = 0; i < F_idx.rows(); ++i)
        N.row(i) = FN.row(F_idx(i));
}
