#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_point_rigid_matching.h"
#include "point_to_plane_rigid_matching.h"
#include "symmetric_rigid_matching.h"
#include "igl/slice.h"
#include "igl/median.h"
#include "iostream"


void icp_single_iteration(
        const Eigen::MatrixXd &VP,
        const Eigen::MatrixXi &FP,
        const Eigen::MatrixXd &VQ,
        const Eigen::MatrixXi &FQ,
        const int num_samples,
        const bool is_robust,
        const ICPMethod method,
        Eigen::Matrix3d &R,
        Eigen::RowVector3d &t) {

    Eigen::MatrixXd P, NP;
    Eigen::MatrixXd Q, NQ;
    Eigen::VectorXd D;

    random_points_on_mesh(num_samples, VP, FP, P, NP); // select random points
    point_mesh_distance(P, VQ, FQ, D, Q, NQ); // find closest points

    // outlier rejection
    if (is_robust) {
        Eigen::MatrixXd robustP, robustNP;
        Eigen::MatrixXd robustQ, robustNQ;
        std::vector<int> robust_rows;
        double median;
        igl::median(D, median);
        double std_estimation = 1.4826 * median;
        for (int i = 0; i < num_samples; ++i) {
            if ((NP.row(i).dot(NQ.row(i)) >= 0) and (D(i) <= 2.5 * std_estimation)) {
                robust_rows.push_back(i);
            }
        }
        std::cout << robust_rows.size() << std::endl;
        int *ptr = &robust_rows[0];
        Eigen::Map<Eigen::VectorXi> robust(ptr, robust_rows.size());
        igl::slice(P, robust, 1, robustP);
        igl::slice(Q, robust, 1, robustQ);
        igl::slice(NP, robust, 1, robustNP);
        igl::slice(NQ, robust, 1, robustNQ);
        P = robustP;
        Q = robustQ;
        NP = robustNP;
        NQ = robustNQ;
    }

    // method selection
    if (method == ICP_METHOD_POINT_TO_POINT) {
        point_to_point_rigid_matching(P, Q, R, t); // point-to-point
    } else if (method == ICP_METHOD_POINT_TO_PLANE) {
        point_to_plane_rigid_matching(P, Q, NQ, R, t); //point-to-plane
    } else if (method == ICP_METHOD_SYMMETRIC) { // symmetric
        symmetric_rigid_matching(P, Q, NP, NQ, R, t);
    }
}
