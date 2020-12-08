#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
#include <random>
#include <cmath>

int main(int argc, char *argv[]) {
    // Load input meshes
    Eigen::MatrixXd OVX, RVX, VX, VY;
    Eigen::MatrixXi FX, FY;
    if (argc <= 1) {
        igl::read_triangle_mesh("../data/data2/max-registration-partial.obj", OVX, FX);
        igl::read_triangle_mesh("../data/data2/max-registration-complete.obj", VY, FY);
    } else {
        if (argc >= 3) {
            igl::read_triangle_mesh(argv[1], OVX, FX);
            igl::read_triangle_mesh(argv[2], VY, FY);
        } else {
            igl::read_triangle_mesh(argv[1], VY, FY);
            Eigen::RowVector3d COMY = VY.colwise().mean();
            double rmseY = sqrt(((VY.rowwise() - COMY).rowwise().norm()).mean());
            VY = ((VY.rowwise() - COMY) / rmseY).rowwise() + COMY;
            FX = FY;
            OVX = VY;
        }
    }
    int num_samples = 100;
    bool show_samples = true;
    bool is_robust = false;
    ICPMethod method = ICP_METHOD_POINT_TO_POINT;

    igl::opengl::glfw::Viewer viewer;
    std::cout << R"(
  [space]  toggle animation
  M,m      toggle between point-to-point, point-to-plane and symmetric methods
  P,p      show sample points
  R,r      reset, also recomputes a random sampling and closest points
  T,t      Transform initial mesh
  B,b      toggle between robust and non-robust
  S        double number of samples
  s        halve number of samples
)";

    // predefined colors
    const Eigen::RowVector3d orange(1.0, 0.7, 0.2);
    const Eigen::RowVector3d blue(0.2, 0.3, 0.8);
    const auto &set_meshes = [&]() {
        // taken from assignment
        // Concatenate meshes into one big mesh
        Eigen::MatrixXd V(VX.rows() + VY.rows(), VX.cols());
        V << VX, VY;
        Eigen::MatrixXi F(FX.rows() + FY.rows(), FX.cols());
        F << FX, FY.array() + VX.rows();
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        // Assign orange and blue colors to each mesh's faces
        Eigen::MatrixXd C(F.rows(), 3);
        C.topLeftCorner(FX.rows(), 3).rowwise() = orange;
        C.bottomLeftCorner(FY.rows(), 3).rowwise() = blue;
        viewer.data().set_colors(C);
    };
    const auto &set_points = [&]() {
        // taken from assignment
        Eigen::MatrixXd X, P, NN;
        random_points_on_mesh(num_samples, VX, FX, X, NN);
        Eigen::VectorXd D;
        Eigen::MatrixXd N;
        point_mesh_distance(X, VY, FY, D, P, N);
        Eigen::MatrixXd XP(X.rows() + P.rows(), 3);
        XP << X, P;
        Eigen::MatrixXd C(XP.rows(), 3);
        C.array().topRows(X.rows()).rowwise() = (1. - (1. - orange.array()) * .8);
        C.array().bottomRows(P.rows()).rowwise() = (1. - (1. - blue.array()) * .4);
        viewer.data().set_points(XP, C);
        Eigen::MatrixXi E(X.rows(), 2);
        E.col(0) = Eigen::VectorXi::LinSpaced(X.rows(), 0, X.rows() - 1);
        E.col(1) = Eigen::VectorXi::LinSpaced(X.rows(), X.rows(), 2 * X.rows() - 1);
        viewer.data().set_edges(XP, E, Eigen::RowVector3d(0.3, 0.3, 0.3));
    };
    const auto &reset = [&]() {
        VX = RVX;
        set_meshes();
        if (show_samples) {
            set_points();
        }
    };
    const auto &transform = [&]() {
        if (not viewer.core().is_animating) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, M_PI/3);
            double rand_theta = dis(gen);
            Eigen::Vector3d a = Eigen::Vector3d::Random().normalized();
            double scale = (OVX.colwise().maxCoeff() - OVX.colwise().minCoeff()).maxCoeff();
            Eigen::RowVector3d t = Eigen::RowVector3d::Random() * scale * 0.5;
            Eigen::Matrix3d W;
            W << 0, -a(2), a(1), a(2), 0, -a(0), -a(1), a(0), 0;
            Eigen::Matrix3d random_R = Eigen::Matrix3d::Identity() + sin(rand_theta) * W + (1 - cos(rand_theta)) * (W * W);
            RVX = ((OVX * random_R).rowwise() + t).eval();
            reset();
        }
    };
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &) -> bool {
        if (viewer.core().is_animating) {
            // Perform single iteration of ICP method
            Eigen::Matrix3d R;
            Eigen::RowVector3d t;
            icp_single_iteration(VX, FX, VY, FY, num_samples, is_robust, method, R, t);
            // Apply transformation to source mesh
            VX = ((VX * R.transpose()).rowwise() + t).eval();
            set_meshes();
            if (show_samples) {
                set_points();
            }
        }
        return false;
    };
    viewer.callback_key_pressed =
            [&](igl::opengl::glfw::Viewer &, unsigned char key, int) -> bool {
                switch (key) {
                    case ' ':
                        viewer.core().is_animating ^= 1;
                        break;
                    case 'T':
                    case 't':
                        transform();
                        break;
                    case 'R':
                    case 'r':
                        reset();
                        break;
                    case 'B':
                    case 'b':
                        is_robust ^= 1;
                        if (is_robust) {
                            std::cout << "robust" << std::endl;
                        } else {
                            std::cout << "not robust" << std::endl;
                        }
                        break;
                    case 'M':
                    case 'm': {
                        method = (ICPMethod) ((((int) method) + 1) % ((int) NUM_ICP_METHODS));
                        if (method == ICP_METHOD_POINT_TO_POINT) {
                            std::cout << "point-to-point" << std::endl;
                        } else if (method == ICP_METHOD_POINT_TO_PLANE) {
                            std::cout << "point-to-plane" << std::endl;
                        } else {
                            std::cout << "symmetric" << std::endl;
                        }
                        break;
                    }
                    case 'P':
                    case 'p':
                        show_samples ^= 1;
                        break;
                    case 'S':
                        num_samples = (num_samples - 1) * 2;
                        break;
                    case 's':
                        num_samples = (num_samples / 2) + 1;
                        break;
                    default:
                        return false;
                }
                return true;
            };
    transform();
    reset();
    viewer.core().is_animating = false;
    viewer.data().point_size = 10;
    viewer.launch();

    return EXIT_SUCCESS;
}
