#include "FastMassSpring.h"
#include <iostream>


namespace USTC_CG::node_mass_spring {
FastMassSpring::FastMassSpring(
    const Eigen::MatrixXd& X,
    const EdgeSet& E,
    const float stiffness,
    const float h,
    unsigned max_iter)
    : MassSpring(X, E){
    // construct L and J at initialization
    std::cout << "init fast mass spring" << std::endl;

    size_t n_vertices = X.rows();
    size_t n_edges = E.size();
    this->stiffness = stiffness; 
    this->D = MatrixXd::Zero(n_edges, 3);
    this->h = h; 
    this->max_iter = max_iter;

    MatrixXd L_left = MatrixXd::Zero(n_vertices, n_vertices);
    MatrixXd J_left = MatrixXd::Zero(n_vertices, n_edges);
    size_t i = 0;
    for (const auto& e : E) {
        Eigen::Vector3d x = X.row(e.first) - X.row(e.second);
        double length = x.norm();
        double length_rest = E_rest_length[i];
        D.row(i) = length_rest * x / length;

        Eigen::VectorXd a(n_vertices);
        Eigen::VectorXd s(n_edges);
        a.setZero();
        s.setZero();
        a(e.first) = 1.;
        a(e.second) = -1.;
        s(i) = 1.;

        L_left += stiffness * a * a.transpose();
        J_left += stiffness * a * s.transpose();
        ++i;
    }
    for (size_t i = 0; i < n_vertices; ++i) {
        if (dirichlet_bc_mask[i]) {
            L_left.row(i).setZero();
        }
    }
    L = KroneckerProduct_I(L_left);
    J = KroneckerProduct_I(J_left);

    size_t size = 3 * n_vertices;
    double mass_per_vertex = mass / n_vertices;
    
    SparseMatrix_d M(size, size);
    std::vector<Trip_d> TripletListM;
    for (size_t i = 0; i < size; ++i) {
        if (!dirichlet_bc_mask[i / 3])
            TripletListM.push_back(Trip_d(i, i, mass_per_vertex));
        else 
            TripletListM.push_back(Trip_d(i, i, 1.));
    }
    M.setFromTriplets(TripletListM.begin(), TripletListM.end());
    SparseMatrix_d A = M + pow(h, 2) * L;
    A.makeCompressed();

    Eigen::initParallel();
    solver.compute(A);
    if (solver.info() != Eigen::Success) {
        std::cerr << "Decompsition failed!" << std::endl;
        exit(-1);
    }
    // (HW Optional) precompute A and prefactorize
    // Note: one thing to take care of: A is related with stiffness, if stiffness changes, A need to be recomputed
}

void FastMassSpring::step()
{
    // (HW Optional) Necessary preparation
    // ...
    size_t n_vertices = X.rows();
    size_t size = 3 * n_vertices;

    double mass_per_vertex = mass / n_vertices;

    Eigen::Vector3d acceleration_ext = gravity + wind_ext_acc;
    Eigen::MatrixXd acceleration_collision =
        getSphereCollisionForce(sphere_center.cast<double>(), sphere_radius) / mass_per_vertex;

    MatrixXd Y = X + h * vel;
    Y.rowwise() += pow(h, 2) * acceleration_ext.transpose();
    if (enable_sphere_collision) {
        Y += acceleration_collision;
    }
    /*for (size_t i = 0; i < n_vertices; ++i) {
        if (dirichlet_bc_mask[i])
            Y.row(i) = X.row(i) +
                       pow(h, 2) *
                           (unflatten(L * flatten(X)).row(i) - unflatten(J * flatten(D)).row(i)) /
                           mass_per_vertex;
    }*/

    MatrixXd X_former = X;
    for (unsigned iter = 0; iter < max_iter; iter++) {
        // (HW Optional)
        // local_step and global_step alternating solving
        size_t i = 0;
        for (const auto& e : E) { 
            Eigen::Vector3d x = X.row(e.first) - X.row(e.second);
            double length = x.norm();
            double length_rest = E_rest_length[i];
            D.row(i) = length_rest * x / length;
            ++i;
        }

        Eigen::VectorXd b = pow(h, 2) * J * flatten(D) + mass_per_vertex * flatten(Y);
        for (size_t k = 0; k < n_vertices; ++k) {
            if (dirichlet_bc_mask[k])
                for (int l = 0; l < 3; ++l)
                    b(3 * k + l) = X_former(k, l);
        }

        X = unflatten(solver.solve(b));
        if (solver.info() != Eigen::Success) {
            std::cerr << "Solving failed!" << std::endl;
            exit(-2);
        }
    }
    vel = (X - X_former) / h;
}

SparseMatrix_d FastMassSpring::KroneckerProduct_I(const MatrixXd& A)
{
    size_t r = A.rows();
    size_t c = A.cols();
    SparseMatrix_d A_I(3 * r, 3 * c);
    std::vector<Trip_d> TripletListK;
    for (size_t p = 0; p < r; ++p) {
        for (size_t q = 0; q < c; ++q) {
            if (A(p, q)) {
                for (int i = 0; i < 3; ++i) {
                    size_t pi = 3 * p + i;
                    size_t qi = 3 * q + i;
                    TripletListK.push_back(Trip_d(pi, qi, A(p, q)));
                }
            }
        }
    }
    A_I.setFromTriplets(TripletListK.begin(), TripletListK.end());
    A_I.makeCompressed();

    return A_I;
}
}  // namespace USTC_CG::node_mass_spring
