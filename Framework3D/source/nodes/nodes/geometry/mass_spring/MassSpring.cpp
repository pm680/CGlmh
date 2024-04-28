#include "MassSpring.h"
#include <iostream>

namespace USTC_CG::node_mass_spring {
MassSpring::MassSpring(const Eigen::MatrixXd& X, const EdgeSet& E)
{
    this->X = this->init_X = X;
    this->vel = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    this->E = E;

    std::cout << "number of edges: " << E.size() << std::endl;
    std::cout << "init mass spring" << std::endl;

    // Compute the rest pose edge length
    for (const auto& e : E) {
        Eigen::Vector3d x0 = X.row(e.first);
        Eigen::Vector3d x1 = X.row(e.second);
        this->E_rest_length.push_back((x0 - x1).norm());
    }

    // Initialize the mask for Dirichlet boundary condition
    dirichlet_bc_mask.resize(X.rows(), false);

    // (HW_TODO) Fix two vertices, feel free to modify this 
    unsigned n_fix = sqrt(X.rows());  // Here we assume the cloth is square
    dirichlet_bc_mask[0] = true;
    dirichlet_bc_mask[n_fix - 1] = true;
}

void MassSpring::step()
{
    Eigen::Vector3d acceleration_ext = gravity + wind_ext_acc;

    unsigned n_vertices = X.rows();

    // The reason to not use 1.0 as mass per vertex: the cloth gets heavier as we increase the resolution
    double mass_per_vertex = mass / n_vertices; 

    //----------------------------------------------------
    // (HW Optional) Bonus part: Sphere collision
    Eigen::MatrixXd acceleration_collision =
        getSphereCollisionForce(sphere_center.cast<double>(), sphere_radius) / mass_per_vertex;
    //----------------------------------------------------

    if (time_integrator == IMPLICIT_EULER) {
        // Implicit Euler
        TIC(step)

        // (HW TODO)
        size_t size = 3 * n_vertices;
        SparseMatrix_d M(size, size);
        std::vector<Trip_d> TripletListM;
        for (size_t i = 0; i < size; ++i) {
            TripletListM.push_back(Trip_d(i, i, mass_per_vertex));
        }
        M.setFromTriplets(TripletListM.begin(), TripletListM.end());
        SparseMatrix_d H_elastic =
            M / pow(h, 2) + computeHessianSparse(stiffness);  // size = [nx3, nx3]
        H_elastic = makeSPD(H_elastic);
            
        Eigen::initParallel();
        Eigen::ConjugateGradient<SparseMatrix_d> solver(H_elastic);

        // Solve Newton's search direction with linear solver
        MatrixXd f_int = -computeGrad(stiffness);
        MatrixXd Y = X + h * vel;
        Y.rowwise() += pow(h, 2) * acceleration_ext.transpose();
        if (enable_sphere_collision) {
            Y += acceleration_collision;
        }
        for (size_t i = 0; i < n_vertices; ++i) {
            if (dirichlet_bc_mask[i])
                Y.row(i) = X.row(i) - pow(h,2) * f_int.row(i) / mass_per_vertex;
        } 
        MatrixXd grad_g = -f_int + mass_per_vertex * (X - Y) / pow(h, 2); 
        MatrixXd delta_X_flatten = -solver.solve(flatten(grad_g));
        MatrixXd delta_X = unflatten(delta_X_flatten);
        std::cout << X << std::endl << std::endl;
        // update X and vel 
        X += delta_X;
        vel = delta_X / h;

        TOC(step)
    }
    else if (time_integrator == SEMI_IMPLICIT_EULER) {

        // Semi-implicit Euler
        MatrixXd acceleration = -computeGrad(stiffness) / mass_per_vertex;
        acceleration.rowwise() += acceleration_ext.transpose();

        // -----------------------------------------------
        // (HW Optional)
        if (enable_sphere_collision) {
            acceleration += acceleration_collision;
        }
        // -----------------------------------------------

        // (HW TODO): Implement semi-implicit Euler time integration
        for (size_t i = 0; i < n_vertices; ++i) {
            if (dirichlet_bc_mask[i])
                acceleration.row(i).setZero();
        }

        // Update X and vel 
        vel += h * acceleration;
        vel *= damping;
        X += h * vel;
    }
    else {
        std::cerr << "Unknown time integrator!" << std::endl;
        return;
    }
}

// There are different types of mass spring energy:
// For this homework we will adopt Prof. Huamin Wang's energy definition introduced in GAMES103
// course Lecture 2 E = 0.5 * stiffness * sum_{i=1}^{n} (||x_i - x_j|| - l)^2 There exist other
// types of energy definition, e.g., Prof. Minchen Li's energy definition
// https://www.cs.cmu.edu/~15769-f23/lec/3_Mass_Spring_Systems.pdf
double MassSpring::computeEnergy(double stiffness)
{
    double sum = 0.;
    unsigned i = 0;
    for (const auto& e : E) {
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        sum += 0.5 * stiffness * std::pow((diff.norm() - l), 2);
        i++;
    }
    return sum;
}

Eigen::MatrixXd MassSpring::computeGrad(double stiffness)
{
    Eigen::MatrixXd g = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    unsigned i = 0;
    for (const auto& e : E) {
        // --------------------------------------------------
        // (HW TODO): Implement the gradient computation
        Eigen::Vector3d x1 = X.row(e.first);
        Eigen::Vector3d x2 = X.row(e.second);
        double length = (x1 - x2).norm();
        double length_rest = E_rest_length[i];

        g.row(e.first) += stiffness * (length - length_rest) * (x1 - x2) / length;
        g.row(e.second) += stiffness * (length - length_rest) * (x2 - x1) / length;
        // --------------------------------------------------
        i++;
    }
    return g;
}

Eigen::SparseMatrix<double> MassSpring::computeHessianSparse(double stiffness)
{
    unsigned n_vertices = X.rows();
    Eigen::SparseMatrix<double> H(n_vertices * 3, n_vertices * 3);

    unsigned i = 0;
    auto k = stiffness;
    const auto I = MatrixXd::Identity(3, 3);
    std::vector<Trip_d> TripletList;
    for (const auto& e : E) {
        // --------------------------------------------------
        // (HW TODO): Implement the sparse version Hessian computation
        // Remember to consider fixed points 
        // You can also consider positive definiteness here
        Eigen::Vector3d x1 = X.row(e.first);
        Eigen::Vector3d x2 = X.row(e.second);
        MatrixXd x_xT = (x1 - x2) * (x1 - x2).transpose();
        double length = (x1 - x2).norm();
        double length_square = pow(length, 2); 
        double length_rest = E_rest_length[i];
        
        auto H_e = k * x_xT / length_square +
                   k * (1 - length_rest / length) * (I - x_xT / length_square);
        double first = 3 * e.first;
        double second = 3 * e.second;

        for (int p = 0; p < 3; ++p) {
            for (int q = 0; q < 3; ++q) {
                if (H_e(p, q)) {
                    TripletList.push_back(Trip_d(first + p, first + q, H_e(p, q)));
                    TripletList.push_back(Trip_d(first + p, second + q, -H_e(p, q)));
                    TripletList.push_back(Trip_d(second + p, first + q, -H_e(p, q)));
                    TripletList.push_back(Trip_d(second + p, second + q, H_e(p, q)));
                }     
            }
        }
        // --------------------------------------------------
        i++;
    }
    H.setFromTriplets(TripletList.begin(), TripletList.end());
    H.makeCompressed();
    return H;
}

Eigen::SparseMatrix<double> MassSpring::makeSPD(const Eigen::SparseMatrix<double>& A)
{
    Eigen::SelfAdjointEigenSolver<SparseMatrix_d> es(A);
    double eigen_values_min = es.eigenvalues().minCoeff();
    if (eigen_values_min >= 1e-10)
        return A;
    size_t size = A.rows();
    double epsilon = 1e-10;
    std::vector<Trip_d> TripletListFix;
    for (size_t i = 0; i < size; ++i) {
        TripletListFix.push_back(Trip_d(i, i, epsilon - eigen_values_min));
    }
    SparseMatrix_d A_fixed = A;  // size = [nx3, nx3]
    A_fixed.setFromTriplets(TripletListFix.begin(), TripletListFix.end());
    return A_fixed;
}
bool MassSpring::checkSPD(const Eigen::SparseMatrix<double>& A)
{
    // Eigen::SimplicialLDLT<SparseMatrix_d> ldlt(A);
    // return ldlt.info() == Eigen::Success;
    Eigen::SelfAdjointEigenSolver<SparseMatrix_d> es(A);
    auto eigen_values = es.eigenvalues();
    return eigen_values.minCoeff() >= 1e-10;
}

void MassSpring::reset()
{
    std::cout << "reset" << std::endl;
    this->X = this->init_X;
    this->vel.setZero();
}

// ----------------------------------------------------------------------------------
// (HW Optional) Bonus part
Eigen::MatrixXd MassSpring::getSphereCollisionForce(Eigen::Vector3d center, double radius)
{
    Eigen::MatrixXd force = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    for (int i = 0; i < X.rows(); i++) {
       // (HW Optional) Implement penalty-based force here 
        Eigen::Vector3d x_c = X.row(i) - center.transpose();
        double length = x_c.norm();
        force.row(i) = collision_penalty_k *
                       std::max(collision_scale_factor * radius - length, 0.) * x_c / length;
    }
    return force;
}
// ----------------------------------------------------------------------------------


}  // namespace USTC_CG::node_mass_spring

