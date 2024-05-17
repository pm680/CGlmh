#include "iisph.h"
#include <iostream>

namespace USTC_CG::node_sph_fluid {

using namespace Eigen;

IISPH::IISPH(const MatrixXd& X, const Vector3d& box_min, const Vector3d& box_max)
    : SPHBase(X, box_min, box_max)
{
    // (HW TODO) Feel free to modify this part to remove or add necessary member variables
    predict_density_ = VectorXd::Zero(ps_.particles().size());
    predict_vel_ = MatrixXd::Zero(ps_.particles().size(), 3);
    aii_ = VectorXd::Zero(ps_.particles().size());
    Api_ = VectorXd::Zero(ps_.particles().size());
    last_pressure_ = VectorXd::Zero(ps_.particles().size());
    dii_ = MatrixXd::Zero(ps_.particles().size(), 3);
}

void IISPH::step()
{
    // (HW Optional)
    ps_.assign_particles_to_cells();
    ps_.search_neighbors();
    
    compute_density();
    
    compute_non_pressure_acceleration();
    
    predict_advection();
    
    compute_pressure();
    
    compute_pressure_gradient_acceleration();
   
    advect();
}

void IISPH::compute_pressure()
{
    // (HW Optional) solve pressure using relaxed Jacobi iteration 
    // Something like this: 

    double threshold = 0.001;
    for (size_t iter = 0; iter < max_iter_; ++iter) {
        double avg_density_error = pressure_solve_iteration();
        if (avg_density_error < threshold)
            break;
    }
}

void IISPH::predict_advection()
{
    // (HW Optional)
    // predict new density based on non-pressure forces,
    // compute necessary variables for pressure solve, etc. 

    // Note: feel free to remove or add functions based on your need,
    // you can also rename this function. 

    #pragma omp parallel for
    for (int i = 0; i < ps_.particles().size(); ++i) {
        auto& p = ps_.particles()[i];

        predict_vel_.row(i) = p->vel() + p->acceleration_non() * dt_;
    }
    
    #pragma omp parallel for
    for (int i = 0; i < ps_.particles().size(); ++i) {
        auto& p = ps_.particles()[i];

        predict_density_(i) = p->density();
        dii_.row(i).setZero();
        for (auto& q : p->neighbors()) {
            Vector3d grad_ij = grad_W(p->x() - q->x(), ps_.h());

            predict_density_(i) +=
                dt_ * ps_.mass() * (predict_vel_.row(i) - predict_vel_.row(q->idx())).dot(grad_ij);

            dii_.row(i) += ps_.mass() / (p->density() * p->density()) * grad_ij; 
        }
    }

    #pragma omp parallel for
    for (int i = 0; i < ps_.particles().size(); ++i) {
        auto& p = ps_.particles()[i];

        p->pressure() *= 0.5;

        aii_(i) = 0.;
        for (auto& q : p->neighbors()) {
            Vector3d grad_ij = grad_W(p->x() - q->x(), ps_.h());
            Vector3d dji = ps_.mass() / (p->density() * p->density()) * (-grad_ij);
            
            aii_(i) -= ps_.mass() * (dii_.row(i).transpose() - dji).dot(grad_ij);
        }
    }
}

double IISPH::pressure_solve_iteration()
{
    // (HW Optional)   
    // One step iteration to solve the pressure poisson equation of IISPH
    compute_pressure_gradient_acceleration();

    #pragma omp parallel for
    for (int i = 0; i < ps_.particles().size(); ++i) {
        auto& p = ps_.particles()[i];

        Api_(i) = 0;
        for (auto& q : p->neighbors()) {
            Vector3d grad_ij = grad_W(p->x() - q->x(), ps_.h());

            Api_(i) += ps_.mass() * (p->acceleration_p() - q->acceleration_p()).dot(grad_ij);
        }
    }

    #pragma omp parallel for
    for (int i = 0; i < ps_.particles().size(); ++i) {
        auto& p = ps_.particles()[i];

        double bi = (ps_.density0() - predict_density_(i)) / (dt_ * dt_); 
        if (aii_(i))
            p->pressure() += omega_ / aii_(i) * (bi - Api_(i));
        else
            p->pressure() = 0.;
        p->pressure() = std::clamp(p->pressure(), 0.0, 1e+5);
    }
    return 1.0; 
}

// ------------------ helper function, no need to modify ---------------------
void IISPH::reset()
{
    SPHBase::reset();

    predict_density_ = VectorXd::Zero(ps_.particles().size());
    predict_vel_ = MatrixXd::Zero(ps_.particles().size(), 3);
    aii_ = VectorXd::Zero(ps_.particles().size());
    Api_ = VectorXd::Zero(ps_.particles().size());
    last_pressure_ = VectorXd::Zero(ps_.particles().size());
    dii_ = MatrixXd::Zero(ps_.particles().size(), 3);
}
}  // namespace USTC_CG::node_sph_fluid