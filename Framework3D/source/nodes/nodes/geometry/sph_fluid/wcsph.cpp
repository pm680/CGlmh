#include "wcsph.h"
#include <iostream>
using namespace Eigen;

namespace USTC_CG::node_sph_fluid {

WCSPH::WCSPH(const MatrixXd& X, const Vector3d& box_min, const Vector3d& box_max)
    : SPHBase(X, box_min, box_max)
{
}

void WCSPH::compute_pressure()
{
    #pragma omp parallel for
    for (int i = 0; i < ps_.particles().size(); ++i) {
        auto& p = ps_.particles()[i];

        p->pressure() = stiffness_ * (pow(p->density() / ps_.density0(), exponent_) - 1);
    }
}
void WCSPH::step()
{
    TIC(step)
    // -------------------------------------------------------------
    // (HW TODO) Follow the instruction in documents and PPT,
    // implement the pipeline of fluid simulation
    // -------------------------------------------------------------

    // Search neighbors, compute density, advect, solve pressure acceleration, etc.
    ps_.assign_particles_to_cells();
    ps_.search_neighbors();

    compute_density();
    
    compute_non_pressure_acceleration();
    
    compute_pressure();
    compute_pressure_gradient_acceleration();

    advect();

    TOC(step)
}
}  // namespace USTC_CG::node_sph_fluid