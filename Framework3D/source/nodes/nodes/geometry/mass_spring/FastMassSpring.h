#pragma once 
#include "MassSpring.h"
#include <memory>

namespace USTC_CG::node_mass_spring {
// Impliment the Liu13's paper: https://tiantianliu.cn/papers/liu13fast/liu13fast.pdf
class FastMassSpring : public MassSpring {
   public:
    FastMassSpring() = default;
    ~FastMassSpring() = default; 

    FastMassSpring(const Eigen::MatrixXd& X, const EdgeSet& E, const float stiffness);
    void step() override;
    unsigned max_iter = 10; // (HW Optional) add UI for this parameter

    SparseMatrix_d KroneckerProduct_I(const MatrixXd& A);

   protected:
    MatrixXd D;

    SparseMatrix_d L;
    SparseMatrix_d J;

    Eigen::SparseLU<SparseMatrix_d> solver;
    // Custom variables, like prefactorized A 
   
};
}  // namespace USTC_CG::node_mass_spring