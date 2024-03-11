#include "warping_rbf.h"
#include <cmath>
#include <iostream>

namespace USTC_CG
{
std::pair<int, int>
WarpingRBF::warping(int x, int y)
{   
    if(start_points_.empty()) return { x, y };
    const auto N = start_points_.size();
    Eigen::VectorXf Radial_basis(N);
    Eigen::Vector2f point, new_point;

    point << x, y;
    for(int i=0;i<N;++i)
        Radial_basis(i) = radial_basis_func(
            sqrtf(pow(x - start_points_[i].x, 2) + pow(y - start_points_[i].y, 2)));
    
    solve();
    
    new_point = Alpha * Radial_basis + A * point + b;

    return { static_cast<int>(new_point(0)), static_cast<int>(new_point(1)) }; 
}

void WarpingRBF::solve()
{
    using Eigen::MatrixXf;

    const auto N = start_points_.size();
    MatrixXf A_equation(N+3, N+3), b_equation(N+3, 2);

    // Assign values to the coefficient matrices and constant terms of the system of linear equations
    for(int j=0;j<N;++j) {
        A_equation(0, j) = start_points_[j].x;
        A_equation(1, j) = start_points_[j].y;
        A_equation(2, 0) = 1.f;
        for(int i=3;i<N+3;++i) 
            A_equation(i, j) = radial_basis_func(
            sqrtf(pow(start_points_[i-3].x - start_points_[j].x, 2) + 
            pow(start_points_[i-3].y - start_points_[j].y, 2)));
    }
    for(int i = 3;i<N+3;++i) {
        A_equation(i, N) = start_points_[i-3].x;
        A_equation(i, N+1) = start_points_[i-3].y;
        A_equation(i, N+2) = 1.f;
    }

    for(int i = 3;i<N+3;++i) {
        b_equation(i, 0) = end_points_[i-3].x;
        b_equation(i, 1) = end_points_[i-3].y;
    }
    
    MatrixXf Coef = A_equation.colPivHouseholderQr().solve(b_equation);
    
    Alpha = Coef.topRows(N).transpose();
    A << Coef(N, 0), Coef(N+1, 0),
         Coef(N, 1), Coef(N+1, 1);
    b << Coef(N+2, 0), Coef(N+2, 1);
}

float WarpingRBF::radial_basis_func(float d, float r, float mu)
{   
    return pow((pow(d, 2) + pow(r, 2)), mu/2);
}

} // namespace USTC_CG