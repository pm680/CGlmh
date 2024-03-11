#pragma once

#include "warping.h"
#include <Eigen/Dense>

namespace USTC_CG
{
// iamge warping by RBF algorithm
class WarpingRBF : public Warping
{
    public:
        WarpingRBF() = default;
        WarpingRBF(
            const std::vector<ImVec2> start_points, 
            const std::vector<ImVec2> end_points)
            : start_points_(start_points),
              end_points_(end_points){}
        ~WarpingRBF() = default;

        std::pair<int, int> warping(int x, int y) override;

        void solve();
        float radial_basis_func(float d, float r = 1.f, float mu = 2.f);

    private:
        std::vector<ImVec2> start_points_, end_points_;
        Eigen::Matrix2Xf Alpha;
        Eigen::Matrix2f A; 
        Eigen::Vector2f b;
};

} // namespace USTC_CG
