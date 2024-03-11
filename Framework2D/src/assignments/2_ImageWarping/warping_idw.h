#pragma once

#include "warping.h"

namespace USTC_CG
{
// iamge warping by IDW algorithm
class WarpingIDW : public Warping
{
    public:
        WarpingIDW() = default;
        WarpingIDW(
            const std::vector<ImVec2> start_points, 
            const std::vector<ImVec2> end_points)
            : start_points_(start_points),
              end_points_(end_points){}
        ~WarpingIDW() = default;

        std::pair<int, int> warping(int x, int y) override;

    private:
        std::vector<ImVec2> start_points_, end_points_;
};

} // namespace USTC_CG
