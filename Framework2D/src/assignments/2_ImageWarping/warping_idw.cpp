#include "warping_idw.h"
#include <math.h>

namespace USTC_CG
{
std::pair<int, int>
WarpingIDW::warping(int x, int y)
{   
    if (start_points_.empty()) return { x, y };
    float new_x = 0.f, new_y = 0.f;
    std::vector<float> weight, sigma;
    float d11 = 0.f, d12 = 0.f, d21 = 0.f, d22 = 0.f;
    float Sigma = 0.f;

    for(ImVec2 p : start_points_)
    {
        sigma.push_back(1.0 / (pow(x - p.x, 2) + pow(y - p.y, 2)));
        Sigma += sigma.back();
    }

    for(int i = 0;i < start_points_.size();++i)
    {
        weight.push_back(sigma[i] / Sigma);
        
        new_x += weight.back() * (end_points_[i].x + x - start_points_[i].x);
        new_y += weight.back() * (end_points_[i].y + y - start_points_[i].y);
    }
    
    return { static_cast<int>(new_x), static_cast<int>(new_y) };
}
} // namespace USTC_CG