#pragma once

#include <utility>
#include <vector>

#include "imgui.h"

namespace USTC_CG
{
class Warping
{
    public:
        virtual ~Warping() noexcept = default;

        /**
         * Mathematical mapping for image warping.
         * Outputs the coordinates after warping
         * @param x, y Coordinates before warping
         */
        virtual std::pair<int, int> warping(int x, int y) = 0;  
};

} // namespace USTC_CG