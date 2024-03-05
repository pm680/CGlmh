#pragma once

#include "shape.h"
#include <vector>
#include <imgui.h>
#include <iostream>

namespace USTC_CG
{
class Freehand : public Shape
{
   public:
    Freehand() = default;

    // Constructor to initialize a Polygon with start and end coordinates
    Freehand(
        float start_point_x,
        float start_point_y,
        float end_point_x,
        float end_point_y)
        : start_point_x_(start_point_x),
          start_point_y_(start_point_y),
          end_point_x_(end_point_x),
          end_point_y_(end_point_y)
    {
        vertices_.push_back(ImVec2(start_point_x_, start_point_y_));
    }

    virtual ~Freehand() = default;

    // Overrides draw function to implement polygon-specific drawing logic
    void draw(const Config& config) const override;

    // Overrides Shape's update function to adjust the end point during
    // interaction
    void update(float x, float y) override;

   private:
    float start_point_x_, start_point_y_, end_point_x_, end_point_y_;
    std::vector<ImVec2> vertices_;
};
}  // namespace USTC_CG
