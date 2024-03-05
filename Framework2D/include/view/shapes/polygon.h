#pragma once

#include "shape.h"
#include <vector>
#include <imgui.h>
#include <iostream>

namespace USTC_CG
{
class Polygon : public Shape
{
   public:
    Polygon() = default;

    // Constructor to initialize a Polygon with start and end coordinates
    Polygon(
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
        vertices_.push_back(ImVec2(end_point_x_, end_point_y_));
        poly_draw_status_ = false;
    }

    virtual ~Polygon() = default;

    // Overrides draw function to implement polygon-specific drawing logic
    void draw(const Config& config) const override;

    // Overrides Shape's update function to adjust the end point during
    // interaction
    void update(float x, float y) override;

   private:
    float start_point_x_, start_point_y_, end_point_x_, end_point_y_;
    std::vector<ImVec2> vertices_;
    bool poly_draw_status_ = false;
};
}  // namespace USTC_CG
