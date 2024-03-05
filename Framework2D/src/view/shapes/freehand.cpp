#include "view/shapes/freehand.h"

#include <math.h>

namespace USTC_CG
{
// freehand using ImGui
void Freehand::draw(const Config& config) const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    draw_list->AddPolyline(
        &vertices_[0],
        (int)vertices_.size(),
        IM_COL32(
            config.line_color[0],
            config.line_color[1],
            config.line_color[2],
            config.line_color[3]),
        ImDrawFlags_None,
        config.line_thickness);
}

void Freehand::update(float x, float y)
{   
    if(vertices_.back().x != x || vertices_.back().y != y) 
        vertices_.push_back(ImVec2(x, y));
}
}  // namespace USTC_CG