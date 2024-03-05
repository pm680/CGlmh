#include "view/shapes/polygon.h"


namespace USTC_CG
{
// Draw the polygon using ImGui
void Polygon::draw(const Config& config) const
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

void Polygon::update(float x, float y)
{   
    vertices_.back() = ImVec2(x, y);
    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
        if(!poly_draw_status_) poly_draw_status_ = true;
        else if (vertices_.size() >= 3){
            vertices_.push_back(vertices_[0]);
        }
    }
    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        vertices_.push_back(ImVec2(x, y));
}
}  // namespace USTC_CG