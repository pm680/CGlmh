#pragma once

#include "comp_source_image.h"
#include "view/comp_image.h"
#include <Eigen/SparseCholesky>
#include <iostream>

namespace USTC_CG
{
class Seamless
{
    public:
    enum GuidanceType
    {
        kGradient = 0,
        kMixedGradient = 1
    };

    explicit Seamless(
        std::shared_ptr<CompSourceImage>& source_image,
        std::shared_ptr<Image>& data)
        : source_image_(source_image),
          data_(data){}
    virtual ~Seamless() noexcept = default;

    void set_gradient();
    void set_mixed_gradient();
    void set_mouse_position(ImVec2 mouse_position);
    
    void selected_position();

    void predecomposition();
    float guidance(int px, int py, int qx, int qy, int channel);
    Eigen::MatrixXf solve();

    private:
    std::shared_ptr<CompSourceImage> source_image_;
    std::shared_ptr<Image> data_;
    ImVec2 mouse_position_;
    GuidanceType guidance_type_;

    int x_selected1, y_selected1;
    int x_selected2, y_selected2;
    int W_selected, H_selected, Size_selected;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> ldlt_;
};
}