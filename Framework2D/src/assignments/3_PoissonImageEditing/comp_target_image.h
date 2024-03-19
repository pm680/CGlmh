#pragma once

#include "comp_source_image.h"
#include "view/comp_image.h"
#include <Eigen/SparseCholesky>
#include <iostream>
#include <string>

namespace USTC_CG
{
class CompTargetImage : public ImageEditor
{
   public:
    // HW3_TODO: Add more types of cloning
    enum CloneType
    {
        kDefault = 0,
        kPaste = 1,
        kSeamless = 2
    };

    enum Guidance
    {
        kGradient = 0,
        kMixedGradient = 1
    };

    explicit CompTargetImage(
        const std::string& label,
        const std::string& filename);
    virtual ~CompTargetImage() noexcept = default;

    void draw() override;
    // Bind the source image component
    void set_source(std::shared_ptr<CompSourceImage> source);
    // Enable realtime updating
    void set_realtime(bool flag);
    void restore();

    // HW3_TODO: Add more types of cloning
    void set_paste();
    void set_seamless();
    // The clone function
    void clone();

    void set_gradient();
    void set_mixedgradient();
    void seamless_predecomposition();
    float guidance(int px, int py, int qx, int qy, int channel);
    Eigen::MatrixXf seamless_solve();

   private:
    // Store the original image data
    std::shared_ptr<Image> back_up_;
    // Source image
    std::shared_ptr<CompSourceImage> source_image_;
    CloneType clone_type_ = kDefault;

    ImVec2 mouse_position_;
    bool edit_status_ = false;
    bool flag_realtime_updating = false;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> ldlt_;
    Guidance guidance_type_;
};

}  // namespace USTC_CG