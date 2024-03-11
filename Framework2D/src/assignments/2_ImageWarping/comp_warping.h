#pragma once

#include "view/comp_image.h"
#include "warping.h"

namespace USTC_CG
{
// Image component for warping and other functions
class CompWarping : public ImageEditor
{
   public:
    explicit CompWarping(const std::string& label, const std::string& filename);
    virtual ~CompWarping() noexcept = default;

    void draw() override;

    enum WarpingAlgorithm {
        IDW,
        RBF,
    };

    void set_idw();
    void set_rbf();

    // Simple edit functions
    void invert();
    void mirror(bool is_horizontal, bool is_vertical);
    void gray_scale();
    void set_warping_algorithm();
    void warping();
    void restore();

    // Point selecting interaction
    void enable_selecting(bool flag);
    void select_points();
    void init_selections();

   private:
    // Store the original image data
    std::shared_ptr<Image> back_up_;
    // The selected point couples for image warping
    std::vector<ImVec2> start_points_, end_points_;

    // The algorithm for image warping
    std::shared_ptr<Warping> warping_alg_;
    WarpingAlgorithm warping_algorithm_;

    ImVec2 start_, end_;
    bool flag_enable_selecting_points_ = false;
    bool draw_status_ = false;

   private:
    // A simple "fish-eye" warping function
    std::pair<int, int> fisheye_warping(int x, int y, int width, int height);
};

}  // namespace USTC_CG