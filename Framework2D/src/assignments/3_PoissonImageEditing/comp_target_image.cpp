#include "comp_target_image.h"

#include <cmath>

namespace USTC_CG
{
using uchar = unsigned char;

CompTargetImage::CompTargetImage(
    const std::string& label,
    const std::string& filename)
    : ImageEditor(label, filename)
{
    if (data_)
        back_up_ = std::make_shared<Image>(*data_);
}

void CompTargetImage::draw()
{
    // Draw the image
    ImageEditor::draw();
    // Invisible button for interactions
    ImGui::SetCursorScreenPos(position_);
    ImGui::InvisibleButton(
        label_.c_str(),
        ImVec2(
            static_cast<float>(image_width_),
            static_cast<float>(image_height_)),
        ImGuiButtonFlags_MouseButtonLeft);
    bool is_hovered_ = ImGui::IsItemHovered();
    // When the mouse is clicked or moving, we would adapt clone function to
    // copy the selected region to the target.
    ImGuiIO& io = ImGui::GetIO();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        edit_status_ = true;
        mouse_position_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        clone();
    }
    if (edit_status_)
    {
        mouse_position_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        if (flag_realtime_updating)
            clone();
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            edit_status_ = false;
        }
    }
}

void CompTargetImage::set_source(std::shared_ptr<CompSourceImage> source)
{
    source_image_ = source;
}

void CompTargetImage::set_realtime(bool flag)
{
    flag_realtime_updating = flag;
}

void CompTargetImage::restore()
{
    *data_ = *back_up_;
    update();
}

void CompTargetImage::set_paste()
{
    clone_type_ = kPaste;
}

void CompTargetImage::set_seamless()
{
    clone_type_ = kSeamless;
}
void CompTargetImage::set_gradient()
{
    guidance_type_ = kGradient;
}
void CompTargetImage::set_mixedgradient()
{
    guidance_type_ = kMixedGradient;
}
void CompTargetImage::clone()
{
    // The implementation of different types of cloning
    // HW3_TODO: In this function, you should at least implement the "seamless"
    // cloning labeled by `clone_type_ ==kSeamless`.
    //
    // The realtime updating (update when the mouse is moving) is only available
    // when the checkboard is selected. It is required to improve the efficiency
    // of your seamless cloning to achieve realtime editing. (Use decomposition
    // of sparse matrix before solve the linear system)
    if (data_ == nullptr || source_image_ == nullptr ||
        source_image_->get_region() == nullptr)
        return;
    std::shared_ptr<Image> mask = source_image_->get_region();

    switch (clone_type_)
    {
        case USTC_CG::CompTargetImage::kDefault: break;
        case USTC_CG::CompTargetImage::kPaste:
        {
            restore();

            for (int i = 0; i < mask->width(); ++i)
            {
                for (int j = 0; j < mask->height(); ++j)
                {
                    int tar_x =
                        static_cast<int>(mouse_position_.x) + i -
                        static_cast<int>(source_image_->get_position().x);
                    int tar_y =
                        static_cast<int>(mouse_position_.y) + j -
                        static_cast<int>(source_image_->get_position().y);
                    if (0 <= tar_x && tar_x < image_width_ && 0 <= tar_y &&
                        tar_y < image_height_ && mask->get_pixel(i, j)[0] > 0)
                    {
                        data_->set_pixel(
                            tar_x,
                            tar_y,
                            source_image_->get_data()->get_pixel(i, j));
                    }
                }
            }
            break;
        }
        case USTC_CG::CompTargetImage::kSeamless:
        {
            // You should delete this block and implement your own seamless
            // cloning. For each pixel in the selected region, calculate the
            // final RGB color by solving Poisson Equations.
            Eigen::MatrixXf seamless_pixel = seamless_solve();
            std::vector<unsigned char> seamless_pixelValues(3);

            restore();
            for (int i = 0; i < mask->width(); ++i)
            {
                for (int j = 0; j < mask->height(); ++j)
                {
                    int tar_x =
                        static_cast<int>(mouse_position_.x) + i -
                        static_cast<int>(source_image_->get_position().x);
                    int tar_y =
                        static_cast<int>(mouse_position_.y) + j -
                        static_cast<int>(source_image_->get_position().y);
                    int selected_x = i - static_cast<int>(source_image_->get_position().x);
                    int selected_y = j - static_cast<int>(source_image_->get_position().y);
                    int selected_W = static_cast<int>(source_image_->get_position_end().x) - 
                                        static_cast<int>(source_image_->get_position().x);   
                    int selected_i = selected_x + selected_W * selected_y;
                    if (0 <= tar_x && tar_x < image_width_ && 0 <= tar_y &&
                        tar_y < image_height_ && mask->get_pixel(i, j)[0] > 0)
                    {      
                        for(int channel = 0; channel < 3; ++channel)
                        {
                            seamless_pixelValues[channel] = 
                                static_cast<unsigned char>
                                (seamless_pixel.col(channel)(selected_i));
                        }
                        data_->set_pixel(
                            tar_x,
                            tar_y,
                            seamless_pixelValues);
                    }
                }
            }
            break;
        }
        default: break;
    }

    update();
}
void CompTargetImage::seamless_predecomposition()
{
    using Eigen::SparseMatrix, Eigen::MatrixXf, Eigen::VectorXf, Eigen::SimplicialLDLT;
    using Eigen::Triplet;
    std::shared_ptr<Image> mask = source_image_->get_region();
    int W = mask->width();
    int H = mask->height();
    int x_selected1, y_selected1;
    int x_selected2, y_selected2;
    int W_selected, H_selected, Size_selected;

    x_selected1 = static_cast<int>(source_image_->get_position().x);
    y_selected1 = static_cast<int>(source_image_->get_position().y);
    x_selected2 = static_cast<int>(source_image_->get_position_end().x);
    y_selected2 = static_cast<int>(source_image_->get_position_end().y);
    W_selected = x_selected2 - x_selected1;
    H_selected = y_selected2 - y_selected1;
    Size_selected = W_selected * H_selected;
    
    SparseMatrix<float> A(Size_selected, Size_selected);
    std::vector<Triplet<float>> triplet_A;
    int x, y;

    A.setZero();
    for (int i = 0; i < Size_selected; ++i)
    {   
        x = i % W_selected; 
        y = i / W_selected;
        triplet_A.push_back(Triplet<float>(i, i, 4.0));
        if(x > 0) triplet_A.push_back(Triplet<float>(i, i-1, -1.0));
        if(x < W_selected-1) triplet_A.push_back(Triplet<float>(i, i+1, -1.0));   
        if(y > 0) triplet_A.push_back(Triplet<float>(i, i-W_selected, -1.0));
        if(y < H_selected-1) triplet_A.push_back(Triplet<float>(i, i+W_selected, -1.0));
    }
    A.setFromTriplets(triplet_A.begin(), triplet_A.end());

    ldlt_.compute(A);
    if(ldlt_.info() != Eigen::Success)
    {
        std::cerr << "The coefficient matrix is not positive-definite." << std::endl;
        exit(ldlt_.info()); 
    }
}

Eigen::MatrixXf CompTargetImage::seamless_solve()
{
    using Eigen::SparseMatrix, Eigen::MatrixXf, Eigen::VectorXf, Eigen::SimplicialLDLT;
    std::shared_ptr<Image> mask = source_image_->get_region();
    int W = mask->width();
    int H = mask->height();
    int Size = W * H;
    int x_selected1, y_selected1;
    int x_selected2, y_selected2;
    int W_selected, H_selected, Size_selected;
    int i_selected1;

    x_selected1 = static_cast<int>(source_image_->get_position().x);
    y_selected1 = static_cast<int>(source_image_->get_position().y);
    x_selected2 = static_cast<int>(source_image_->get_position_end().x);
    y_selected2 = static_cast<int>(source_image_->get_position_end().y);
    W_selected = x_selected2 - x_selected1;
    H_selected = y_selected2 - y_selected1;
    Size_selected = W_selected * H_selected;
    i_selected1 = x_selected1 + y_selected1 * W;

    VectorXf b(Size_selected);
    MatrixXf new_pixel(Size_selected, 3);
    int x, y;
    int x_source, y_source;


    new_pixel.setZero();
    for(int channel = 0; channel < 3; ++channel)
    {
        b.setZero();
        for (int i = 0; i < Size_selected; ++i)
        {   
            x = i % W_selected; 
            y = i / W_selected;
            x_source = x_selected1 + x; 
            y_source = y_selected1 + y;
            int tar_x = static_cast<int>(mouse_position_.x) + x;
            int tar_y = static_cast<int>(mouse_position_.y) + y;

            if(x > 0) b(i) += guidance(x_source, y_source, x_source-1, y_source, channel);
            if(x < W_selected-1) b(i) += guidance(x_source, y_source, x_source+1, y_source, channel);
            if(y > 0) b(i) += guidance(x_source, y_source, x_source, y_source-1, channel);
            if(y < H_selected-1) b(i) += guidance(x_source, y_source, x_source, y_source+1, channel);
            if(x == 0) b(i) += static_cast<float>(data_->get_pixel(tar_x-1, tar_y)[channel]);
            if(x == W_selected-1) b(i) += static_cast<float>(data_->get_pixel(tar_x+1, tar_y)[channel]);
            if(y == 0) b(i) += static_cast<float>(data_->get_pixel(tar_x, tar_y-1)[channel]);
            if(y == H_selected-1) b(i) += static_cast<float>(data_->get_pixel(tar_x, tar_y+1)[channel]);
        }
        
        new_pixel.col(channel) = ldlt_.solve(b);
        for(float &pixelValue : new_pixel.col(channel))
        {
            pixelValue = (pixelValue >= 0) ? pixelValue : 0;
            pixelValue = (pixelValue <= 255) ? pixelValue : 255;
        }
    }
    return new_pixel;
}
float CompTargetImage::guidance(int px, int py, int qx, int qy, int channel)
{
    auto source_data = source_image_->get_data();
    switch (guidance_type_)
    {
        case kGradient:
        {
            return static_cast<float>(source_data->get_pixel(px, py)[channel]) 
                    - static_cast<float>(source_data->get_pixel(qx, qy)[channel]); 
            break;
        }
        case kMixedGradient:
        {
            int tar_px = px + static_cast<int>(mouse_position_.x)
                         - static_cast<int>(source_image_->get_position().x);
            int tar_py = py + static_cast<int>(mouse_position_.y)
                         - static_cast<int>(source_image_->get_position().y);
            int tar_qx = qx + static_cast<int>(mouse_position_.x)
                         - static_cast<int>(source_image_->get_position().x);
            int tar_qy = qy + static_cast<int>(mouse_position_.y)
                         - static_cast<int>(source_image_->get_position().y);
            float source_gradient = static_cast<float>(source_data->get_pixel(px, py)[channel]) 
                    - static_cast<float>(source_data->get_pixel(qx, qy)[channel]); 
            float target_gradient = static_cast<float>(data_->get_pixel(tar_px, tar_py)[channel]) 
                    - static_cast<float>(data_->get_pixel(tar_qx, tar_qy)[channel]); 
            return (fabs(source_gradient) > fabs(target_gradient)) ? 
                    source_gradient : target_gradient;
            break;
        }
        default: break;
    }
}
}  // namespace USTC_CG