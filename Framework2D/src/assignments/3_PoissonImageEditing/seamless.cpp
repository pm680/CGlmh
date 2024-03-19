#include "seamless.h"

namespace USTC_CG
{
void Seamless::set_gradient()
{
    guidance_type_ = kGradient;
}
void Seamless::set_mixed_gradient()
{
    guidance_type_ = kMixedGradient;
}
void Seamless::set_mouse_position(ImVec2 mouse_position)
{
    mouse_position_ = mouse_position;
}
void Seamless::selected_position()
{
    x_selected1 = static_cast<int>(source_image_->get_position().x);
    y_selected1 = static_cast<int>(source_image_->get_position().y);
    x_selected2 = static_cast<int>(source_image_->get_position_end().x);
    y_selected2 = static_cast<int>(source_image_->get_position_end().y);
    W_selected = x_selected2 - x_selected1;
    H_selected = y_selected2 - y_selected1;
    Size_selected = W_selected * H_selected;
}
void Seamless::predecomposition()
{
    using Eigen::Triplet;
    Eigen::SparseMatrix<float> A(Size_selected, Size_selected);
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

Eigen::MatrixXf Seamless::solve()
{
    Eigen::VectorXf b(Size_selected);
    Eigen::MatrixXf new_pixel(Size_selected, 3);
    int x, y;
    int x_source, y_source;
    int tar_x, tar_y;

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
            tar_x = static_cast<int>(mouse_position_.x) + x;
            tar_y = static_cast<int>(mouse_position_.y) + y;

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
float Seamless::guidance(int px, int py, int qx, int qy, int channel)
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
}