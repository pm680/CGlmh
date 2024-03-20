#include "minimal_surface_calculation.h"

namespace USTC_CG
{
void MinSurfCal::set_uniform()
{
    weight_type_ = kUniform;
}
void MinSurfCal::set_cotangent()
{
    weight_type_ = kCotangent;
}
void MinSurfCal::decompsition()
{
    using Eigen::SparseMatrix, Eigen::Triplet, Eigen::VectorXf;
    NSize = mesh_->n_vertices();

    SparseMatrix<float> A(NSize, NSize);
    std::vector<Triplet<float>> triplet_A;
    float sum_weight = 0.f;
    A.setZero();
    for(auto v_it = mesh_->vertices_begin(); v_it != mesh_->vertices_end(); ++v_it)
    {
        triplet_A.push_back(Triplet<float>(v_it->idx(), v_it->idx(), 1.0));
        if(!mesh_->is_boundary(*v_it))
        {
            auto voh_it_sum = mesh_->voh_iter(*v_it);
            for(;voh_it_sum.is_valid();++voh_it_sum) sum_weight += weight(*v_it, *voh_it_sum);

            auto voh_it = mesh_->voh_iter(*v_it);
            for(;voh_it.is_valid();++voh_it)
                triplet_A.push_back(
                    Triplet<float>(v_it->idx(), voh_it->to().idx(),
                                    -weight(*v_it, *voh_it) / sum_weight));
            sum_weight = 0.f;
        }
    }
    A.setFromTriplets(triplet_A.begin(), triplet_A.end());

    solver_.compute(A);
    if(solver_.info() != Eigen::Success)
    {
        std::cerr << "Decompsition failed!" << std::endl;
        exit(solver_.info()); 
    }
}
Eigen::MatrixXf MinSurfCal::solve()
{
    using Eigen::MatrixXf;

    decompsition();

    MatrixXf B(NSize, 3);
    MatrixXf new_pos(NSize, 3);

    B.setZero();
    for(auto v_it = mesh_->vertices_begin(); v_it != mesh_->vertices_end(); ++v_it)
        if(mesh_->is_boundary(*v_it))
            B.row(v_it->idx()) << mesh_->point(*v_it)[0], 
                                  mesh_->point(*v_it)[1], 
                                  mesh_->point(*v_it)[2];

    
    new_pos = solver_.solve(B);
    if(solver_.info() != Eigen::Success)
    {
        std::cerr << "Solving failed!" << std::endl;
        exit(solver_.info());
    }

    return new_pos;
}
float MinSurfCal::weight(OpenMesh::VertexHandle vh_i, OpenMesh::SmartHalfedgeHandle he_j) const
{
    float weight;
    switch (weight_type_)
    {
        case kUniform:
            weight = 1.0;
            break;
        case kCotangent:
            const auto& vh_j = he_j.to();
            const auto& vh_alpha = he_j.next().to();
            const auto& vh_beta = he_j.opp().next().to();
            const auto& vec_alpha1 = original_mesh_->point(vh_i) - original_mesh_->point(vh_alpha);
            const auto& vec_alpha2 = original_mesh_->point(vh_j) - original_mesh_->point(vh_alpha);
            const auto& vec_beta1 = original_mesh_->point(vh_i) - original_mesh_->point(vh_beta);
            const auto& vec_beta2 = original_mesh_->point(vh_j) - original_mesh_->point(vh_beta);

            float cos_alpha = vec_alpha1.dot(vec_alpha2) / (vec_alpha1.norm() * vec_alpha2.norm());
            float cos_beta = vec_beta1.dot(vec_beta2) / (vec_beta1.norm() * vec_beta2.norm());
            float alpha = acosf(cos_alpha);
            float beta = acosf(cos_beta);
            
            weight = 1.0 / tanf(alpha) + 1.0 / tanf(beta);

            break;
    }
    return weight;
}
}