#include "arap.h"

namespace USTC_CG
{
void ARAP::local_isometric_parameterization()
{
    TSize = mesh_->n_faces();

    vector_x.resize(TSize);
    
    #pragma omp parallel for
    for(int idx_f = 0; idx_f < TSize; ++idx_f)
    {
        Eigen::Vector2f x;
        x.setZero();
        auto f = mesh_->face_handle(idx_f);
        auto fh_it = mesh_->fh_iter(f);
        vector_x[idx_f][fh_it->from().idx()] = x;

        auto vec_1 = mesh_->point(fh_it->to()) - mesh_->point(fh_it->from());
        float r1 = vec_1.norm();
        x(0) =  r1;
        vector_x[idx_f][fh_it->to().idx()] = x;

        ++fh_it;++fh_it;
        auto vec_2 = mesh_->point(fh_it->from()) - mesh_->point(fh_it->to()); 
        float r2 = vec_2.norm();
        float cos = vec_1.dot(vec_2) / (r1 * r2);
        float sin = sqrt(1 - pow(cos, 2));
        x(0) = r2 * cos; x(1) = r2 * sin;
        vector_x[idx_f][fh_it->from().idx()] = x;
    }
}

void ARAP::local_phase()
{
    vector_L.clear();
    vector_L.resize(TSize);

    #pragma omp parallel for
    for(int idx_f = 0; idx_f < TSize; ++idx_f)
    {
        auto f = mesh_->face_handle(idx_f);
        Eigen::Matrix2f S;
        S.setZero();
        auto fh_it = mesh_->fh_iter(f);
        for(; fh_it.is_valid(); ++fh_it)
        { 
            int idx_fh = fh_it->idx();
            int idx_vi = fh_it->from().idx();
            int idx_vi1 = fh_it->to().idx();
            S += cotangent(*fh_it) * (uv.row(idx_vi) - uv.row(idx_vi1)).transpose()
                 * (vector_x[idx_f][idx_vi] - vector_x[idx_f][idx_vi1]).transpose();
        }
        Eigen::JacobiSVD<Eigen::Matrix2f> svd(S, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto U = svd.matrixU();
        auto V = svd.matrixV();
        if (U.determinant() * V.determinant() > 0)
            vector_L[idx_f] = U * V.transpose();
        else{
            V.col(0) *= -1;
            vector_L[idx_f] = U * V.transpose();
        }
    }
}
void ARAP::global_phase_decompsition()
{
    using Eigen::SparseMatrix, Eigen::Triplet, Eigen::VectorXf;
    NSize = mesh_->n_vertices();

    SparseMatrix<float> A(NSize, NSize);
    std::vector<Triplet<float>> triplet_A;
    A.setZero();
    for(auto v_it = mesh_->vertices_begin(); v_it != mesh_->vertices_end(); ++v_it)
    {
        float sum_weight = 0.f;
        auto voh_it = mesh_->voh_iter(*v_it);
        for(; voh_it.is_valid(); ++voh_it) 
        {
            float weight = cotangent(*voh_it) + cotangent(voh_it->opp());
            triplet_A.push_back(Triplet<float>(v_it->idx(), voh_it->to().idx(), -weight));
            sum_weight += weight;
        }
        triplet_A.push_back(Triplet<float>(v_it->idx(), v_it->idx(), sum_weight));     
    }
    A.setFromTriplets(triplet_A.begin(), triplet_A.end());
    A.makeCompressed();

    solver_.compute(A);
    if(solver_.info() != Eigen::Success)
    {
        std::cerr << "Decompsition failed!" << std::endl;
        exit(solver_.info()); 
    }
}
void ARAP::global_phase_solve()
{
    using Eigen::MatrixXf;

    MatrixXf B(NSize, 2);

    B.setZero();
    for(auto he_it = mesh_->halfedges_begin(); he_it != mesh_->halfedges_end(); ++he_it)
    {
        if(!mesh_->is_boundary(*he_it))
        {
            int idx_fh = mesh_->face_handle(*he_it).idx();
            int idx_vi = he_it->from().idx();
            int idx_vj = he_it->to().idx();
            auto xi = vector_x[idx_fh][idx_vi];
            auto xj = vector_x[idx_fh][idx_vj];
            B.row(idx_vi) += cotangent(*he_it) * vector_L[idx_fh] * (xi - xj);
            B.row(idx_vj) += cotangent(*he_it) * vector_L[idx_fh] * (xj - xi);
        }
    }

    uv = solver_.solve(B);
    if(solver_.info() != Eigen::Success)
    {
        std::cerr << "Solving failed!" << std::endl;
        exit(solver_.info());
    }
}

pxr::VtArray<pxr::GfVec2f> ARAP::iteration()
{
    local_isometric_parameterization();
    global_phase_decompsition();
    uv.resize(NSize, 2);
    for(auto v_it = mesh_->vertices_begin(); v_it != mesh_->vertices_end(); ++v_it)
    {
        uv(v_it->idx(), 0) = textureCoords_[v_it->idx()][0];
        uv(v_it->idx(), 1) = textureCoords_[v_it->idx()][1];
    }
    for(int i = 0; i < max_iterations_; ++i)
    {
        local_phase();
        global_phase_solve();
    }
    
    float max_0 = uv.col(0).maxCoeff();
    float min_0 = uv.col(0).minCoeff();
    float max_1 = uv.col(1).maxCoeff();
    float min_1 = uv.col(1).minCoeff();
    textureCoords_.clear();
    for (auto v_it = mesh_->vertices_begin(); v_it != mesh_->vertices_end(); ++v_it) 
    {
        uv(v_it->idx(), 0) = (uv(v_it->idx(), 0) - min_0) / (max_0 - min_0);
        uv(v_it->idx(), 1) = (uv(v_it->idx(), 1) - min_1) / (max_1 - min_1);
        pxr::GfVec2f gfTexCoord(uv(v_it->idx(), 0), uv(v_it->idx(), 1));
        textureCoords_.push_back(gfTexCoord);
    }
    
    return textureCoords_;
}

float ARAP::cotangent(OpenMesh::SmartHalfedgeHandle he)
{
    if(mesh_->is_boundary(he)) return 0.f;
    auto vh = he.next().to();
    auto vec_1 = mesh_->point(he.from()) - mesh_->point(vh);
    auto vec_2 = mesh_->point(he.to()) - mesh_->point(vh);

    return vec_1.dot(vec_2) / vec_1.cross(vec_2).norm();
}

void ARAP::set_max_iterations(int max_iterations)
{
    max_iterations_ = max_iterations;
}
}